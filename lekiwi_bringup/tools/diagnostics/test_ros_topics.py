#!/usr/bin/python3
"""PC/Pi ROS integration checks. Nonzero commands require an exclusive mock base.

Run with a sourced ROS workspace. Sensor-only checks never publish commands.
The JSON report records failures as well as successes; exit code 1 means failure.
"""

import argparse
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def vector(message):
    return [message.linear.x, message.linear.y, message.angular.z]


def close(actual, expected):
    return all(abs(a - b) < 1e-6 for a, b in zip(actual, expected))


def validate_mock(parameters):
    require(parameters['motor_backend'] == 'mock', 'Refusing commands: backend is not mock')
    require(parameters['enable_motor_write'] is False, 'Refusing commands: motor write enabled')
    require(parameters['torque_enable'] is False, 'Refusing commands: torque enabled')
    require(parameters['odom_source'] == 'command', 'Expected mock command odometry')


def scan_details(message):
    require(message.header.frame_id == 'lidar_link', 'Unexpected lidar frame')
    require(math.isfinite(message.range_min) and math.isfinite(message.range_max)
            and 0 <= message.range_min < message.range_max, 'Invalid range bounds')
    require(math.isfinite(message.angle_increment) and message.angle_increment > 0,
            'Invalid scan angle increment')
    require(math.isfinite(message.scan_time) and message.scan_time > 0, 'Invalid scan time')
    require(len(message.ranges) > 0, 'Empty scan')
    valid = [x for x in message.ranges
             if math.isfinite(x) and message.range_min <= x <= message.range_max]
    require(bool(valid), 'No finite ranges within sensor limits')
    return {'frame': message.header.frame_id, 'samples': len(message.ranges),
            'valid_samples': len(valid), 'min_m': min(valid), 'max_m': max(valid),
            'scan_time': message.scan_time}


def image_details(message):
    # Import only for camera checks so teleop/lidar checks need no OpenCV.
    import cv2
    import numpy as np

    require(bool(message.header.frame_id), 'Empty camera frame')
    require(bool(message.data), 'Empty compressed image')
    decoded = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    require(decoded is not None, 'Compressed image decoding failed')
    require(decoded.shape[:2] == (480, 640), f'Unexpected image dimensions: {decoded.shape}')
    return {'frame': message.header.frame_id, 'width': decoded.shape[1],
            'height': decoded.shape[0], 'bytes': len(message.data), 'format': message.format}


def odom_details(message):
    p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
    require(message.header.frame_id == 'odom' and message.child_frame_id == 'base_footprint',
            'Unexpected odometry frames')
    require(all(math.isfinite(x) for x in (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                          v.linear.x, v.linear.y, v.angular.z)), 'Invalid odometry')
    require(abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) < .01,
            'Invalid odometry quaternion')
    return {'frame': message.header.frame_id, 'child_frame': message.child_frame_id,
            'position_xy_m': [p.x, p.y], 'twist': vector(v)}


class Stream:
    def __init__(self, validator=None):
        self.validator = validator
        self.reset()

    def reset(self):
        self.window_start = time.monotonic()
        self.count = 0
        self.first = self.last = self.latest = None
        self.last_stamp = None
        self.stamp_advances = 0
        self.error = None
        self.details = {}
        self.max_receive_gap = self.max_stamp_gap = 0.

    def receive(self, message):
        now = time.monotonic()
        if self.last is not None:
            self.max_receive_gap = max(self.max_receive_gap, now - self.last)
        if self.first is None:
            self.first = now
        self.last, self.latest = now, message
        self.count += 1
        try:
            if hasattr(message, 'header'):
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                require(stamp > 0, 'Zero message timestamp')
                if self.last_stamp is not None:
                    require(stamp > self.last_stamp, 'Non-increasing message timestamp')
                    self.max_stamp_gap = max(self.max_stamp_gap, (stamp - self.last_stamp) / 1e9)
                    self.stamp_advances += 1
                self.last_stamp = stamp
            if self.validator is not None:
                self.details = self.validator(message)
        except Exception as error:
            if self.error is None:
                self.error = str(error)

    def result(self, minimum_hz, minimum_messages=10, maximum_gap=.5):
        now = time.monotonic()
        hz = ((self.count - 1) / (self.last - self.first)
              if self.count > 1 and self.last > self.first else 0.)
        window_hz = self.count / max(now - self.window_start, 1e-9)
        fresh = self.last is not None and now - self.last <= maximum_gap
        receive_gap = max(self.max_receive_gap,
                          now - self.window_start if self.first is None else self.first - self.window_start,
                          now - (self.last if self.last is not None else self.window_start))
        passed = (self.count >= minimum_messages and min(hz, window_hz) >= minimum_hz and fresh
                  and receive_gap <= maximum_gap and self.max_stamp_gap <= maximum_gap
                  and self.error is None and self.stamp_advances >= minimum_messages - 1)
        return {'passed': passed, 'messages': self.count, 'hz': round(hz, 2),
                'window_hz': round(window_hz, 2), 'max_receive_gap_s': receive_gap,
                'max_header_gap_s': self.max_stamp_gap, 'maximum_gap_s': maximum_gap,
                'minimum_hz': minimum_hz, 'fresh': fresh, 'error': self.error,
                'stamp_advances': self.stamp_advances, **self.details}


class TopicChecks:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node('lekiwi_topic_test')
        self.streams = {}
        self.subscriptions = []
        self.topics = {}
        self.add('cmd', Twist, '/cmd_vel')
        self.add('odom', Odometry, '/odom', odom_details)
        if 'lidar' in args.checks:
            self.add('scan', LaserScan, '/scan', scan_details)
            self.add('scan_raw', LaserScan, '/scan_raw', scan_details)
        if 'camera' in args.checks:
            for index in args.camera_indices:
                self.add(f'camera{index}', CompressedImage,
                         f'/camera_{index}/image_raw/compressed', image_details)
                self.add(f'info{index}', CameraInfo, f'/camera_{index}/camera_info')

    def add(self, key, message_type, topic, validator=None):
        stream = Stream(validator)
        self.streams[key] = stream
        self.topics[key] = topic
        self.subscriptions.append(self.node.create_subscription(
            message_type, topic, stream.receive, qos_profile_sensor_data))

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=min(.02, max(0., deadline-time.monotonic())))

    def wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.spin(.02)
        require(predicate(), f'Timeout: {description}')

    def endpoints(self, topic):
        def names(endpoints):
            return [f'{x.node_namespace.rstrip("/")}/{x.node_name}' for x in endpoints]
        return {'publishers': names(self.node.get_publishers_info_by_topic(topic)),
                'subscribers': names(self.node.get_subscriptions_info_by_topic(topic))}

    def controller_parameters(self):
        client = AsyncParameterClient(self.node, '/lekiwi_node')
        require(client.wait_for_services(timeout_sec=self.args.discovery_timeout),
                '/lekiwi_node parameter service unavailable')
        names = ['motor_backend', 'enable_motor_write', 'torque_enable', 'odom_source',
                 'cmd_vel_timeout']
        future = client.get_parameters(names)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.)
        require(future.done() and future.result() is not None, 'Parameter query failed')
        values = future.result().values
        return {names[0]: values[0].string_value, names[1]: values[1].bool_value,
                names[2]: values[2].bool_value, names[3]: values[3].string_value,
                names[4]: values[4].double_value}

    def mock_preflight(self):
        parameters = self.controller_parameters()
        validate_mock(parameters)
        self.cmd_timeout = parameters['cmd_vel_timeout']
        require(0 < self.cmd_timeout <= 2, 'Unexpected command timeout')
        self.wait(lambda: bool(self.streams['odom'].count), self.args.discovery_timeout,
                  'mock odom reception')
        require(self.node.count_publishers('/cmd_vel') == 0, 'Another cmd_vel publisher exists')
        # DDS endpoints can arrive before their ROS node names. Require the full
        # exclusive graph within the discovery window before allowing commands.
        self.wait(lambda: sorted(self.endpoints('/cmd_vel')['subscribers'])
                  == ['/lekiwi_node', '/lekiwi_topic_test'], self.args.discovery_timeout,
                  'exclusive mock cmd_vel subscriber discovery')
        require(self.node.count_publishers('/cmd_vel') == 0, 'Another cmd_vel publisher exists')
        return parameters

    def wait_vector(self, key, expected, since, timeout=1.):
        def matches():
            stream = self.streams[key]
            if stream.last is None or stream.last < since:
                return False
            message = stream.latest.twist.twist if key == 'odom' else stream.latest
            return close(vector(message), expected)
        self.wait(matches, timeout, f'{key}={expected}')

    def teleop(self):
        parameters = self.mock_preflight()
        master, slave = pty.openpty()
        child = None
        checks = []
        try:
            # Exercise the installed executable and real raw-terminal input, not just its mapper.
            executable = Path(get_package_prefix('lekiwi_teleop')) / 'lib/lekiwi_teleop/lekiwi_base_teleop'
            # Share the diagnostic worker's group so service cancellation covers the keyboard too.
            child = subprocess.Popen([str(executable)], stdin=slave, stdout=slave, stderr=slave)
            self.wait(lambda: self.node.count_publishers('/cmd_vel') == 1,
                      self.args.discovery_timeout, 'teleop publisher discovery')
            self.spin(.5)
            for key, expected in [('w', [.05, 0., 0.]), ('s', [-.05, 0., 0.]),
                                  ('a', [0., .05, 0.]), ('d', [0., -.05, 0.]),
                                  ('q', [0., 0., .3]), ('e', [0., 0., -.3])]:
                sent = time.monotonic()
                os.write(master, key.encode())
                self.wait_vector('cmd', expected, sent)
                self.wait_vector('odom', expected, sent)
                # No further key input: the controller must stop on cmd_vel timeout.
                self.spin(self.cmd_timeout + .2)
                self.wait_vector('odom', [0., 0., 0.], sent + self.cmd_timeout)
                stopped = time.monotonic()
                os.write(master, b' ')
                self.wait_vector('cmd', [0., 0., 0.], stopped)
                checks.append({'key': key, 'expected': expected, 'timeout_stop': True,
                               'space_stop': True})
            # Quit while a nonzero command is active to test the exit stop independently.
            sent = time.monotonic()
            os.write(master, b'w')
            self.wait_vector('odom', [.05, 0., 0.], sent)
            exited = time.monotonic()
            os.write(master, b'x')
            self.wait(lambda: child.poll() is not None, 5., 'teleop exit')
            require(child.returncode == 0, f'Teleop exit code {child.returncode}')
            self.wait_vector('cmd', [0., 0., 0.], exited)
            self.wait_vector('odom', [0., 0., 0.], exited)
            result = {'passed': True, 'parameters': parameters, 'keys': checks,
                      'exit_stop': True, 'scope': 'mock keyboard integration'}
        finally:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=3.)
            os.close(master)
            os.close(slave)
        result['guarded_profile'] = self.guarded_profile()
        result['manual_checks'] = ['physical wheel direction', 'physical stop', 'floor travel']
        return result

    def guarded_profile(self):
        """Exercise hold-to-move speed stages and release stops on a mock base."""
        from lekiwi_teleop.guarded_base_teleop import HeldKeySpeedProfile, Motion
        self.wait(lambda: self.node.count_publishers('/cmd_vel') == 0,
                  self.args.discovery_timeout, 'keyboard publisher removal')
        self.mock_preflight()
        publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        profile = HeldKeySpeedProfile()
        records = []
        try:
            self.wait(lambda: publisher.get_subscription_count() == 2,
                      self.args.discovery_timeout, 'profile publisher discovery')
            for stage, key in ((1, 'w'), (2, 'w'), (2, 's'), (2, 'a'),
                               (2, 'd'), (2, 'q'), (2, 'e'), (3, 'w')):
                profile.press(str(stage))
                require(profile.press(key), f'hold key {key} was not accepted')
                sent = time.monotonic()
                for _ in range(22):
                    control = profile.next_motion()
                    msg = Twist()
                    msg.linear.x, msg.linear.y, msg.angular.z = (
                        control.linear_x, control.linear_y, control.angular_z)
                    publisher.publish(msg)
                    self.spin(.05)
                axes = {'w': (1,0,0), 's': (-1,0,0), 'a': (0,1,0), 'd': (0,-1,0),
                        'q': (0,0,1), 'e': (0,0,-1)}
                linear, angular = profile.SPEEDS[stage]
                expected = [axes[key][i] * (linear if i < 2 else angular)
                            for i in range(3)]
                require(profile.control == Motion(*expected), 'Speed ramp did not reach target')
                self.wait_vector('odom', expected, sent)
                released = time.monotonic()
                require(profile.release(key), f'hold key {key} did not release')
                require(profile.next_motion() == Motion(), 'Key release did not stop motion')
                for _ in range(3):
                    publisher.publish(Twist())
                    self.spin(.05)
                self.wait_vector('odom', [0., 0., 0.], released)
                records.append({'key': key, 'stage': profile.speed_stage, 'target': expected,
                                'release_stop': True})
            return {'passed': True, 'steps': records,
                    'scope': 'hold-to-move profile on mock; no GUI focus, hardware readiness or torque test'}
        finally:
            for _ in range(3):
                publisher.publish(Twist())
                self.spin(.05)
            self.node.destroy_publisher(publisher)

    def cmd_vel(self):
        self.wait(lambda: self.node.count_publishers('/cmd_vel') == 0,
                  self.args.discovery_timeout, 'previous publisher removal')
        parameters = self.mock_preflight()
        publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        try:
            self.wait(lambda: publisher.get_subscription_count() == 2,
                      self.args.discovery_timeout, 'cmd_vel subscribers')
            expected = [.04, -.02, .1]
            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = expected
            sent = time.monotonic()
            for _ in range(5):
                publisher.publish(message)
                self.spin(.05)
            self.wait_vector('cmd', expected, sent)
            self.wait_vector('odom', expected, sent)
            endpoints = self.endpoints('/cmd_vel')
            self.spin(self.cmd_timeout + .2)
            self.wait_vector('odom', [0., 0., 0.], sent + self.cmd_timeout + .2)
            stopped = time.monotonic()
            publisher.publish(Twist())
            self.wait_vector('cmd', [0., 0., 0.], stopped)
            return {'passed': True, 'parameters': parameters, 'sent': expected,
                    'received': expected, 'mock_odom_response': True,
                    'timeout_stop': True, 'endpoints': endpoints}
        finally:
            for _ in range(3):
                publisher.publish(Twist())
                self.spin(.03)
            self.node.destroy_publisher(publisher)

    def sensors(self):
        keys = [key for key in self.streams if key not in ('cmd', 'odom')]
        parameters = None
        odom_error = None
        if 'odom' in self.args.checks:
            keys.append('odom')
            try:
                parameters = self.controller_parameters()
            except Exception as error:
                # A missing base must not prevent the independent sensor checks.
                odom_error = str(error)
        # Give absent publishers a bounded discovery window, then report each independently.
        deadline = time.monotonic() + self.args.discovery_timeout
        while not all(self.streams[k].count for k in keys) and time.monotonic() < deadline:
            self.spin(.1)
        for key in keys:
            self.streams[key].reset()
        self.spin(self.args.duration)
        results = {}
        for group, selected, rate in [
            ('lidar', ['scan', 'scan_raw'], 5.),
            ('camera', [f'{kind}{i}' for i in self.args.camera_indices for kind in ('camera', 'info')], 10.),
            ('odom', ['odom'], 10.),
        ]:
            if group not in self.args.checks:
                continue
            topics = {}
            for key in selected:
                stream = self.streams[key]
                details = stream.result(rate)
                if key.startswith('info') and stream.latest is not None:
                    info = stream.latest
                    image = self.streams['camera' + key[4:]].latest
                    details['calibrated'] = bool(info.k[0] > 0)
                    details['matches_image'] = bool(image is not None
                        and info.header.frame_id == image.header.frame_id
                        and (info.width, info.height) == (640, 480))
                    details['passed'] &= details['matches_image']
                details['endpoints'] = self.endpoints(self.topics[key])
                details['passed'] &= bool(details['endpoints']['publishers'])
                topics[self.topics[key]] = details
            manual = {'lidar': ['clear field of view', 'physical obstacle direction and distance'],
                      'camera': ['physical camera identification', 'visual image quality'],
                      'odom': ['distance accuracy against an external measurement']}[group]
            results[group] = {'passed': all(x['passed'] for x in topics.values()),
                              'manual_checks': manual,
                              'observation_seconds': self.args.duration, 'topics': topics}
            if group == 'odom':
                results[group]['parameters'] = parameters
                if odom_error is not None:
                    results[group].update(passed=False, error=odom_error)
        return results


def positive_seconds(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('Expected a finite positive number')
    return value


def save_report(path, report):
    """Publish a complete snapshot so service status never reads a partial JSON write."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checks', nargs='+', choices=['teleop', 'cmd_vel', 'lidar', 'camera', 'odom'],
                        default=['teleop', 'cmd_vel', 'lidar', 'camera'])
    parser.add_argument('--duration', type=positive_seconds, default=30.)
    parser.add_argument('--camera-indices', nargs='+', type=int, choices=[0, 2], default=[0, 2])
    parser.add_argument('--discovery-timeout', type=positive_seconds, default=20.)
    parser.add_argument('--report', type=Path, default=Path('/tmp/lekiwi-topic-test.json'))
    args = parser.parse_args(argv)
    report = {'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
              'host': os.uname().nodename, 'checks': {}, 'passed': False, 'stage': 'discovery',
              'scope': 'mock commands; real sensor reception; excludes camera shutdown'}
    handlers = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'signal {signum}')
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        handlers[sig] = signal.signal(sig, interrupted)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    checks = None
    try:
        save_report(args.report, report)
        checks = TopicChecks(args)
        for name in ('teleop', 'cmd_vel'):
            if name in args.checks:
                report['stage'] = name
                save_report(args.report, report)
                try:
                    report['checks'][name] = getattr(checks, name)()
                except Exception as error:
                    report['checks'][name] = {'passed': False, 'error': str(error)}
                print(f'{name}: {"PASS" if report["checks"][name]["passed"] else "FAIL"}', flush=True)
                save_report(args.report, report)
        if any(name in args.checks for name in ('lidar', 'camera', 'odom')):
            report['stage'] = 'sensors'
            save_report(args.report, report)
            report['checks'].update(checks.sensors())
    except Exception as error:
        report['error'] = str(error)
    except KeyboardInterrupt as error:
        report['error'] = str(error)
        report['cancelled'] = True
    finally:
        if checks is not None:
            checks.node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    report['passed'] = ('error' not in report and all(
        report['checks'].get(name, {}).get('passed', False) for name in args.checks))
    report['stage'] = None
    save_report(args.report, report)
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
