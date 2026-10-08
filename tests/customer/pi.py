"""Virtual shipped Pi: real bringup/Feetech driver, PTY wheels and synthetic sensors.

No physical ports are opened. The serial emulator models perfect wheels in a
4 m square room; sensor axes come from the shipped robot description.
"""

import json
import math
import os
from pathlib import Path
import shutil
import signal
import sys
import threading
import time

import numpy as np
import yaml

from readme_commands import SOURCE, apply_ros_environment, command_starting, ros_command

sys.path.insert(0, str(SOURCE / 'tests/integration'))
from process_cleanup import cleanup_processes, stop_owned_process


class Wheels:
    def __init__(self, parameters, geometry):
        import subprocess
        self.port = Path('/tmp/lekiwi-customer-motor')
        self.state_file = Path('/tmp/lekiwi-customer-wheel-state.json')
        self.log = Path('/evidence/serial.log').open('w')
        arguments = [str(self.state_file), str(self.port),
                     *map(str, parameters['wheel_ids']),
                     *map(str, parameters['wheel_directions']),
                     str(parameters['wheel_radius']), str(parameters['base_radius']),
                     str(parameters['speed_tick_scale']),
                     str(geometry['kinematics_frame_yaw']),
                     f"{geometry['kinematics_frame_x']},{geometry['kinematics_frame_y']}"]
        self.process = subprocess.Popen(['/customer-serial-wheels', *arguments],
                                        stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        deadline = time.monotonic() + 10
        while not (self.port.exists() and self.state_file.exists()):
            if self.process.poll() is not None or time.monotonic() >= deadline:
                self.close()
                raise RuntimeError('virtual actuator did not start; see serial.log')
            time.sleep(0.01)
        run_name = os.environ.get('LEKIWI_CUSTOMER_ACTUATOR_RUN')
        scheduling = Path('/evidence/actuator-scheduling.json')
        while run_name:
            if scheduling.is_file():
                report = json.loads(scheduling.read_text())
                if report.get('run') == run_name and report.get('nice') == -20:
                    break
            if self.process.poll() is not None or time.monotonic() >= deadline:
                self.close()
                raise RuntimeError('virtual actuator priority was not prepared by the host')
            time.sleep(0.01)

    def snapshot(self):
        if self.process.poll() not in (None, 0):
            raise RuntimeError('virtual actuator exited; see serial.log')
        return json.loads(self.state_file.read_text())

    def close(self):
        try:
            stop_owned_process(self.process, interrupt_timeout=5)
        finally:
            self.log.close()



def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])


def main():
    apply_ros_environment()
    import subprocess
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan
    from tf2_ros import Buffer, TransformListener, TransformException
    from ament_index_python.packages import get_package_share_directory

    evidence = Path('/evidence')
    shutil.copyfile(Path.home() / 'installation.json', evidence / 'installation.json')
    profile = Path.home() / 'lekiwi_ws/base_profile.yaml'
    shutil.copyfile(SOURCE / 'lekiwi_bringup/param/base_profile.example.yaml', profile)
    parameters = yaml.safe_load(profile.read_text())['lekiwi_node']['ros__parameters']
    geometry = yaml.safe_load((Path(get_package_share_directory('lekiwi_description')) /
                              'config/base_frame.yaml').read_text())['lekiwi_node']['ros__parameters']
    wheels = Wheels(parameters, geometry)
    processes = []

    def spawn(name, command):
        log = (evidence / (name + '.log')).open('w')
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((proc, log))
        return proc

    rclpy.init()
    node = rclpy.create_node('customer_virtual_pi_sensors')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    scan_pub = node.create_publisher(LaserScan, '/scan_raw', qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
    ending = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: ending.set())
    signal.signal(signal.SIGINT, lambda *_: ending.set())
    try:
        command = command_starting('ros2 launch lekiwi_bringup robot.launch.py').replace(
            '/dev/serial/by-id/실제-모터-포트', str(wheels.port))
        # Only physical sensor access is replaced. Retain the real encoder backend,
        # factory profile, robot TF, command timeouts, motor services and safety code.
        command += ' use_lidar:=false use_imu:=false'
        spawn('robot', ros_command(command))
        spawn('normalizer', ['ros2', 'run', 'lekiwi_sensors', 'scan_timing_normalizer'])
        (evidence / 'substitutions.json').write_text(json.dumps({
            'robot_command': command,
            'physical_sensors': 'Synthetic /scan_raw and /imu/data_raw; actual normalizer and URDF TF.',
            'motor_port': 'Container-local PTY, with register-level wheel/torque/encoder emulation.',
            'runtime': os.environ['LEKIWI_CUSTOMER_EXECUTION'],
            'virtual_actuator': 'Native static emulator avoids QEMU/Python reply latency; driver deadlines are unchanged.',
        }, indent=2) + '\n')
        last_scan = last_report = 0.0
        angles = np.arange(360) * (2 * math.pi / 360) - math.pi
        scan_vectors = np.stack((np.cos(angles), np.sin(angles), np.zeros(360)))
        transforms = None
        while not ending.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            if any(proc.poll() is not None for proc, _ in processes):
                raise RuntimeError('Pi runtime child exited')
            state = wheels.snapshot()
            if state['error']:
                raise RuntimeError(state['error'])
            if transforms is None:
                try:
                    transforms = [buffer.lookup_transform('base_footprint', frame, rclpy.time.Time()).transform
                                  for frame in ('lidar_link', 'imu_link')]
                except TransformException:
                    continue
            now = time.monotonic()
            imu = Imu()
            imu.header.stamp = node.get_clock().now().to_msg()
            imu.header.frame_id = 'imu_link'
            imu.orientation_covariance[0] = -1.0
            gravity = rotation(transforms[1].rotation).T @ np.array([0, 0, 9.81])
            angular = rotation(transforms[1].rotation).T @ np.array([0, 0, state['yaw_rate']])
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, gravity)
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, angular)
            imu_pub.publish(imu)
            if now - last_scan >= 0.1:
                last_scan = now
                x, y, yaw = state['pose']
                c, s = math.cos(yaw), math.sin(yaw)
                body_rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                transform = transforms[0]
                origin = np.array([x, y, 0]) + body_rotation @ np.array([
                    transform.translation.x, transform.translation.y, transform.translation.z])
                scan_rotation = body_rotation @ rotation(transform.rotation)
                scan = LaserScan()
                scan.header.stamp = node.get_clock().now().to_msg()
                scan.header.frame_id = 'lidar_link'
                scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 360
                scan.angle_max = scan.angle_min + 359 * scan.angle_increment
                scan.range_min, scan.range_max, scan.scan_time = 0.05, 12.0, 0.1
                scan.time_increment = 0.1 / 360
                directions = scan_rotation @ scan_vectors
                dx, dy = directions[0], directions[1]
                with np.errstate(divide='ignore', invalid='ignore'):
                    tx = np.where(np.abs(dx) > 1e-9,
                                  (np.where(dx > 0, 2, -2) - origin[0]) / dx, 100)
                    ty = np.where(np.abs(dy) > 1e-9,
                                  (np.where(dy > 0, 2, -2) - origin[1]) / dy, 100)
                scan.ranges = np.minimum(tx, ty).tolist()
                scan_pub.publish(scan)
            if now - last_report >= 1:
                last_report = now
                (evidence / 'wheels.json').write_text(json.dumps(state, indent=2) + '\n')
    finally:
        errors = cleanup_processes(processes)
        wheels.close()
        (evidence / 'wheels-final.json').write_text(json.dumps(wheels.snapshot(), indent=2) + '\n')
        node.destroy_node()
        rclpy.try_shutdown()
        if errors:
            raise RuntimeError(f'Pi cleanup failed: {errors}')


if __name__ == '__main__':
    main()
