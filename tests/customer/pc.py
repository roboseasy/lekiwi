"""Customer README journey on the PC, using a separate ARM64 Pi over DDS.

Starts the published GUI/launch commands. Keyboard input is automated on Xvfb;
initial pose and goal use the same ROS interfaces as RViz. No product parameter
or motor-guard check is relaxed for this test.
"""

import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from readme_commands import SOURCE, apply_ros_environment, command_starting, ros_command
from gui_input import focus_tk_window, key_event, visible_windows
from lifecycle_ready import LifecycleReadiness

sys.path.insert(0, str(SOURCE / 'tests/integration'))
from process_cleanup import cleanup_processes, stop_owned_process


def main():
    apply_ros_environment()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rclpy.action import ActionClient
    from rclpy.parameter_client import AsyncParameterClient
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from sensor_msgs.msg import LaserScan, Imu
    from std_msgs.msg import Bool
    from tf2_ros import Buffer, TransformListener

    evidence = Path('/evidence')
    shutil.copyfile(Path.home() / 'installation.json', evidence / 'installation.json')
    processes, required, checks = [], {}, []
    result = {'success': False, 'checks': checks, 'source_commit': os.environ['SOURCE_COMMIT'],
              'pc': os.environ['LEKIWI_CUSTOMER_PC_ARCH'], 'pi': os.environ['LEKIWI_CUSTOMER_PI_ARCH'],
              'execution': os.environ['LEKIWI_CUSTOMER_EXECUTION'],
              'discovery': os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'],
              'hardware': 'virtual PTY wheels and synthetic LiDAR/IMU; no host devices',
              'rviz_input': 'Initial pose and goal use ROS interfaces; teleop keys use Xvfb.'}
    rclpy.init()
    node = rclpy.create_node('customer_pc_acceptance')
    state = {'odom': None, 'scan': None, 'imu': None, 'map': None, 'ready': None, 'amcl': None}
    counts = {'odom': 0, 'scan': 0, 'imu': 0}

    def receive(key, message):
        state[key] = message
        if key in counts:
            counts[key] += 1

    node.create_subscription(Odometry, '/odom', lambda m: receive('odom', m), qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', lambda m: receive('scan', m), qos_profile_sensor_data)
    node.create_subscription(Imu, '/imu/data_raw', lambda m: receive('imu', m), qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/map', lambda m: receive('map', m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Bool, '/motor_ready', lambda m: state.update(ready=m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: receive('amcl', m), 10)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def spawn(name, command, monitor=True):
        log = (evidence / (name + '.log')).open('w')
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((proc, log))
        if monitor:
            required[proc] = name
        return proc

    def wait_for(predicate, timeout, message):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            failures = [name for proc, name in required.items() if proc.poll() is not None]
            if failures:
                raise RuntimeError(f'PC runtime exited: {failures}')
            if predicate():
                checks.append(message)
                print('[PASS] ' + message, flush=True)
                return
            rclpy.spin_once(node, timeout_sec=0.02)
        raise RuntimeError('timed out: ' + message)

    def pump(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def stop(proc):
        required.pop(proc, None)
        stop_owned_process(proc)

    gui_windows = visible_windows

    def stopped():
        twist = state['odom'].twist.twist
        return math.hypot(twist.linear.x, twist.linear.y) < 0.005 and abs(twist.angular.z) < 0.05

    try:
        # A real X server runs the README's RViz and Tk windows remotely.
        os.environ.update(DISPLAY=':99', QT_X11_NO_MITSHM='1', LIBGL_ALWAYS_SOFTWARE='1')
        spawn('display', ['Xvfb', ':99', '-screen', '0', '1280x800x24', '-nolisten', 'tcp'])
        wait_for(lambda: Path('/tmp/.X11-unix/X99').exists(), 10, 'virtual PC desktop ready')
        wait_for(lambda: all(state[key] is not None for key in ('odom', 'scan', 'imu', 'ready')),
                 90, 'Pi /odom /scan /imu/data_raw /motor_ready received across network')
        assert state['ready'] is False
        for key in ('odom', 'scan', 'imu'):
            stamp = state[key].header.stamp
            age = (node.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec) / 1e9
            assert -0.1 <= age < 1.0, (key, age)
        assert state['scan'].time_increment == 0.0
        wait_for(lambda: buffer.can_transform('odom', 'lidar_link', rclpy.time.Time()) and
                 buffer.can_transform('base_footprint', 'imu_link', rclpy.time.Time()),
                 20, 'shipped robot description sensor TF received across network')
        params = AsyncParameterClient(node, '/lekiwi_node')
        wait_for(params.services_are_ready, 20, 'Pi base parameter services discovered from PC')
        future = params.get_parameters(['motor_backend', 'enable_motor_write', 'odom_source', 'torque_enable'])
        wait_for(future.done, 20, 'Pi factory backend parameters read from PC')
        values = future.result().values
        assert values[0].string_value == 'feetech' and values[1].bool_value
        assert values[2].string_value == 'encoder' and not values[3].bool_value
        start_counts = dict(counts)
        pump(5)
        result['topic_hz'] = {key: (counts[key] - start_counts[key]) / 5 for key in counts}
        assert result['topic_hz']['odom'] > 20 and result['topic_hz']['scan'] > 5

        carto = spawn('cartographer', ros_command(command_starting('ros2 launch lekiwi_cartographer')))
        wait_for(lambda: state['map'] is not None and sum(value >= 0 for value in state['map'].data) > 100,
                 90, 'default BMI160 Cartographer creates map from Pi sensors')
        wait_for(lambda: bool(gui_windows('RViz')), 20, 'README SLAM RViz window opens')
        teleop = spawn('teleop', ros_command(command_starting('ros2 run lekiwi_teleop lekiwi_guarded_base_teleop')))
        wait_for(lambda: state['ready'] is True and bool(gui_windows('^LeKiwi ')), 30,
                 'README guarded teleop GUI arms the virtual Pi wheels')
        # Tk also creates an unmapped application-leader window. Select the
        # visible control window, after its post-arm event bindings are ready.
        pump(1)
        window = gui_windows('^LeKiwi ')[0]
        input_window, children = focus_tk_window(window)
        pump(0.2)
        with (evidence / 'teleop-window.txt').open('w') as log:
            log.write(children + '\n')
            log.flush()
            subprocess.run(['xwininfo', '-root', '-tree'], stdout=log, check=True)
            subprocess.run(['xdotool', 'getwindowfocus', 'getwindowname'], stdout=log, check=True)
        before = state['odom'].pose.pose.position
        before_xy = (before.x, before.y)
        key_event(input_window, 'w', pressed=True)
        try:
            pump(4)
        finally:
            key_event(input_window, 'w', pressed=False)
        wait_for(stopped, 5, 'releasing teleop key stops encoder-measured motion')
        after = state['odom'].pose.pose.position
        travelled = math.hypot(after.x - before_xy[0], after.y - before_xy[1])
        assert travelled > 0.05, travelled
        result['teleop_distance_m'] = travelled
        saver = spawn('map_saver', ['bash', '-ec', command_starting('mkdir -p ~/maps/lekiwi')], monitor=False)
        wait_for(lambda: saver.poll() is not None, 30, 'README map_saver finishes')
        assert saver.returncode == 0
        map_dir = Path.home() / 'maps/lekiwi'
        assert (map_dir / 'map.yaml').is_file() and (map_dir / 'map.pgm').stat().st_size > 100
        shutil.copytree(map_dir, evidence / 'maps', dirs_exist_ok=True)
        wait_for(lambda: buffer.can_transform('map', 'base_footprint', rclpy.time.Time()), 20,
                 'SLAM provides map-to-robot transform before saving initial pose')
        transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        key_event(input_window, 'Escape', pressed=True)
        required.pop(teleop, None)
        wait_for(lambda: teleop.poll() is not None and state['ready'] is False, 15,
                 'Esc closes teleop and disarms the virtual Pi wheels')
        assert teleop.returncode == 0
        stop(carto)
        pump(5)

        spawn('nav2', ros_command(command_starting('ros2 launch lekiwi_navigation2')))
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.pose.pose.position.x = transform.translation.x
        initial.pose.pose.position.y = transform.translation.y
        initial.pose.pose.orientation = transform.rotation
        initial.pose.covariance[0] = initial.pose.covariance[7] = 0.01
        initial.pose.covariance[35] = 0.01

        def pose_tick():
            initial.header.stamp = node.get_clock().now().to_msg()
            initial_pub.publish(initial)

        timer = node.create_timer(0.5, pose_tick)
        wait_for(lambda: state['amcl'] is not None, 60, 'saved map AMCL accepts RViz-equivalent initial pose')
        node.destroy_timer(timer)
        wait_for(lambda: state['ready'] is True and 'Nav2 motor guard active' in
                 (evidence / 'nav2.log').read_text(), 90,
                 'README default Nav2 auto-start and motor guard arm encoder base across network')
        wait_for(lambda: bool(gui_windows('RViz')), 20, 'README Nav2 RViz window opens')
        lifecycle = LifecycleReadiness(node)
        wait_for(lifecycle.ready, 60, 'all localization and navigation lifecycle nodes are active')
        result['navigation_lifecycle_states'] = dict(lifecycle.states)
        action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        wait_for(action.server_is_ready, 30, 'Nav2 goal server ready without manual lifecycle Startup')
        q = transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = transform.translation.x + 0.4 * math.cos(yaw)
        goal.pose.pose.position.y = transform.translation.y + 0.4 * math.sin(yaw)
        goal.pose.pose.orientation = transform.rotation
        before = state['odom'].pose.pose.position
        before_xy = (before.x, before.y)
        future = action.send_goal_async(goal)
        wait_for(future.done, 20, 'RViz-equivalent Nav2 goal response received')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('Nav2 rejected the goal after lifecycle activation')
        finished = handle.get_result_async()
        wait_for(finished.done, 90, 'Nav2 drives virtual encoder wheels to goal on saved map')
        status = finished.result().status
        assert status == GoalStatus.STATUS_SUCCEEDED, status
        after = state['odom'].pose.pose.position
        result['nav2_distance_m'] = math.hypot(after.x - before_xy[0], after.y - before_xy[1])
        assert result['nav2_distance_m'] > 0.1
        result['goal_status'] = 'STATUS_SUCCEEDED'
        nav2 = next(proc for proc, name in required.items() if name == 'nav2')
        stop(nav2)
        wait_for(lambda: state['ready'] is False and stopped(), 15,
                 'stopping README Nav2 disarms Pi wheels and stops encoder motion')
        result['success'] = True
    except BaseException as error:
        result['error'] = repr(error)
        raise
    finally:
        errors = cleanup_processes(processes)
        if errors:
            result['success'] = False
            result['cleanup_errors'] = [repr(error) for error in errors]
        result['observed_topic_counts'] = counts
        (evidence / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        rclpy.try_shutdown()
        if errors:
            raise RuntimeError(f'PC cleanup failed: {errors}')


if __name__ == '__main__':
    main()
