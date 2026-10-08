#!/usr/bin/python3
"""Exercise the installed SLAM/Nav2 stack using a mock base and synthetic scans.

Run after sourcing the built workspace. Uses LOCALHOST and an empty domain,
never starts hardware drivers. Logs/maps are saved outside the repository.
"""
import argparse
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from process_cleanup import cleanup_processes, stop_owned_process


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=94)
    parser.add_argument('--imu', action='store_true', help='also exercise the reference IMU SLAM profile with synthetic stationary IMU')
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error('domain ID must be between 0 and 232')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_LOCALHOST_ONLY='0', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                      FASTDDS_BUILTIN_TRANSPORTS='UDPv4', ROS_STATIC_PEERS='')
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan, Imu
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    from tf2_ros import StaticTransformBroadcaster
    from nav2_msgs.srv import ManageLifecycleNodes
    from cartographer_ros_msgs.srv import WriteState
    from rclpy.parameter_client import AsyncParameterClient
    from ament_index_python.packages import get_package_share_directory

    artifacts = Path(tempfile.mkdtemp(prefix='lekiwi-navigation-mock-'))
    processes = []
    required = {}
    mock_confirmed = False
    rclpy.init()
    node = rclpy.create_node('lekiwi_navigation_mock_test')
    state = {'odom': None, 'map': None, 'cmd': [], 'amcl': None, 'scan_enabled': True}
    scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    command_pub = node.create_publisher(Twist, '/cmd_vel_nav', 10)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    node.create_subscription(Odometry, '/odom', lambda m: state.update(odom=m), 10)
    node.create_subscription(OccupancyGrid, '/map', lambda m: state.update(map=m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Twist, '/cmd_vel', lambda m: state['cmd'].append((time.monotonic(), m)), 100)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: state.update(amcl=m), 10)

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def wait_for(predicate, timeout, message):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            failed = [name for proc, name in required.items() if proc.poll() is not None]
            if failed:
                raise RuntimeError(f"process exited: {failed}; logs: {artifacts}")
            if predicate():
                print(f'[통과] {message}', flush=True)
                return
            rclpy.spin_once(node, timeout_sec=0.02)
        raise RuntimeError(f'timed out: {message}; logs: {artifacts}')

    def spawn(name, command):
        log = (artifacts / f'{name}.log').open('w')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((process, log))
        if name != "map_saver":
            required[process] = name
        return process

    def stop(process):
        required.pop(process, None)
        stop_owned_process(process)

    def request(client, message, timeout=30):
        wait_for(client.service_is_ready, 15, client.srv_name + ' 연결')
        future = client.call_async(message)
        wait_for(future.done, timeout, client.srv_name + ' 응답')
        result = future.result()
        if result is None:
            raise RuntimeError(str(future.exception()))
        return result

    def scan_tick():
        if not state['scan_enabled'] or state['odom'] is None:
            return
        pose = state['odom'].pose.pose
        x, y = pose.position.x, pose.position.y
        q = pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        msg = LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = -math.pi
        msg.angle_increment = 2*math.pi/360
        msg.angle_max = msg.angle_min + 359*msg.angle_increment
        msg.range_min, msg.range_max, msg.scan_time = 0.05, 12.0, 0.1
        ranges = []
        for i in range(360):
            angle = yaw + msg.angle_min + i*msg.angle_increment
            dx, dy = math.cos(angle), math.sin(angle)
            tx = ((2-x) if dx > 0 else (-2-x))/dx if abs(dx) > 1e-9 else 100.0
            ty = ((2-y) if dy > 0 else (-2-y))/dy if abs(dy) > 1e-9 else 100.0
            ranges.append(min(tx, ty))
        msg.ranges = [float('inf')]*360 if state.get('blind', False) else ranges
        scan_pub.publish(msg)

    try:
        pump(6)
        others = [n for n in node.get_node_names() if n != node.get_name()]
        if others:
            raise RuntimeError(f'domain is not empty: {others}')
        base_share = Path(get_package_share_directory('lekiwi_bringup'))
        spawn('base', ['ros2', 'run', 'lekiwi_node', 'lekiwi_node', '--ros-args',
                       '--params-file', str(base_share/'param/base.yaml'),
                       '-p', 'motor_backend:=mock', '-p', 'enable_motor_write:=false',
                       '-p', 'torque_enable:=false', '-p', 'odom_source:=command'])
        wait_for(lambda: state['odom'] is not None, 15, 'mock odom')
        params = AsyncParameterClient(node, '/lekiwi_node')
        wait_for(params.services_are_ready, 10, 'mock 파라미터 서비스')
        future = params.get_parameters(['motor_backend', 'enable_motor_write', 'torque_enable'])
        wait_for(future.done, 10, 'mock/write-disabled/torque-off 확인')
        values = future.result().values
        assert values[0].string_value == 'mock' and not values[1].bool_value and not values[2].bool_value
        mock_confirmed = True
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'base_footprint', 'lidar_link'
        tf.transform.translation.z = 0.2
        tf.transform.rotation.w = 1.0
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(tf)
        timer = node.create_timer(0.1, scan_tick)
        if args.imu:
            imu_tf = TransformStamped()
            imu_tf.header.stamp = node.get_clock().now().to_msg()
            imu_tf.header.frame_id, imu_tf.child_frame_id = 'base_footprint', 'imu_link'
            imu_tf.transform.rotation.w = 1.0
            broadcaster.sendTransform([tf, imu_tf])
            imu_pub = node.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
            def imu_tick():
                msg = Imu()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'
                msg.orientation_covariance[0] = -1.0
                msg.linear_acceleration.z = 9.81
                imu_pub.publish(msg)
            imu_timer = node.create_timer(0.01, imu_tick)
        config = 'lekiwi_2d_imu.lua' if args.imu else 'lekiwi_2d.lua'
        carto = spawn('cartographer', ['ros2', 'launch', 'lekiwi_cartographer', 'cartographer.launch.py',
                                        f'configuration_basename:={config}', 'use_rviz:=false'])
        wait_for(lambda: state['map'] is not None and sum(v >= 0 for v in state['map'].data) > 100,
                 45, 'Cartographer 합성 스캔 지도 생성')
        client = node.create_client(WriteState, '/write_state')
        result = request(client, WriteState.Request(filename=str(artifacts/'map.pbstream'),
                                                    include_unfinished_submaps=True))
        assert result.status.code == 0 and (artifacts/'map.pbstream').stat().st_size > 0, result
        saver = spawn('map_saver', ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                                    '-f', str(artifacts/'map'), '--ros-args', '-p', 'save_map_timeout:=10.0'])
        wait_for(lambda: saver.poll() is not None, 20, '지도 YAML/PGM 저장')
        assert saver.returncode == 0 and (artifacts/'map.yaml').is_file()
        stop(carto)
        spawn('nav2', ['ros2', 'launch', 'lekiwi_navigation2', 'navigation.launch.py',
                       f'map:={artifacts}/map.yaml', 'use_rviz:=false', 'autostart:=false'])
        # Wait for active AMCL by publishing the explicit initial pose until acknowledged.
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.pose.pose.orientation.w = 1.0
        initial.pose.covariance[0] = initial.pose.covariance[7] = 0.01
        initial.pose.covariance[35] = 0.01
        def pose_tick():
            initial.header.stamp = node.get_clock().now().to_msg()
            initial_pub.publish(initial)
        initial_timer = node.create_timer(0.5, pose_tick)
        wait_for(lambda: state['amcl'] is not None, 30, '저장 지도 AMCL 초기 위치 수신')
        node.destroy_timer(initial_timer)
        manager = node.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        result = request(manager, ManageLifecycleNodes.Request(command=0), 45)
        assert result.success, f'Nav2 startup failed; {artifacts}/nav2.log'
        print('[통과] Omni MPPI·Nav2·Collision Monitor 활성화', flush=True)
        action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        wait_for(action.server_is_ready, 15, 'Nav2 goal action 연결')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 0.3
        goal.pose.pose.orientation.w = 1.0
        future = action.send_goal_async(goal)
        wait_for(future.done, 10, 'mock 30cm 목표 수락 응답')
        handle = future.result()
        assert handle.accepted
        result_future = handle.get_result_async()
        wait_for(result_future.done, 45, 'mock 경로 계획·MPPI 목표 주행 완료')
        assert result_future.result().status == GoalStatus.STATUS_SUCCEEDED, result_future.result()
        # No controller goal remains when testing the command pipeline directly.
        state['cmd'].clear()
        command = Twist()
        command.linear.y = 0.04
        command_timer = node.create_timer(0.05, lambda: command_pub.publish(command))
        wait_for(lambda: any(m.linear.y > 0.02 for _, m in state['cmd']), 10,
                 '횡이동 명령 → smoother → collision monitor → mock base')
        assert all(abs(m.linear.y) <= 0.06001 for _, m in state['cmd'])
        for failure in ('no_data', 'all_invalid'):
            state['scan_enabled'], state['blind'] = True, False
            state['cmd'].clear()
            wait_for(lambda: any(m.linear.y > 0.02 for _, m in state['cmd']), 10,
                     f'{failure} 검사 전 정상 명령 회복')
            state['scan_enabled'] = failure != 'no_data'
            state['blind'] = failure == 'all_invalid'
            stale_start = time.monotonic()
            wait_for(lambda: any(t > stale_start + 1.1 and m.linear.y == 0.0 for t, m in state['cmd']),
                     8, f'{failure}: 유효 scan 소실에서 최종 명령 정지')
            pump(0.5)
            assert all(m.linear.y == 0.0 for t, m in state['cmd'] if t > stale_start + 1.5)
        command.linear.y = 0.0
        pump(0.5)
        print(f'[통과] mock 통합 검사 완료. 실물 주행 검증은 별도. 결과: {artifacts}', flush=True)
    finally:
        primary_error = sys.exception()
        if mock_confirmed and rclpy.ok():
            try:
                command_pub.publish(Twist())
            except Exception:
                pass  # Context can close concurrently with SIGTERM; still clean up children.
        errors = cleanup_processes(processes)
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        if errors:
            error = RuntimeError('mock process cleanup failed: ' + '; '.join(map(str, errors)))
            if primary_error is None:
                raise error
            primary_error.add_note(str(error))
            print(error, file=sys.stderr)


if __name__ == '__main__':
    main()
