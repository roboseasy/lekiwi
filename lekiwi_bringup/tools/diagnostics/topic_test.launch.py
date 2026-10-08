"""Real sensors and a mock base for test_ros_topics.py; no motor serial access."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, TimerAction
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = Path(get_package_share_directory('lekiwi_bringup')) / 'launch/robot.launch.py'
    actions = [
        DeclareLaunchArgument('duration', default_value='600.0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('camera_0_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_2_device', default_value='/dev/video2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup)),
            launch_arguments={
                'use_lidar': 'true',
                'use_imu': 'false',
                'motor_backend': 'mock',
                'enable_motor_write': 'false',
                'torque_enable': 'false',
                'odom_source': 'command',
                'lidar_port': LaunchConfiguration('lidar_port'),
            }.items(),
        ),
    ]
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('lekiwi_sensors')) / 'launch/cameras.launch.py')),
        launch_arguments={f'camera_{i}_device': LaunchConfiguration(f'camera_{i}_device')
                          for i in (0, 2)}.items(),
    ))
    actions.append(TimerAction(
        period=LaunchConfiguration('duration'),
        actions=[EmitEvent(event=Shutdown(reason='Topic test time limit'))],
    ))
    return LaunchDescription(actions)
