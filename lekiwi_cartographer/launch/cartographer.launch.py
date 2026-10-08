import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_prefix = get_package_share_directory('lekiwi_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    imu_topic = LaunchConfiguration('imu_topic')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    use_rviz = LaunchConfiguration('use_rviz')

    preflight_log = LogInfo(
        msg=[
            'LeKiwi Cartographer 사전 확인: base bringup은 /scan, /odom, TF ',
            'odom->base_footprint->base_link->soarm_base_link->lidar_link를 publish해야 합니다. ',
            'Cartographer는 map->odom을 publish합니다.',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=os.path.join(cartographer_prefix, 'config'),
            description='Cartographer lua config directory.',
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='lekiwi_2d_imu.lua',
            description='Cartographer lua config filename; standard LeKiwi uses BMI160.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='sensor_msgs/Imu topic remapped to Cartographer input imu.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='시뮬레이션 시간을 사용할지 여부.',
        ),
        preflight_log,
        DeclareLaunchArgument('scan_topic', default_value='/scan_navigation'),
        DeclareLaunchArgument('use_scan_filter', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('lekiwi_sensors'), 'launch', 'navigation_scan.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(LaunchConfiguration('use_scan_filter')),
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            arguments=[
                '-configuration_directory',
                cartographer_config_dir,
                '-configuration_basename',
                configuration_basename,
            ],
            remappings=[('imu', imu_topic), ('scan', LaunchConfiguration('scan_topic'))],
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='OccupancyGrid cell resolution.',
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='OccupancyGrid publish period.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(cartographer_prefix, 'launch', 'occupancy_grid.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec,
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(cartographer_prefix, 'rviz', 'slam.rviz')],
            condition=IfCondition(use_rviz),
        ),
    ])
