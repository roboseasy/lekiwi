import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('lekiwi_sensors')

    params_file = LaunchConfiguration('params_file')
    lidar_port = LaunchConfiguration('lidar_port')
    driver_scan_topic = LaunchConfiguration('driver_scan_topic')
    normalized_scan_topic = LaunchConfiguration('normalized_scan_topic')
    use_scan_timing_normalizer = LaunchConfiguration('use_scan_timing_normalizer')
    zero_time_increment = LaunchConfiguration('zero_time_increment')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'param', 'ydlidar.yaml'),
        description='Path to the YDLIDAR ROS 2 parameter file.',
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the YDLIDAR device.',
    )
    driver_scan_topic_arg = DeclareLaunchArgument(
        'driver_scan_topic',
        default_value='/scan_raw',
        description='Raw LaserScan topic published by the YDLIDAR driver.',
    )
    normalized_scan_topic_arg = DeclareLaunchArgument(
        'normalized_scan_topic',
        default_value='/scan',
        description='LaserScan topic after LeKiwi timing normalization.',
    )
    use_scan_timing_normalizer_arg = DeclareLaunchArgument(
        'use_scan_timing_normalizer',
        default_value='true',
        description='Whether to publish normalized /scan from raw YDLIDAR LaserScan.',
    )
    zero_time_increment_arg = DeclareLaunchArgument(
        'zero_time_increment',
        default_value='true',
        description='Whether the normalizer should set LaserScan time_increment to 0.0.',
    )

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, {'port': lidar_port}],
        remappings=[('/scan', driver_scan_topic)],
        namespace='/',
    )
    normalizer_node = Node(
        package='lekiwi_sensors',
        executable='scan_timing_normalizer',
        name='scan_timing_normalizer',
        output='screen',
        parameters=[{
            'input_topic': driver_scan_topic,
            'output_topic': normalized_scan_topic,
            'zero_time_increment': ParameterValue(zero_time_increment, value_type=bool),
        }],
        condition=IfCondition(use_scan_timing_normalizer),
    )

    return LaunchDescription([
        params_file_arg,
        lidar_port_arg,
        driver_scan_topic_arg,
        normalized_scan_topic_arg,
        use_scan_timing_normalizer_arg,
        zero_time_increment_arg,
        driver_node,
        normalizer_node,
    ])
