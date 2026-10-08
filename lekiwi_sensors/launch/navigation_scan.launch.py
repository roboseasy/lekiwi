from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('filter_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('lekiwi_sensors'), 'param', 'navigation_scan.yaml'])),
        Node(package='lekiwi_sensors', executable='navigation_scan_filter',
             name='navigation_scan_filter', output='screen',
             on_exit=[Shutdown(reason='Navigation scan filter exited')], parameters=[
                 LaunchConfiguration('filter_params_file'),
                 {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
