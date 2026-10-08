from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from lekiwi_bringup.base_arguments import base_launch_arguments, declare_base_arguments


def validate_mode(context, *args, **kwargs):
    mode_value = LaunchConfiguration('mode').perform(context)
    if mode_value != 'base':
        raise RuntimeError(
            f'Unsupported LeKiwi mode "{mode_value}". This branch currently supports only mode:=base.'
        )
    return []


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='base',
        description='LeKiwi bringup mode. This branch currently supports only base.',
    )
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'launch', 'base.launch.py',
            ]),
        ]),
        launch_arguments=base_launch_arguments().items(),
    )
    return LaunchDescription([
        mode_arg,
        *declare_base_arguments(),
        OpaqueFunction(function=validate_mode),
        base_launch,
    ])
