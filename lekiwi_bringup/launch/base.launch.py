import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable

from lekiwi_bringup.base_arguments import BASE_ARGUMENT_DEFAULTS, declare_base_arguments


# A factory profile supplies calibration values unless a non-default launch
# argument was passed for a one-off diagnostic run.
PROFILE_CALIBRATION_DEFAULTS = {
    name: BASE_ARGUMENT_DEFAULTS[name] for name in (
        'speed_tick_scale', 'encoder_tick_deadband', 'encoder_feedback_source',
        'encoder_position_ticks_per_revolution', 'encoder_odom_scale',
    )
}


def apply_profile_calibration(context, *args, **kwargs):
    profile_path = LaunchConfiguration('base_params_file').perform(context)
    with open(profile_path, encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    try:
        parameters = document['lekiwi_node']['ros__parameters']
    except (KeyError, TypeError) as error:
        raise RuntimeError(f'Invalid LeKiwi base profile: {profile_path}') from error
    for name, default in PROFILE_CALIBRATION_DEFAULTS.items():
        if name in parameters and LaunchConfiguration(name).perform(context) == default:
            context.launch_configurations[name] = str(parameters[name])
    return []


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar')
    use_imu = LaunchConfiguration('use_imu')
    imu_i2c_device = LaunchConfiguration('imu_i2c_device')
    imu_i2c_address = LaunchConfiguration('imu_i2c_address')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    driver_scan_topic = LaunchConfiguration('driver_scan_topic')
    normalized_scan_topic = LaunchConfiguration('normalized_scan_topic')
    use_scan_timing_normalizer = LaunchConfiguration('use_scan_timing_normalizer')
    zero_time_increment = LaunchConfiguration('zero_time_increment')
    motor_backend = LaunchConfiguration('motor_backend')
    enable_motor_write = LaunchConfiguration('enable_motor_write')
    serial_port = LaunchConfiguration('serial_port')
    odom_source = LaunchConfiguration('odom_source')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_linear_y = LaunchConfiguration('max_linear_y')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_z = LaunchConfiguration('max_angular_z')
    max_wheel_speed = LaunchConfiguration('max_wheel_speed')
    speed_tick_scale = LaunchConfiguration('speed_tick_scale')
    speed_tick_limit = LaunchConfiguration('speed_tick_limit')
    encoder_tick_deadband = LaunchConfiguration('encoder_tick_deadband')
    encoder_feedback_source = LaunchConfiguration('encoder_feedback_source')
    encoder_position_ticks_per_revolution = LaunchConfiguration(
        'encoder_position_ticks_per_revolution'
    )
    encoder_odom_scale = LaunchConfiguration('encoder_odom_scale')
    torque_enable = LaunchConfiguration('torque_enable')
    log_encoder_reads = LaunchConfiguration('log_encoder_reads')
    monitor_motor_health = LaunchConfiguration('monitor_motor_health')

    base_params_file = LaunchConfiguration('base_params_file')
    robot_description_xacro = PathJoinSubstitution([
        FindPackageShare('lekiwi_description'),
        'urdf',
        'lekiwi.urdf.xacro',
    ])
    geometry_params_file = PathJoinSubstitution([
        FindPackageShare('lekiwi_description'), 'config', 'base_frame.yaml',
    ])
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        robot_description_xacro,
        ' use_hardware_sensors:=true',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': ['wheel_joint_states'],
            'rate': 50,
            'zeros.wrist_roll': -1.5707963267948966,
        }],
    )

    lekiwi_node = Node(
        package='lekiwi_node',
        executable='lekiwi_node',
        name='lekiwi_node',
        output='screen',
        remappings=[('joint_states', 'wheel_joint_states')],
        parameters=[base_params_file, geometry_params_file, {
            'motor_backend': motor_backend,
            'enable_motor_write': ParameterValue(enable_motor_write, value_type=bool),
            'serial_port': serial_port,
            'odom_source': odom_source,
            'max_linear_x': ParameterValue(max_linear_x, value_type=float),
            'max_linear_y': ParameterValue(max_linear_y, value_type=float),
            'max_linear_speed': ParameterValue(max_linear_speed, value_type=float),
            'max_angular_z': ParameterValue(max_angular_z, value_type=float),
            'max_wheel_speed': ParameterValue(max_wheel_speed, value_type=float),
            'speed_tick_scale': ParameterValue(speed_tick_scale, value_type=float),
            'speed_tick_limit': ParameterValue(speed_tick_limit, value_type=int),
            'encoder_tick_deadband': ParameterValue(encoder_tick_deadband, value_type=int),
            'encoder_feedback_source': encoder_feedback_source,
            'encoder_position_ticks_per_revolution': ParameterValue(
                encoder_position_ticks_per_revolution,
                value_type=float,
            ),
            'encoder_odom_scale': ParameterValue(encoder_odom_scale, value_type=float),
            'torque_enable': ParameterValue(torque_enable, value_type=bool),
            'log_encoder_reads': ParameterValue(log_encoder_reads, value_type=bool),
            'monitor_motor_health': ParameterValue(monitor_motor_health, value_type=bool),
        }],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lekiwi_sensors'),
                'launch',
                'ydlidar.launch.py',
            ]),
        ]),
        launch_arguments={
            'params_file': lidar_params_file,
            'lidar_port': lidar_port,
            'driver_scan_topic': driver_scan_topic,
            'normalized_scan_topic': normalized_scan_topic,
            'use_scan_timing_normalizer': use_scan_timing_normalizer,
            'zero_time_increment': zero_time_increment,
        }.items(),
        condition=IfCondition(use_lidar),
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lekiwi_sensors'),
                'launch',
                'bmi160.launch.py',
            ]),
        ]),
        launch_arguments={
            'i2c_device': imu_i2c_device,
            'i2c_address': imu_i2c_address,
            'frame_id': imu_frame_id,
        }.items(),
        condition=IfCondition(use_imu),
    )

    return LaunchDescription([
        *declare_base_arguments(),
        OpaqueFunction(function=apply_profile_calibration),
        robot_state_publisher,
        joint_state_publisher,
        lekiwi_node,
        lidar_launch,
        imu_launch,
    ])
