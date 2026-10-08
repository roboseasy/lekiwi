"""Shared bringup argument defaults and forwarding for the standard LeKiwi."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


BASE_ARGUMENTS = (
    ('base_params_file', PathJoinSubstitution([
        FindPackageShare('lekiwi_bringup'), 'param', 'base.yaml']),
     'Per-robot base parameters, including wheel IDs and directions.'),
    ('use_lidar', 'true',
     'Start the LeKiwi lidar launch file.'),
    ('use_imu', 'true',
     'Start the Bosch BMI160 publisher on the standard LeKiwi.'),
    ('imu_i2c_device', '/dev/i2c-1',
     'Linux I2C device used by the BMI160.'),
    ('imu_i2c_address', '104',
     'BMI160 7-bit I2C address as a decimal integer (104 is 0x68).'),
    ('imu_frame_id', 'imu_link',
     'TF frame of the physical BMI160 sensor axes.'),
    ('lidar_port', '/dev/ttyUSB0',
     'Serial port for the YDLIDAR device.'),
    ('lidar_params_file', PathJoinSubstitution([
        FindPackageShare('lekiwi_sensors'), 'param', 'ydlidar.yaml']),
     'Path to the YDLIDAR ROS 2 parameter file.'),
    ('driver_scan_topic', '/scan_raw',
     'Raw LaserScan topic published by the YDLIDAR driver.'),
    ('normalized_scan_topic', '/scan',
     'LaserScan topic after LeKiwi timing normalization.'),
    ('use_scan_timing_normalizer', 'true',
     'Whether to publish normalized /scan from raw YDLIDAR LaserScan.'),
    ('zero_time_increment', 'true',
     'Whether the normalizer should set LaserScan time_increment to 0.0.'),
    ('motor_backend', 'feetech',
     'Base motor backend on the standard LeKiwi; use mock for offline tests.'),
    ('enable_motor_write', 'true',
     'Open the Feetech motor bus; wheel torque remains disabled until explicitly enabled.'),
    ('serial_port', '/dev/ttyACM0',
     'Serial port for the Feetech base motor bus.'),
    ('odom_source', 'encoder',
     'Odometry source: command or encoder.'),
    ('max_linear_x', '0.11',
     'Maximum forward or reverse body velocity in m/s.'),
    ('max_linear_y', '0.1',
     'Maximum lateral body velocity in m/s.'),
    ('max_linear_speed', '0.11',
     'Maximum planar body speed in m/s.'),
    ('max_angular_z', '0.75',
     'Maximum body yaw rate in rad/s.'),
    ('max_wheel_speed', '2.3009711818284617',
     'Maximum wheel angular speed passed to the base controller.'),
    ('speed_tick_scale', '651.8986469044033',
     'Feetech goal-speed ticks per wheel rad/s.'),
    ('speed_tick_limit', '1500',
     'Maximum Feetech goal-speed tick magnitude.'),
    ('encoder_tick_deadband', '50',
     'Raw Feetech speed feedback deadband in ticks.'),
    ('encoder_feedback_source', 'position',
     'Feetech encoder feedback source: position or speed.'),
    ('encoder_position_ticks_per_revolution', '4096.0',
     'Feetech present-position ticks per wheel revolution.'),
    ('encoder_odom_scale', '1.0',
     'Scale applied to decoded encoder wheel speeds for odometry calibration.'),
    ('torque_enable', 'false',
     'Enable Feetech wheel torque after base motor initialization.'),
    ('log_encoder_reads', 'false',
     'Log raw Feetech encoder read packets for stationary odometry diagnostics.'),
    ('monitor_motor_health', 'false',
     'Check Feetech motor voltage while powered.'),
)

BASE_ARGUMENT_DEFAULTS = {name: default for name, default, _description in BASE_ARGUMENTS}


def declare_base_arguments():
    return [DeclareLaunchArgument(name, default_value=default, description=description)
            for name, default, description in BASE_ARGUMENTS]


def base_launch_arguments():
    return {name: LaunchConfiguration(name) for name in BASE_ARGUMENT_DEFAULTS}
