import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('lekiwi_sensors')
    default_params = os.path.join(package_share, 'param', 'bmi160.yaml')

    i2c_device = LaunchConfiguration('i2c_device')
    i2c_address = LaunchConfiguration('i2c_address')
    frame_id = LaunchConfiguration('frame_id')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'i2c_device',
            default_value='/dev/i2c-1',
            description='Linux I2C device used by the BMI160.',
        ),
        DeclareLaunchArgument(
            'i2c_address',
            default_value='104',
            description='BMI160 7-bit I2C address as a decimal integer (104 is 0x68).',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='TF frame of the physical BMI160 sensor axes.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='100.0',
            description='sensor_msgs/Imu publication rate.',
        ),
        Node(
            package='lekiwi_sensors',
            executable='bmi160_node',
            name='bmi160_node',
            output='screen',
            parameters=[
                default_params,
                {
                    'i2c_device': i2c_device,
                    'i2c_address': ParameterValue(i2c_address, value_type=int),
                    'frame_id': frame_id,
                    'publish_rate_hz': ParameterValue(publish_rate_hz, value_type=float),
                },
            ],
        ),
    ])
