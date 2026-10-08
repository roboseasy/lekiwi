"""Two independently selectable USB cameras, using the verified 640x480 MJPEG profile."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    actions = []
    for index in (0, 2):
        actions += [
            DeclareLaunchArgument(f'use_camera_{index}', default_value='true'),
            DeclareLaunchArgument(f'camera_{index}_device', default_value=f'/dev/video{index}'),
            Node(package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
                 namespace=f'camera_{index}', output='screen',
                 # Keep TBB loaded until process exit; image transport teardown can
                 # otherwise unload it while worker threads are still running.
                 additional_env={'LD_PRELOAD': [
                     EnvironmentVariable('LD_PRELOAD', default_value=''),
                     ' libtbb.so.12',
                 ]},
                 condition=IfCondition(LaunchConfiguration(f'use_camera_{index}')),
                 parameters=[{
                     'video_device': LaunchConfiguration(f'camera_{index}_device'),
                     'pixel_format': 'mjpeg2rgb', 'av_device_format': 'YUV422P',
                     'image_width': 640, 'image_height': 480, 'framerate': 30.,
                     'io_method': 'mmap', 'frame_id': f'usb_camera_{index}_optical_frame',
                     'camera_name': f'usb_camera_{index}', 'brightness': -1, 'focus': -1,
                 }]),
        ]
    return LaunchDescription(actions)
