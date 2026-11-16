"""Launch the USB camera driver with configurable capture settings."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start usb_cam and publish its images in the camera namespace."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Camera device visible inside the container.',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='mjpeg2rgb',
            description='Pixel format supported by usb_cam and the camera.',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Image width in pixels.',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='360',
            description='Image height in pixels.',
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='30.0',
            description='Requested frame rate in frames per second.',
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'image_width': ParameterValue(
                    LaunchConfiguration('image_width'), value_type=int),
                'image_height': ParameterValue(
                    LaunchConfiguration('image_height'), value_type=int),
                'framerate': ParameterValue(
                    LaunchConfiguration('framerate'), value_type=float),
            }],
            output='screen',
        ),
    ])
