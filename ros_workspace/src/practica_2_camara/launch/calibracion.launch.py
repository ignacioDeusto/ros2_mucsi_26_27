"""Start the ROS camera calibration interface for a chessboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Configure the pattern dimensions and camera image topic."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'board_size',
            description='Number of interior corners, e.g. 8x6.',
        ),
        DeclareLaunchArgument(
            'square_size',
            description='Measured side of a square in metres.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='ROS topic publishing images to calibrate.',
        ),
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            arguments=[
                '--size', LaunchConfiguration('board_size'),
                '--square', LaunchConfiguration('square_size'),
                '--no-service-check',
            ],
            remappings=[('image', LaunchConfiguration('image_topic'))],
            output='screen',
        ),
    ])
