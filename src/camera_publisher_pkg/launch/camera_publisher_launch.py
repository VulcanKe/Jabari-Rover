from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_publisher_pkg',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        )
    ])
