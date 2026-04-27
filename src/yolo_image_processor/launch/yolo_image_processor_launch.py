from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_image_processor',
            executable='yolo_image_processor',
            name='yolo_image_processor',
            output='screen'
        )
    ])
