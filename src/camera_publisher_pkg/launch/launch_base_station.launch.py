from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control_pkg',
            executable='keyboard_control_node',
            name='keyboard_control',
            output='screen'
        ),
        Node(
            package='yolo_image_processor',
            executable='yolo_image_processor',
            name='yolo_image_processor',
            output='screen'
        ),
        Node(
            package='camera_publisher_pkg',
            executable='web_streamer',
            name='web_streamer',
            output='screen'
        ),
    ])
