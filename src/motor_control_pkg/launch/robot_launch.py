from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control_pkg',
            executable='motor_controller_node',
            name='motor_controller'
        ),
        Node(
            package='motor_control_pkg',
            executable='dual_motor_driver_node',
            name='dual_motor_driver'
        )
    ])
