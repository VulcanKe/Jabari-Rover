from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manual_control',
            executable='manual_control_node',
            name='manual_control_node',
            output='screen',
            parameters=[{
                'button_toggle_pivot': 7,      # Button for toggling pivot mode
                'button_lin_up': 0,            # Increase linear speed
                'button_lin_down': 2,          # Decrease linear speed
                'button_ang_up': 3,            # Increase angular speed
                'button_ang_down': 1,          # Decrease angular speed
                'pivot_cmd_value': 1.0,        # Angular.z value in pivot mode
                'speed_step': 0.1              # Speed increment per button press
            }]
        )
    ])
# This launch file starts the manual control node and the joystick node.
