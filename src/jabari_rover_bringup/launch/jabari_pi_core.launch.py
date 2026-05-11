# jabari_pi_core.launch.py
#
# Launch Pi core stack:
#   - Servo node
#   - Motor controller node
#   - Dual motor driver node
#

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jabari_pan_tilt_control_pkg',
            executable='servo_node',
            name='servo_node',
            output='screen',
        ),
	#  Node(
    #         package='jabari_pan_tilt_control_pkg',
    #         executable='mock_rpi_global',
    #         name='servo_node',
    #         output='screen',
    #     ),
    #      Node(
    #         package='jabari_pan_tilt_control_pkg',
    #         executable='mock_gpio',
    #         name='servo_node',
    #         output='screen',
    #     ),
        Node(
            package='motor_control_pkg',
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen',
        ),
        Node(
            package='motor_control_pkg',
            executable='dual_motor_driver_node',
            name='dual_motor_driver_node',
            output='screen',
        ),
	Node(
            package='yolo_image_processor',
            executable='yolo_image_processor',
            name='yolo_image_processor',
            output='screen'
        ),
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
        ),
	Node(
            package='camera_publisher_pkg',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
	Node(
            package='camera_publisher_pkg',
            executable='web_streamer',
            name='web_streamer',
            output='screen'
        ),
	



    ])
