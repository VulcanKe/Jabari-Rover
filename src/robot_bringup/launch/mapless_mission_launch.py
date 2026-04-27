#!/usr/bin/env python3
"""
Mapless Mission Controller Launch
Runs only the custom mission node for balloon collection
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Directories
    bringup_dir = get_package_share_directory('robot_bringup')
    
    # Optional RViz config path
    rviz_config = os.path.join(os.path.expanduser('~/robot_rviz'), 'robot_basic.rviz')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2 for visualization'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Core nodes list
    nodes = [
        # === Mapless Mission Controller ===
        Node(
            package='robot_bringup',
            executable='mapless_mission_controller',
            name='mapless_mission_controller',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Mission parameters
                'v_max': 0.2,
                'w_max': 0.4,
                'safe_distance': 0.5,
                'retreat_speed': -0.2,
                'stop_duration': 5.0,
                'yaw_gain': 1.5,
                'obstacle_threshold': 0.25,
                'use_lidar': True,
                'scan_speed': 0.15,
                'approach_distance': 1.0,
                'startup_yaw_duration': 4.0
            }],
            remappings=[
                # Add any topic remappings here if needed
                # ('/cmd_vel', '/robot/cmd_vel'),
                # ('/scan', '/robot/scan'),
                # ('/object_info', '/robot/object_info'),
                # ('/imu/data', '/robot/imu/data')
            ]
        )
    ]
    
    # Conditional RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        use_sim_time_arg,
        
        # Core mission node
        *nodes,
        
        # Optional RViz
        rviz_node
    ])
