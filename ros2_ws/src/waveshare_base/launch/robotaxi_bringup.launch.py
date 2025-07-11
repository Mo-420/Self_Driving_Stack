#!/usr/bin/env python3
"""
robotaxi_bringup.launch.py
--------------------------
Main launch file for the WaveShare Robotaxi. Brings up:
- Motor control (base_node)
- Odometry (encoder_odom_node)
- Perception (YOLO sign detection, lane segmentation)
- Safety (ultrasonic emergency stop)
- Navigation (waypoint follower, route receiver)
- Web integration (route receiver from Socket.IO)

Usage:
    ros2 launch waveshare_base robotaxi_bringup.launch.py
    
Parameters:
    use_sim_time: bool (default: false)
    web_server_url: str (default: http://localhost:5000)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    web_server_url = LaunchConfiguration('web_server_url', default='http://localhost:5000')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_web_server = DeclareLaunchArgument(
        'web_server_url',
        default_value='http://localhost:5000',
        description='URL of the robotaxi web server'
    )
    
    # Base motor control node
    base_node = Node(
        package='waveshare_base',
        executable='base_node',
        name='base_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Encoder odometry node (replaces fake odom)
    encoder_odom_node = Node(
        package='waveshare_base',
        executable='encoder_odom_node',
        name='encoder_odom_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_base': 0.3,  # Adjust for your robot
            'publish_tf': True
        }]
    )
    
    # Perception nodes
    yolo_sign_node = Node(
        package='waveshare_perception',
        executable='yolo_sign_node',
        name='yolo_sign_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': os.path.expanduser('~/WaveShare/models/yolov8n-signs.pt'),
            'confidence_threshold': 0.5
        }]
    )
    
    lane_segmentation_node = Node(
        package='waveshare_perception',
        executable='lane_segmentation_node',
        name='lane_segmentation_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Safety node
    ultrasonic_safety_node = Node(
        package='waveshare_safety',
        executable='ultrasonic_safety_node',
        name='ultrasonic_safety_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'stop_distance': 0.3,  # meters
            'slow_distance': 1.0   # meters
        }]
    )
    
    # Navigation nodes
    goal_follower_node = Node(
        package='waveshare_navigation',
        executable='goal_follower_node',
        name='goal_follower_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_vel': 0.3,   # m/s
            'max_angular_vel': 1.0,  # rad/s
            'goal_tolerance': 0.5    # meters
        }]
    )
    
    rules_of_road_node = Node(
        package='waveshare_navigation',
        executable='rules_of_road_node',
        name='rules_of_road_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Web integration
    route_receiver_node = Node(
        package='waveshare_navigation',
        executable='route_receiver_node',
        name='route_receiver_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'server_url': web_server_url,
            'namespace': '/robotaxi'
        }]
    )
    
    # Static transform publishers (robot frames)
    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    base_to_ultrasonic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_ultrasonic',
        arguments=['0.15', '0', '0.05', '0', '0', '0', 'base_link', 'ultrasonic_link']
    )
    
    # Nav2 bringup (optional - uncomment if you want full Nav2 stack)
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('nav2_bringup'),
    #             'launch',
    #             'navigation_launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'params_file': PathJoinSubstitution([
    #             FindPackageShare('waveshare_navigation'),
    #             'config',
    #             'nav2_params.yaml'
    #         ])
    #     }.items()
    # )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_web_server,
        
        # Core nodes
        base_node,
        encoder_odom_node,
        
        # Perception
        yolo_sign_node,
        lane_segmentation_node,
        
        # Safety
        ultrasonic_safety_node,
        
        # Navigation
        goal_follower_node,
        rules_of_road_node,
        route_receiver_node,
        
        # TF
        base_to_camera,
        base_to_ultrasonic,
        
        # Optional Nav2
        # nav2_launch
    ]) 