#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mask_yaml = PathJoinSubstitution([
        FindPackageShare('waveshare_slam'), 'maps', 'sidewalk_mask.yaml'
    ])
    server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='sidewalk_mask_server',
        parameters=[{'yaml_filename': mask_yaml}],
        remappings=[('map', 'sidewalk_mask')]
    )
    return LaunchDescription([server]) 