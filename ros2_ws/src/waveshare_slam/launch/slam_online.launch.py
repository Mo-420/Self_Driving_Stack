#!/usr/bin/env python3
"""Launch SLAM-Toolbox in localization mode using existing map if available."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_arg = DeclareLaunchArgument('map_file', default_value='map.yaml',
                                    description='YAML map file to load')

    map_path = PathJoinSubstitution([
        FindPackageShare('waveshare_slam'), 'maps', LaunchConfiguration('map_file')
    ])

    slam_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'map_file_name': map_path}],
    )

    return LaunchDescription([
        map_arg,
        slam_node,
    ]) 