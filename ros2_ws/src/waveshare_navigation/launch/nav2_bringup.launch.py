#!/usr/bin/env python3
"""Launch Nav2 stack with preconfigured params."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = DeclareLaunchArgument('use_sim_time', default_value='false')
    map_yaml = DeclareLaunchArgument('map', default_value='map.yaml')

    nav2_params = PathJoinSubstitution([
        FindPackageShare('waveshare_navigation'), 'config', 'nav2_params.yaml'
    ])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params,
            'map': LaunchConfiguration('map'),
        }.items()
    )

    return LaunchDescription([
        use_sim,
        map_yaml,
        nav2_launch,
    ]) 