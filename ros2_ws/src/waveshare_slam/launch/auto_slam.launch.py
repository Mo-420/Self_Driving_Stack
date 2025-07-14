#!/usr/bin/env python3
"""auto_slam.launch.py

Launches localisation (slam_online) if a map YAML file exists, otherwise
launches mapping (slam_mapping) to create the first map.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


MAP_FILE = LaunchConfiguration('map_file')


def _choose_launch(context, *args, **kwargs):
    pkg_share = context.perform_substitution(FindPackageShare('waveshare_slam'))
    map_path = os.path.join(pkg_share, 'maps', context.perform_substitution(MAP_FILE))
    if os.path.exists(map_path):
        chosen = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('waveshare_slam'), 'launch', 'slam_online.launch.py'])
            ]),
            launch_arguments={'map_file': map_path}.items()
        )
    else:
        chosen = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('waveshare_slam'), 'launch', 'slam_mapping.launch.py'])
            ])
        )
    return [chosen]


def generate_launch_description():
    map_arg = DeclareLaunchArgument('map_file', default_value='map.yaml',
                                    description='Map YAML file in waveshare_slam/maps')
    chooser = OpaqueFunction(function=_choose_launch)
    return LaunchDescription([map_arg, chooser]) 