#!/usr/bin/env python3
"""Publish robot description via robot_state_publisher"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('waveshare_description'), 'urdf', 'robot.urdf.xacro'
    ])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}]
    )

    return LaunchDescription([rsp]) 