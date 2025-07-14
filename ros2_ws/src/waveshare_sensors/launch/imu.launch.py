#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_node = Node(
        package='waveshare_sensors',
        executable='bno085_node',
        name='bno085_node',
        output='screen',
    )
    return LaunchDescription([imu_node]) 