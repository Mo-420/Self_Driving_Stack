#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    telemetry_node = Node(
        package='waveshare_navigation',
        executable='telemetry_sender_node',
        parameters=[{'server_url': 'http://localhost:5000'}]
    )
    return LaunchDescription([telemetry_node]) 