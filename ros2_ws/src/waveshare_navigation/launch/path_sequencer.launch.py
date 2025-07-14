#!/usr/bin/env python3
"""Launch Path Sequencer node

This launch file starts the `path_sequencer_node` which converts the
`/route_waypoints` Path into sequential `/goal_pose` messages or Nav2
FollowWaypoints (future).
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sequencer_node = Node(
        package='waveshare_navigation',
        executable='path_sequencer_node',
        name='path_sequencer_node',
        output='screen',
    )
    return LaunchDescription([sequencer_node]) 