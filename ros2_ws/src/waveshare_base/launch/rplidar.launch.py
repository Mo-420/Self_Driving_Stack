#!/usr/bin/env python3
"""Launch Slamtec RPLiDAR A1/A2 driver

Expects the `rplidar_ros` package (https://github.com/Slamtec/rplidar_ros).
By default the USB serial device is /dev/ttyUSB0 at 115200 baud.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                                     description='Serial port for RPLIDAR')
    frame_arg = DeclareLaunchArgument('frame_id', default_value='laser',
                                      description='Frame id for published scans')
    scan_mode_arg = DeclareLaunchArgument('scan_mode', default_value='Standard',
                                          description='RPLIDAR scan mode')

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 115200,
            'frame_id': LaunchConfiguration('frame_id'),
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': LaunchConfiguration('scan_mode'),
        }],
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        frame_arg,
        scan_mode_arg,
        lidar_node,
    ]) 