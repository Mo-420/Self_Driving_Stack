#!/usr/bin/env python3
"""Launch GPS serial driver (u-blox Air530) and navsat_transform_node"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB1')
    baud_arg = DeclareLaunchArgument('baud', default_value='9600')

    gps_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'frame_id': 'gps'
        }],
        output='screen',
    )

    navsat_tf = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[{
            'imu0': '/imu/data',
            'odom0': '/odom',
            'yaw_offset': 0.0,
            'zero_altitude': True,
            'publish_tf': False,
            'use_odometry_yaw': True
        }],
        remappings=[('/gps/fix', '/fix'),
                    ('/imu/data', '/imu/data'),
                    ('/odometry/gps', '/odometry/gps')],
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        gps_driver,
        navsat_tf,
    ]) 