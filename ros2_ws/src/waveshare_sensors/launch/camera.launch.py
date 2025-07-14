#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dev_arg = DeclareLaunchArgument('device', default_value='/dev/video0')
    width_arg = DeclareLaunchArgument('width', default_value='640')
    height_arg = DeclareLaunchArgument('height', default_value='480')
    fps_arg = DeclareLaunchArgument('fps', default_value='30')

    cam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='pi_ai_camera',
        parameters=[{
            'video_device': LaunchConfiguration('device'),
            'image_size': [LaunchConfiguration('width'), LaunchConfiguration('height')],
            'frame_rate': LaunchConfiguration('fps'),
            'output_encoding': 'bgr8'
        }],
        remappings=[('image_raw', '/usb_cam/image_raw')]
    )

    return LaunchDescription([dev_arg, width_arg, height_arg, fps_arg, cam_node]) 