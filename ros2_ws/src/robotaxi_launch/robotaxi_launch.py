from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Ultrasonic safety filter
        Node(
            package='waveshare_safety',
            executable='ultrasonic_safety_node.py',
            name='ultrasonic_safety',
            output='screen'
        ),

        # Base controller (listens to /cmd_vel_safe)
        Node(
            package='waveshare_base',
            executable='base_node.py',
            name='waveshare_base',
            output='screen',
            remappings=[
                ('/cmd_vel', '/cmd_vel_safe')
            ]
        ),

        # USB camera driver
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_cam',
            output='screen',
            parameters=[{'video_device': '/dev/video0',
                         'image_size': [640, 480],
                         'frame_rate': 30}]
        ),

        # Wheel odometry publisher
        Node(
            package='waveshare_base',
            executable='odom_publisher_node',
            name='wheel_odom',
            output='screen'
        ),

        # YOLO sign detector
        Node(
            package='waveshare_perception',
            executable='yolo_sign_node',
            name='yolo_sign',
            output='screen'
        ),

        # YOLO traffic-light detector
        Node(
            package='waveshare_perception',
            executable='yolo_traffic_light_node',
            name='yolo_traffic',
            output='screen'
        ),

        # Lane segmentation
        Node(
            package='waveshare_perception',
            executable='lane_segmentation_node',
            name='lane_segmentation',
            output='screen'
        ),
    ]) 