#!/usr/bin/env python3
"""
waveshare_base/odom_publisher_node.py
------------------------------------
Publishes real odometry based on wheel speeds received from the Waveshare base.
The BaseController feedback JSON contains:
    {
        "T": 1001,
        "L": <left wheel speed>,   # -1..1
        "R": <right wheel speed>,  # -1..1
        "v": <battery voltage>,
        ...
    }
Assumptions
-----------
• Wheel radius 0.03 m (6 cm diameter) – adjust WHEEL_RADIUS.
• Track width 0.16 m (distance between wheels) – adjust WHEEL_BASE.
• Speeds are already scaled -1..1 → max 0.3 m/s (same scale used in base_node).

Publishes:
    nav_msgs/Odometry on /odom
"""

import math
from threading import Lock

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time
from tf_transformations import quaternion_from_euler

from ugv_rpi.base_ctrl import BaseController

DEFAULT_UART = "/dev/serial0"
DEFAULT_BAUD = 115200
MAX_WHEEL_SPEED = 0.3   # m/s when input JSON speed = 1.0
WHEEL_RADIUS = 0.03     # metres
WHEEL_BASE   = 0.16     # metres (axle length)

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('waveshare_wheel_odom')

        self.base = BaseController(DEFAULT_UART, DEFAULT_BAUD)
        self.get_logger().info('Wheel-odom node connected to chassis')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.05, self.update)  # 20 Hz

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.lock = Lock()

    # ------------------------------------------------------------------
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Poll one feedback line (non-blocking)
        feedback = self.base.feedback_data()
        if not feedback or 'T' not in feedback or feedback['T'] != 1001:
            return  # skip if no fresh data

        left_norm  = float(feedback.get('L', 0.0))
        right_norm = float(feedback.get('R', 0.0))
        v_l = left_norm  * MAX_WHEEL_SPEED
        v_r = right_norm * MAX_WHEEL_SPEED

        # Unicycle model
        v  = (v_r + v_l) / 2.0
        w  = (v_r - v_l) / WHEEL_BASE

        # Integrate pose
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th += w * dt

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        q = quaternion_from_euler(0.0, 0.0, self.th)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom_msg.twist.twist.linear.x  = v
        odom_msg.twist.twist.angular.z = w

        self.odom_pub.publish(odom_msg)


def main():
    rclpy.init()
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 