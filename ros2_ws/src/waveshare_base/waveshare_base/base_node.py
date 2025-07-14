#!/usr/bin/env python3
"""
waveshare_base/base_node.py
---------------------------
ROS 2 node that connects the Waveshare Robotaxi chassis (serial UART) to the
standard ROS geometry_msgs/Twist interface.  Subscribes to `/cmd_vel` and sends
speed / turning commands to the chassis; publishes a stub `/odom` so other
Nav2 components can start before full odometry is implemented.

Requirements:
- ROS 2 (Humble+) with `rclpy`
- geometry_msgs, nav_msgs
- The `ugv_rpi` codebase must be on the PYTHONPATH (already in this repo).

Launching:
    ros2 run waveshare_base base_node.py
"""

import math
import time
from threading import Thread

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool

try:
    # Re-use existing serial controller
    from ugv_rpi.base_ctrl import BaseController
except ModuleNotFoundError as e:
    raise SystemExit("ugv_rpi package not found in PYTHONPATH. Add the project root to PYTHONPATH before running this node.") from e


DEFAULT_UART = "/dev/serial0"  # Pi 4 UART once Bluetooth disabled
DEFAULT_BAUD = 115200

class WaveshareBaseNode(Node):
    """ROS 2 interface to the Waveshare base controller."""

    def __init__(self):
        super().__init__('waveshare_base')

        # Initialise serial controller
        self.base = BaseController(DEFAULT_UART, DEFAULT_BAUD)
        self.get_logger().info("Connected to Waveshare base on %s" % DEFAULT_UART)

        # ROS 2 Interfaces --------------------------------------------------
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_cb,
            qos)

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)

        self.e_stop = False
        self.create_subscription(Bool, 'e_stop', self._e_stop_cb, 10)

        # Odometry placeholders
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self._last_time = self.get_clock().now()

        # Publish odometry at 10 Hz (placeholder – replace with real data)
        self.create_timer(0.1, self._publish_fake_odom)

    # ------------------------------------------------------------------
    # ROS 2 Callbacks
    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        """Translate Twist into chassis JSON command."""
        linear_x = float(msg.linear.x)      # m/s (positive forward)
        angular_z = float(msg.angular.z)    # rad/s (positive left)

        # Waveshare JSON expects forward speed `X` (-1..1) and yaw rate `Z` (-1..1)
        # We assume max sidewalk speed 0.3 m/s maps to 1.0
        MAX_SPEED = 0.3
        MAX_TURN  = 1.0  # rad/s

        # Alternative: Use direct wheel speed control for differential drive
        # This gives more precise control
        if abs(angular_z) < 0.01:  # Nearly straight motion
            # Use T:1 command for direct wheel speeds in m/s
            left_speed = linear_x
            right_speed = linear_x
            cmd = {"T": 1, "L": left_speed, "R": right_speed}
            self.base.base_json_ctrl(cmd)
            self.get_logger().debug(f"Direct wheel control: L={left_speed}, R={right_speed}")
        else:
            # Use T:13 for combined linear/angular motion
            speed_norm = max(min(linear_x / MAX_SPEED, 1.0), -1.0)
            turn_norm  = max(min(angular_z / MAX_TURN, 1.0), -1.0)
            cmd = {"T": 13, "X": speed_norm, "Z": turn_norm}
            self.base.base_json_ctrl(cmd)
            self.get_logger().debug(f"ROS control: X={speed_norm}, Z={turn_norm}")

    def _e_stop_cb(self, msg: Bool):
        self.e_stop = msg.data
        if self.e_stop:
            self.base.gimbal_emergency_stop()

    # ------------------------------------------------------------------
    def _publish_fake_odom(self):
        """Publish a dummy Odometry message so that Nav2 can start.
        Real odometry should be wired from wheel encoders or SLAM later."""
        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        # No motion integration yet
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.w = 1.0  # identity quaternion

        # Zero velocities (until proper feedback added)
        self.odom_pub.publish(odom)


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------

def main():
    rclpy.init()
    node = WaveshareBaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 