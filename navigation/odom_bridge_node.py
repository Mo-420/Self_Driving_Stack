#!/usr/bin/env python3
"""odom_bridge_node.py
Bridges ROS 2 odometry and compass topics into the pure-Python PositionTracker
(singleton inside NavigationController).
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from .navigation_controller import NavigationController

class OdomBridgeNode(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._cb_odom, 20)
        self.sub_compass = self.create_subscription(Float32, '/compass_heading', self._cb_heading, 10)
        self.tracker = NavigationController.get_instance()._tracker  # access singleton

    def _cb_odom(self, msg: Odometry):
        self.tracker.set_position(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _cb_heading(self, msg: Float32):
        self.tracker.set_heading(math.radians(msg.data))


def main():
    rclpy.init()
    node = OdomBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 