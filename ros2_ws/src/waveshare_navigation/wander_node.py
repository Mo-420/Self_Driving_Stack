#!/usr/bin/env python3
"""
waveshare_navigation/wander_node.py
-----------------------------------
Simple reactive wander behaviour:
• Constantly drives forward at a set speed.
• When the ultrasonic safety layer reports an obstacle in front (< 0.6 m),
  it chooses the side (left/right) with more space and turns ~45° using the IMU
  yaw estimate.  Then it resumes forward motion.

Inputs
------
/safety_status   – std_msgs/String  (JSON produced by ultrasonic_safety_node)
/imu/data        – sensor_msgs/Imu (optional but recommended)

Output
------
/cmd_vel         – geometry_msgs/Twist (raw command – will be filtered again
                   by the ultrasonic safety node before reaching the base)

The node is deliberately stateless and lightweight; it does not build a map.
It simply wanders while avoiding collisions, useful when lane markers are
missing.
"""

import json
import math
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu

# ------------------------------------------------------------
class State(Enum):
    FORWARD = 1
    TURNING = 2


class WanderNode(Node):
    def __init__(self):
        super().__init__('wander')

        # Parameters --------------------------------------------------
        self.declare_parameter('forward_speed', 0.20)      # m/s
        self.declare_parameter('turn_speed',    0.6)       # rad/s
        self.declare_parameter('front_clear_d', 0.60)      # m
        self.declare_parameter('turn_angle_deg', 45.0)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed    = float(self.get_parameter('turn_speed').value)
        self.front_clear_d = float(self.get_parameter('front_clear_d').value)
        self.turn_angle    = math.radians(float(self.get_parameter('turn_angle_deg').value))

        # State -------------------------------------------------------
        self.state = State.FORWARD
        self.turn_dir = 0             # +1 left, -1 right
        self.yaw_start: Optional[float] = None
        self.current_yaw: float = 0.0

        # Publishers / Subscribers -----------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/safety_status', self._cb_safety, 10)
        self.create_subscription(Imu, '/imu/data', self._cb_imu, 10)

        # Main loop 20 Hz
        self.create_timer(0.05, self._control_loop)

        self.front_dist = 999.0
        self.left_dist  = 999.0
        self.right_dist = 999.0

        self.get_logger().info('Wander node ready – roaming behaviour active.')

    # ----------------------------------------------------------
    def _cb_safety(self, msg: String):
        try:
            data = json.loads(msg.data)
            d = data.get('distances', {})
            self.front_dist = float(d.get('front', 999.0))
            self.left_dist  = float(d.get('left', 999.0))
            self.right_dist = float(d.get('right', 999.0))
        except Exception as e:
            self.get_logger().warn(f'Failed to parse safety_status JSON: {e}')

    # ----------------------------------------------------------
    def _cb_imu(self, msg: Imu):
        # Extract yaw from quaternion
        q = msg.orientation
        # yaw calculation (z-rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ----------------------------------------------------------
    def _control_loop(self):
        cmd = Twist()

        if self.state == State.FORWARD:
            # If obstacle ahead, switch to TURNING
            if self.front_dist < self.front_clear_d:
                self.state = State.TURNING
                # decide direction
                self.turn_dir = 1 if self.left_dist > self.right_dist else -1
                self.yaw_start = self.current_yaw
                self.get_logger().info(f'Obstacle ahead ({self.front_dist:.2f} m) – turning {"left" if self.turn_dir==1 else "right"}')
            else:
                # Just go forward
                cmd.linear.x = self.forward_speed

        if self.state == State.TURNING:
            cmd.angular.z = self.turn_dir * self.turn_speed
            # Check if we have turned enough using IMU
            if self.yaw_start is not None:
                delta = self._angle_diff(self.current_yaw, self.yaw_start)
                if abs(delta) >= self.turn_angle and self.front_dist >= self.front_clear_d:
                    self.state = State.FORWARD
                    self.get_logger().info('Turn complete – resuming forward motion')

        self.cmd_pub.publish(cmd)

    # ----------------------------------------------------------
    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Smallest signed difference between two angles (rad)."""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d


def main():
    rclpy.init()
    node = WanderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 