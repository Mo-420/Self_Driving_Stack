#!/usr/bin/env python3
"""
waveshare_navigation/goal_follower_node.py
-----------------------------------------
Drives the robot to a user-specified goal pose (2D) without relying on lane
markings.  Works with the existing safety layer; if the ultrasonic guardian
forces a detour, the node keeps re-evaluating the remaining vector to the goal
and corrects its course.

How to use (RViz example):
1.  Start launch file – this node will advertise /goal_pose.
2.  In RViz, add the “2D Nav Goal” tool and click a spot; RViz publishes the
    clicked pose on /goal_pose (geometry_msgs/PoseStamped).
3.  The robot rotates to face the goal, drives forward, and stops when it is
    within 20 cm.

Inputs
------
/goal_pose        geometry_msgs/PoseStamped (latched) – target.
/odom             nav_msgs/Odometry         – current pose.
/imu/data         sensor_msgs/Imu (optional) – improves heading estimation.

Outputs
-------
/cmd_vel          geometry_msgs/Twist – raw command (filtered by safety node).
/status           std_msgs/String – human-readable state.

Limitations
-----------
This is a dead-reckoning controller; without an external map or SLAM it drifts
slowly.  Good for short indoor trips (<5 m) or until a full Nav2 setup is
added.
"""

import math
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

# ------------------------------------------------------------
class GoalFollower(Node):
    def __init__(self):
        super().__init__('goal_follower')

        # Parameters --------------------------------------------------
        self.declare_parameter('linear_speed', 0.25)        # m/s
        self.declare_parameter('angular_speed', 0.6)        # rad/s
        self.declare_parameter('goal_tolerance', 0.20)      # m
        self.declare_parameter('angle_tolerance', 0.05)     # rad

        self.lin_speed = float(self.get_parameter('linear_speed').value)
        self.ang_speed = float(self.get_parameter('angular_speed').value)
        self.goal_tol  = float(self.get_parameter('goal_tolerance').value)
        self.ang_tol   = float(self.get_parameter('angle_tolerance').value)

        # Internal state ---------------------------------------------
        self.goal: Optional[PoseStamped] = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw    = 0.0
        self.active = False

        # Publishers / Subscribers -----------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/goal_follower/status', 10)

        self.create_subscription(PoseStamped, '/goal_pose', self._cb_goal, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(Imu, '/imu/data', self._cb_imu, 10)

        # Main loop 20 Hz
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Goal follower ready – waiting for /goal_pose')

    # ----------------------------------------------------------
    def _cb_goal(self, msg: PoseStamped):
        self.goal = msg
        self.active = True
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def _cb_odom(self, msg: Odometry):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        # Derive yaw from quaternion if no IMU yet
        q = msg.pose.pose.orientation
        self.yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)

    def _cb_imu(self, msg: Imu):
        q = msg.orientation
        self.yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)

    # ----------------------------------------------------------
    def _control_loop(self):
        cmd = Twist()
        status_txt = ''

        if self.active and self.goal is not None:
            gx = self.goal.pose.position.x
            gy = self.goal.pose.position.y

            # Vector to goal
            dx = gx - self.pose_x
            dy = gy - self.pose_y
            dist = math.hypot(dx, dy)
            target_heading = math.atan2(dy, dx)
            heading_error = self._angle_diff(target_heading, self.yaw)

            if dist <= self.goal_tol:
                self.active = False
                status_txt = 'GOAL_REACHED'
            else:
                # First rotate until heading error small
                if abs(heading_error) > self.ang_tol:
                    cmd.angular.z = self.ang_speed if heading_error > 0 else -self.ang_speed
                    status_txt = 'ROTATING'
                else:
                    cmd.linear.x = self.lin_speed
                    status_txt = 'DRIVING'
        else:
            status_txt = 'IDLE'

        self.cmd_pub.publish(cmd)
        self.status_pub.publish(String(data=status_txt))

    # ----------------------------------------------------------
    @staticmethod
    def _quat_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d


def main():
    rclpy.init()
    node = GoalFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 