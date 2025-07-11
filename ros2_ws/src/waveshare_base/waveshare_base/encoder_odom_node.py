#!/usr/bin/env python3
"""
encoder_odom_node.py
--------------------
Reads wheel encoder data from the ESP32 feedback (T:1003 messages) and
publishes accurate odometry based on actual wheel rotations.

The ESP32 sends feedback like:
{"T":1003,"L":left_speed,"R":right_speed,"r":roll,"p":pitch,"v":voltage}

This node integrates the wheel speeds to compute pose.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

try:
    from ugv_rpi.base_ctrl import BaseController
except ImportError:
    # For development/testing without ugv_rpi
    class BaseController:
        def __init__(self, *args):
            pass
        def feedback_data(self):
            return {"T": 1003, "L": 0.0, "R": 0.0, "r": 0.0, "p": 0.0, "v": 12.0}

class EncoderOdomNode(Node):
    """Publishes odometry from wheel encoder feedback"""
    
    def __init__(self):
        super().__init__('encoder_odom_node')
        
        # Parameters
        self.declare_parameter('wheel_base', 0.3)  # meters between wheels
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('uart_device', '/dev/serial0')
        self.declare_parameter('baud_rate', 115200)
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        uart_dev = self.get_parameter('uart_device').value
        baud = self.get_parameter('baud_rate').value
        
        # Initialize hardware interface
        try:
            self.base = BaseController(uart_dev, baud)
            self.get_logger().info(f"Connected to ESP32 on {uart_dev}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ESP32: {e}")
            self.base = BaseController("dummy", 0)  # Fallback for testing
        
        # Publishers
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_left_dist = 0.0
        self.last_right_dist = 0.0
        
        # Start feedback thread
        self.feedback_thread = threading.Thread(target=self._feedback_loop, daemon=True)
        self.feedback_thread.start()
        
        # Timer for publishing at fixed rate
        self.create_timer(0.05, self._publish_odometry)  # 20 Hz
        
    def _feedback_loop(self):
        """Continuously read encoder feedback from ESP32"""
        while rclpy.ok():
            try:
                data = self.base.feedback_data()
                if data and data.get('T') == 1003:
                    # Extract wheel speeds (m/s)
                    self.left_speed = float(data.get('L', 0.0))
                    self.right_speed = float(data.get('R', 0.0))
                    self.battery_voltage = float(data.get('v', 12.0))
            except Exception as e:
                self.get_logger().warn(f"Feedback error: {e}", throttle_duration_sec=5.0)
                self.left_speed = 0.0
                self.right_speed = 0.0
    
    def _publish_odometry(self):
        """Compute and publish odometry based on wheel speeds"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # Differential drive kinematics
        linear_vel = (self.left_speed + self.right_speed) / 2.0
        angular_vel = (self.right_speed - self.left_speed) / self.wheel_base
        
        # Update pose
        if abs(angular_vel) < 1e-6:
            # Straight line motion
            delta_x = linear_vel * math.cos(self.theta) * dt
            delta_y = linear_vel * math.sin(self.theta) * dt
        else:
            # Arc motion
            radius = linear_vel / angular_vel
            delta_theta = angular_vel * dt
            delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = radius * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
        
        self.x += delta_x
        self.y += delta_y
        self.theta += angular_vel * dt
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create quaternion from yaw
        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        
        # Velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        # Covariance (simplified)
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.1  # theta
        
        self.odom_pub.publish(odom)
        
        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 