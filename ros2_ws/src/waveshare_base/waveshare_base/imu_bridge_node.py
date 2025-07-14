#!/usr/bin/env python3
"""imu_bridge_node.py
Reads ESP32 IMU data (roll, pitch) from BaseController feedback (T:1003) and
publishes `sensor_msgs/Imu` so robot_localization EKF can fuse it.
Orientation yaw is left zero (no magnetometer). Covariances are coarse.
"""
import math
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

try:
    from ugv_rpi.base_ctrl import BaseController
except ImportError:
    BaseController = None

DEFAULT_UART = '/dev/serial0'
DEFAULT_BAUD = 115200

class ImuBridgeNode(Node):
    def __init__(self):
        super().__init__('imu_bridge_node')
        self.declare_parameter('uart_device', DEFAULT_UART)
        self.declare_parameter('baud_rate', DEFAULT_BAUD)
        uart = self.get_parameter('uart_device').value
        baud = self.get_parameter('baud_rate').value

        if BaseController:
            self.base = BaseController(uart, baud)
        else:
            self.base = None
            self.get_logger().warning('ugv_rpi not found – IMU bridge running in dummy mode')

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Imu, 'imu/data', qos)

        # Start background thread
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        rate_hz = 50
        while rclpy.ok():
            if self.base:
                data = self.base.feedback_data()
                if data and data.get('T') == 1003:
                    roll = float(data.get('r', 0.0))  # degrees
                    pitch = float(data.get('p', 0.0))
                    # Convert to radians
                    roll_rad = math.radians(roll)
                    pitch_rad = math.radians(pitch)
                    # Construct quaternion (yaw = 0)
                    cy = math.cos(0.0 * 0.5)
                    sy = math.sin(0.0 * 0.5)
                    cp = math.cos(pitch_rad * 0.5)
                    sp = math.sin(pitch_rad * 0.5)
                    cr = math.cos(roll_rad * 0.5)
                    sr = math.sin(roll_rad * 0.5)
                    q_w = cr * cp * cy + sr * sp * sy
                    q_x = sr * cp * cy - cr * sp * sy
                    q_y = cr * sp * cy + sr * cp * sy
                    q_z = cr * cp * sy - sr * sp * cy

                    msg = Imu()
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.orientation.w = q_w
                    msg.orientation.x = q_x
                    msg.orientation.y = q_y
                    msg.orientation.z = q_z
                    msg.orientation_covariance = [0.05,0,0,0,0.05,0,0,0,0.1]
                    self.pub.publish(msg)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0/rate_hz))

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 