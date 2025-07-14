#!/usr/bin/env python3
"""bno085_node.py – minimal BNO085 IMU publisher

Uses Adafruit`s CircuitPython bno08x driver.  If the library or hardware is
missing the node falls back to publishing zero orientation so the rest of the
stack can still launch on a desktop.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

# Try to import real driver; fallback to mock
try:
    import board  # type: ignore
    import busio  # type: ignore
    import adafruit_bno08x  # type: ignore
    from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False


def quat_to_msg(quat):
    msg = Imu()
    msg.orientation.w = quat[0]
    msg.orientation.x = quat[1]
    msg.orientation.y = quat[2]
    msg.orientation.z = quat[3]
    return msg


class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085')

        self.declare_parameter('update_rate_hz', 50.0)
        rate = float(self.get_parameter('update_rate_hz').value)
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.temp_pub = self.create_publisher(Float32, '/imu/temp', 10)

        if HW_AVAILABLE:
            i2c = busio.I2C(board.SCL, board.SDA)  # type: ignore
            self.bno = adafruit_bno08x.BNO08X_I2C(i2c)  # type: ignore
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.get_logger().info('BNO085 IMU initialised')
        else:
            self.bno = None
            self.get_logger().warning('BNO085 hardware not detected – publishing zero IMU')

        self.create_timer(1.0 / rate, self._publish)

    # ------------------------------------------------------
    def _publish(self):
        msg = Imu()
        if self.bno:
            quat = self.bno.quaternion  # w, x, y, z
            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]
            # Gyro and accel left zero for now
            self.temp_pub.publish(Float32(data=float(self.bno.temperature)))  # type: ignore
        else:
            msg.orientation.w = 1.0
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'base_link'
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BNO085Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 