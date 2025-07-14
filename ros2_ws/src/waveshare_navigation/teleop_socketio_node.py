#!/usr/bin/env python3
"""Receive teleop commands from web app via Socket.IO and publish /cmd_vel_remote"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socketio

class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')
        self.declare_parameter('server_url', 'http://localhost:5000')
        self.declare_parameter('namespace', '/robotaxi')
        url = self.get_parameter('server_url').value
        nsp = self.get_parameter('namespace').value
        self.pub = self.create_publisher(Twist, '/cmd_vel_remote', 10)
        self.sio = socketio.Client()
        self.sio.on('teleop_ros', self._on_cmd, namespace=nsp)
        try:
            self.sio.connect(url, namespaces=[nsp])
            self.get_logger().info('Teleop bridge connected')
        except Exception as e:
            self.get_logger().error(f'Socket IO connect error: {e}')

    def _on_cmd(self, data):
        msg = Twist()
        msg.linear.x = float(data.get('linear', 0))
        msg.angular.z = float(data.get('angular', 0))
        self.pub.publish(msg)


def main():
    rclpy.init()
    TeleopBridge()
    rclpy.spin()

if __name__ == '__main__':
    main() 