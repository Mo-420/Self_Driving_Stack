#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import socketio

class WebEStopNode(Node):
    def __init__(self):
        super().__init__('web_e_stop_node')
        self.declare_parameter('server_url', 'http://localhost:5000')
        self.declare_parameter('namespace', '/')
        self.server_url = self.get_parameter('server_url').value
        self.namespace = self.get_parameter('namespace').value
        self.pub = self.create_publisher(Bool, 'e_stop', 10)
        self.sio = socketio.Client()
        self.sio.on('e_stop', self._on_e_stop, namespace=self.namespace)
        self.sio.connect(self.server_url, namespaces=[self.namespace])

    def _on_e_stop(self, data):
        msg = Bool()
        msg.data = bool(data.get('stop', True))
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WebEStopNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main() 