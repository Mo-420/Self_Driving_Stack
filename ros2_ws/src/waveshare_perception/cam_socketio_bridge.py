#!/usr/bin/env python3
"""Bridge Pi camera frames to Socket.IO for web dashboard."""
import base64
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socketio
import cv2

class CamSocketBridge(Node):
    def __init__(self):
        super().__init__('cam_socket_bridge')
        self.declare_parameter('server_url', 'http://localhost:5000')
        self.declare_parameter('namespace', '/robotaxi')
        url = self.get_parameter('server_url').value
        nsp = self.get_parameter('namespace').value
        self.sio = socketio.Client()
        try:
            self.sio.connect(url, namespaces=[nsp])
            self.get_logger().info(f'Connected to web-server at {url}{nsp}')
        except Exception as e:
            self.get_logger().error(f'Socket.IO connect failed: {e}')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/usb_cam/image_raw', self._cb, 10)

    def _cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        _, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        b64 = base64.b64encode(jpg.tobytes()).decode('ascii')
        try:
            self.sio.emit('camera', b64, namespace=self.get_parameter('namespace').value)
        except Exception:
            pass

def main():
    rclpy.init()
    CamSocketBridge()
    rclpy.spin()
if __name__ == '__main__':
    main() 