#!/usr/bin/env python3
"""
waveshare_perception/yolo_traffic_light_node.py
------------------------------------------------
Detects traffic lights and their colour state (red / green / yellow) using a
YOLO-v8 nano network.  Publishes a simple custom message on /traffic_light.
For demo purposes we just output std_msgs/String with value "red", "green",
"yellow" or "none".
"""

import os
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        self.declare_parameter('weights', os.path.expanduser('~/WaveShare/models/yolov8n-traffic.pt'))
        self.declare_parameter('conf', 0.25)
        weights = self.get_parameter('weights').value
        conf_thres = float(self.get_parameter('conf').value)

        if YOLO is None:
            self.get_logger().error('ultralytics not installed; run `pip install ultralytics`')
            raise SystemExit(1)
        self.model = YOLO(weights)
        self.model.fuse()
        self.conf = conf_thres
        self.bridge = CvBridge()

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                         depth=5,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, '/usb_cam/image_raw', self.cb_img, qos)
        self.pub = self.create_publisher(String, '/traffic_light', 5)

    def cb_img(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        res = self.model.predict(source=img, imgsz=320, conf=self.conf, verbose=False)[0]

        state = 'none'
        for box, cls, conf in zip(res.boxes.xyxy, res.boxes.cls, res.boxes.conf):
            label = int(cls)
            if label != 0:  # assuming single class model (traffic light)
                continue
            x1, y1, x2, y2 = box.cpu().numpy().astype(int)
            sub = img[y1:y2, x1:x2]
            if sub.size == 0:
                continue
            hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)

            # naive HSV thresholds for red/green/yellow lights
            mask_red1 = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255))
            mask_red2 = cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            mask_green = cv2.inRange(hsv, (40, 70, 50), (90, 255, 255))
            mask_yellow = cv2.inRange(hsv, (15, 70, 50), (35, 255, 255))

            red_pix = np.sum(mask_red > 0)
            green_pix = np.sum(mask_green > 0)
            yellow_pix = np.sum(mask_yellow > 0)

            if red_pix > green_pix and red_pix > yellow_pix:
                state = 'red'
            elif green_pix > red_pix and green_pix > yellow_pix:
                state = 'green'
            elif yellow_pix > 0:
                state = 'yellow'
            else:
                state = 'none'
            break

        self.pub.publish(String(data=state))


def main():
    rclpy.init()
    node = TrafficLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 