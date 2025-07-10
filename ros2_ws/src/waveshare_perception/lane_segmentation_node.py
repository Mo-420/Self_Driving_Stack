#!/usr/bin/env python3
"""
waveshare_perception/lane_segmentation_node.py
---------------------------------------------
Very lightweight lane-segmentation demo node.
Takes /usb_cam/image_raw (BGR) -> outputs /lane_mask (mono8)
so downstream nodes (e.g., behaviour or Nav2 costmap) can subscribe.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneSegmentationNode(Node):
    def __init__(self):
        super().__init__('lane_segmentation')
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, '/usb_cam/image_raw', self._cb, qos)
        self.pub = self.create_publisher(Image, '/lane_mask', qos)
        self.bridge = CvBridge()
        # Simple HSV thresholds for yellow/white lines (tweak in real env)
        self.yellow_low = np.array([20,  100, 100])
        self.yellow_high= np.array([40,  255, 255])
        self.white_low  = np.array([0,   0,   200])
        self.white_high = np.array([180, 50,  255])

    def _cb(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask_y = cv2.inRange(hsv, self.yellow_low, self.yellow_high)
        mask_w = cv2.inRange(hsv, self.white_low,  self.white_high)
        mask = cv2.bitwise_or(mask_y, mask_w)
        mask = cv2.medianBlur(mask, 5)

        out_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    node = LaneSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 