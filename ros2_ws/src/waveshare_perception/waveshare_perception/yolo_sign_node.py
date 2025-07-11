#!/usr/bin/env python3
"""
waveshare_perception/yolo_sign_node.py
-------------------------------------
Simple ROS 2 node that runs an Ultralytics YOLO-v8 nano model to detect traffic
signs (stop sign, pedestrian crossing, etc.) in camera frames and publishes a
`vision_msgs/Detection2DArray` topic named `/sign_detections`.

Notes
-----
• This node expects an image topic `/usb_cam/image_raw` (sensor_msgs/Image, BGR).
  If you keep using `cv_ctrl.py` outside ROS, you can run a separate bridge or
  modify this node to pull frames directly from OpenCV VideoCapture.
• Model weights file is assumed to be located at:
    ~/WaveShare/models/yolov8n-signs.pt
  Adjust `--weights` argument otherwise.
"""

import os
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header

import numpy as np
import cv2
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # will warn later


CLASS_NAMES: List[str] = [
    'background',
    'person',
    'bicycle',
    'car',
    'motorcycle',
    'bus',
    'truck',
    'traffic light',
    'stop sign',
    'parking meter',
    'bench',
]


class YoloSignNode(Node):
    def __init__(self):
        super().__init__('yolo_sign_detector')

        # Declare parameters
        self.declare_parameter('weights', os.path.expanduser('~/WaveShare/models/yolov8n-signs.pt'))
        self.declare_parameter('conf', 0.25)
        weights = self.get_parameter('weights').value
        conf_thres = float(self.get_parameter('conf').value)

        # Load model
        if YOLO is None:
            self.get_logger().error('ultralytics not installed. Run `pip install ultralytics`')
            raise SystemExit(1)
        self.model = YOLO(weights)
        self.model.fuse()  # speed-up
        self.conf_thres = conf_thres
        self.get_logger().info(f"Loaded YOLO model from {weights}")

        # ROS interfaces
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                         depth=10,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, '/usb_cam/image_raw', self._img_cb, qos)
        self.pub = self.create_publisher(Detection2DArray, '/sign_detections', qos)
        self.bridge = CvBridge()

    # ------------------------------------------------------------------
    def _img_cb(self, msg: Image):
        # Convert to OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Inference -------------------------------------------------
        results = self.model.predict(source=cv_img, imgsz=320, conf=self.conf_thres, verbose=False)[0]

        det_array = Detection2DArray()
        det_array.header = msg.header  # copy timestamp / frame id

        for box, score, cls in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
            x1, y1, x2, y2 = box.cpu().numpy()
            w = x2 - x1
            h = y2 - y1
            cx = x1 + w / 2.0
            cy = y1 + h / 2.0

            det = Detection2D()
            det.bbox.center.x = float(cx)
            det.bbox.center.y = float(cy)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.id = int(cls)
            hyp.score = float(score)
            det.results.append(hyp)
            det_array.detections.append(det)

        self.pub.publish(det_array)


# ----------------------------------------------------------------------

def main():
    rclpy.init()
    node = YoloSignNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 