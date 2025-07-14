#!/usr/bin/env python3
"""YOLOv8 Crosswalk Detector Node

Publishes:
  /crosswalk_detected   std_msgs/Bool  (True when a crosswalk is visible)
  /crosswalk_bbox       vision_msgs/Detection2DArray (optional bboxes)

Expects an image topic `/usb_cam/image_raw` (sensor_msgs/Image, BGR) and a
YOLOv8 model fine-tuned on zebra crossings located at
`~/WaveShare/models/yolov8n-cross.pt`.
"""
import os
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import cv2

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

CLASS_NAME = 'crosswalk'  # single-class model


class YoloCrosswalkNode(Node):
    def __init__(self):
        super().__init__('yolo_crosswalk_detector')
        # ---- params ----
        self.declare_parameter('weights', os.path.expanduser('~/WaveShare/models/yolov8n-cross.pt'))
        self.declare_parameter('conf', 0.3)
        w_path = self.get_parameter('weights').value
        conf_thres = float(self.get_parameter('conf').value)

        if YOLO is None:
            self.get_logger().error('ultralytics not installed.  `pip install ultralytics`')
            raise SystemExit(1)
        self.model = YOLO(w_path)
        self.model.fuse()
        self.conf_thres = conf_thres
        self.get_logger().info(f'Loaded crosswalk YOLO model: {w_path}')

        qos_depth = 10
        self.bridge = CvBridge()
        self.create_subscription(Image, '/usb_cam/image_raw', self._img_cb, qos_depth)
        self.pub_det = self.create_publisher(Detection2DArray, '/crosswalk_bbox', qos_depth)
        self.pub_flag = self.create_publisher(Bool, '/crosswalk_detected', 1)

    # ----------------------------------------------------------------
    def _img_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(source=cv_img, imgsz=320, conf=self.conf_thres, verbose=False)[0]

        detected = False
        det_array = Detection2DArray()
        det_array.header = msg.header

        for box, score in zip(results.boxes.xyxy, results.boxes.conf):
            detected = True
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
            hyp.id = 0
            hyp.score = float(score)
            det.results.append(hyp)
            det_array.detections.append(det)

        # Publish topics
        self.pub_flag.publish(Bool(data=detected))
        if detected:
            self.pub_det.publish(det_array)


def main():
    rclpy.init()
    node = YoloCrosswalkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 