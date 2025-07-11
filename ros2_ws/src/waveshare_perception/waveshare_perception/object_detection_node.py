#!/usr/bin/env python3
"""
object_detection_node.py
------------------------
Unified YOLO-based detector that finds common road agents:
    • person (pedestrian)
    • bicycle
    • car / bus / truck (all published as class CAR)
    • traffic cone / barrel (class CONE)

Publishes: `/detected_objects` (vision_msgs/Detection2DArray)
Camera topic:  `/usb_cam/image_raw` (sensor_msgs/Image, BGR)

Parameters
~~~~~~~~~~
weights   — path to YOLOv8 model weights (default: coco-pretrained yolov8n.pt)
conf      — confidence threshold (default: 0.25)

This node is a superset of yolo_sign_node.py and can replace it when sign
classes are also included in the weights.
"""
from __future__ import annotations

import os
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header

import cv2
import numpy as np
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # handled in __init__

# Map YOLO class names -> simplified agent classes
YOLO_TO_AGENT = {
    'person': 'PEDESTRIAN',
    'bicycle': 'BICYCLE',
    'motorcycle': 'BICYCLE',  # treat together for now
    'car': 'CAR',
    'bus': 'CAR',
    'truck': 'CAR',
    'traffic light': 'TRAFFIC_LIGHT',
    'stop sign': 'SIGN',
    'traffic cone': 'CONE',  # custom name in some datasets
}
# For COCO cones do not exist; many datasets use class 49 (bottle) or 0. We rely
# on fine-tuned weights where cone is labelled. If absent, code simply skips.

AGENT_TO_ID = {name: idx for idx, name in enumerate(sorted(set(YOLO_TO_AGENT.values())))}


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Declare parameters
        default_weights = os.path.expanduser('~/WaveShare/models/yolov8n-road.pt')
        self.declare_parameter('weights', default_weights)
        self.declare_parameter('conf', 0.25)
        weight_path = self.get_parameter('weights').value
        conf_thres = float(self.get_parameter('conf').value)

        # Load model ------------------------------------------------------
        if YOLO is None:
            self.get_logger().error('ultralytics not installed. `pip install ultralytics`')
            raise SystemExit(1)
        if not os.path.exists(weight_path):
            self.get_logger().warn(f'Weights not found at {weight_path}. Loading default yolov8n pretrained model.')
            self.model = YOLO('yolov8n.pt')
        else:
            self.model = YOLO(weight_path)
        self.model.fuse()
        self.conf_thres = conf_thres
        self.get_logger().info(f'YOLO model ready ({weight_path}) conf>={conf_thres}')

        # ROS I/O ---------------------------------------------------------
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, '/usb_cam/image_raw', self._img_cb, qos)
        self.pub = self.create_publisher(Detection2DArray, '/detected_objects', qos)
        self.bridge = CvBridge()

    # ------------------------------------------------------------------
    def _img_cb(self, msg: Image):
        img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(source=img_bgr, imgsz=320, conf=self.conf_thres, verbose=False)[0]

        det_array = Detection2DArray()
        det_array.header = msg.header

        for i in range(len(results.boxes)):
            cls_idx = int(results.boxes.cls[i])
            conf = float(results.boxes.conf[i])
            x1, y1, x2, y2 = results.boxes.xyxy[i].cpu().numpy()
            label = self.model.names[cls_idx]
            if label not in YOLO_TO_AGENT:
                continue  # skip classes we don't care about
            agent = YOLO_TO_AGENT[label]
            agent_id = AGENT_TO_ID[agent]

            # Build Detection2D
            det = Detection2D()
            det.bbox.center.x = float((x1 + x2) / 2)
            det.bbox.center.y = float((y1 + y2) / 2)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.id = agent_id
            hyp.score = conf
            det.results.append(hyp)
            det_array.detections.append(det)

        self.pub.publish(det_array)


# ----------------------------------------------------------------------

def main():
    rclpy.init()
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 