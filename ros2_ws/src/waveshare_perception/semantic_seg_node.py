#!/usr/bin/env python3
"""Real-time semantic segmentation → costmap layer

A lightweight FCN-ResNet50 model from torchvision is used to classify each
pixel as ‘sidewalk’ vs everything else.  The node publishes an 8-bit
nav_msgs/OccupancyGrid on `/semantic_costmap`, where:
  0   = traversable sidewalk
  100 = unknown/neutral (grass, path, etc.)
  254 = lethal obstacle (road, building, car …)
Frame is `map` and resolution is 0.05 m/px to match Nav2 global costmap.
NOTE: The default pre-trained model is for Cityscapes; for production replace
with a Pi-optimised model or Coral-TPU model.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from cv_bridge import CvBridge
import torch
import torchvision.transforms as T
from torchvision.models.segmentation import fcn_resnet50
import numpy as np
import cv2

SIDEWALK_CLASS_ID = 11   # sidewalk in Cityscapes palette
ROAD_CLASS_ID = 0        # road

class SemanticSegNode(Node):
    def __init__(self):
        super().__init__('semantic_seg_node')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('update_rate', 2.0)
        self.res = float(self.get_parameter('resolution').value)

        # model ----------------------------------------------------
        self.get_logger().info('Loading FCN-ResNet50 (torchvision)…')
        self.model = fcn_resnet50(pretrained=True, progress=False)
        self.model.eval()
        self.model.to('cpu')  # Pi CPU inference (~1 Hz) – replace with faster model/GPU
        self.tf = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406],
                          std=[0.229, 0.224, 0.225])
        ])

        # ROS I/O --------------------------------------------------
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/usb_cam/image_raw', self._img_cb, 10)
        self.pub = self.create_publisher(OccupancyGrid, '/semantic_costmap', 1)

        self._last_pub = self.get_clock().now()

    # ----------------------------------------------------------
    def _img_cb(self, msg: Image):
        # throttle to update_rate
        if (self.get_clock().now() - self._last_pub).nanoseconds < 1e9 / self.get_parameter('update_rate').value:
            return
        self._last_pub = self.get_clock().now()

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_img.shape
        inp = self.tf(cv2.resize(cv_img, (512, 256))).unsqueeze(0)
        with torch.no_grad():
            out = self.model(inp)['out'][0].argmax(0).cpu().numpy()

        # build costmap (downscale /2 to ~256×128)
        scale = 2
        out_small = cv2.resize(out.astype(np.uint8), (w//scale, h//scale), interpolation=cv2.INTER_NEAREST)
        cost = np.full_like(out_small, 100, dtype=np.uint8)  # unknown default
        cost[out_small == SIDEWALK_CLASS_ID] = 0
        cost[out_small == ROAD_CLASS_ID] = 254

        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = msg.header.stamp
        grid.header.frame_id = 'map'   # assuming odom==map
        grid.info.resolution = self.res
        grid.info.width  = cost.shape[1]
        grid.info.height = cost.shape[0]
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.data = cost.flatten().tolist()
        self.pub.publish(grid)


def main():
    rclpy.init()
    node = SemanticSegNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 