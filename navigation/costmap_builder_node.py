#!/usr/bin/env python3
"""costmap_builder_node.py
Compiles ultrasonic and vision detections into LocalCostmap.
Publishes nothing yet; NavigationController queries costmap instance.
"""
from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray

from .navigation_controller import NavigationController
from .local_costmap import LocalCostmap

class CostmapBuilderNode(Node):
    def __init__(self):
        super().__init__('local_costmap_builder')
        self.costmap = LocalCostmap()
        self.controller = NavigationController.get_instance()
        self.controller.costmap = self.costmap  # share instance

        self.create_subscription(Float32, '/ultrasonic_front', self._cb_ultra, 10)
        self.create_subscription(Detection2DArray, '/detected_objects', self._cb_objects, 10)

    def _cb_ultra(self, msg: Float32):
        # Mark an obstacle directly ahead at the reported distance
        dist = float(msg.data)
        if dist <= 0.0 or dist > 4.0:
            return
        x_robot, y_robot = self.controller._tracker.position
        heading = self.controller._tracker.heading
        wx = x_robot + dist * math.cos(heading)
        wy = y_robot + dist * math.sin(heading)
        self.costmap.mark_obstacle(wx, wy, (x_robot, y_robot))

    def _cb_objects(self, msg: Detection2DArray):
        x_robot, y_robot = self.controller._tracker.position
        for det in msg.detections:
            # Approx distance by bbox height; assume pedestrian 1.7 m tall, camera fx=600 px
            if not det.results:
                continue
            h_pixels = det.bbox.size_y
            if h_pixels <= 0:
                continue
            est_dist = 600.0 / h_pixels  # very crude
            cx = det.bbox.center.x
            img_w = 640.0
            rel_angle = (cx - img_w / 2) / img_w * (60 * math.pi/180)  # assume 60° FOV
            heading = self.controller._tracker.heading + rel_angle
            wx = x_robot + est_dist * math.cos(heading)
            wy = y_robot + est_dist * math.sin(heading)
            self.costmap.mark_obstacle(wx, wy, (x_robot, y_robot))


def main():
    rclpy.init()
    node = CostmapBuilderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 