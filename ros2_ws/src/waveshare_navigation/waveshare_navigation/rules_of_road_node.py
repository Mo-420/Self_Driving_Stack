#!/usr/bin/env python3
"""
waveshare_navigation/rules_of_road_node.py
------------------------------------------
Enforces basic road rules:
• Stop for 2 s when a stop-sign is detected ahead.
• Freeze when traffic-light is red, release on green, slow on yellow.

It sits between any high-level motion source (goal-follower, tele-op, Nav2)
and the ultrasonic safety layer.

Subscriptions
-------------
/cmd_vel            geometry_msgs/Twist – original motion request.
/sign_detections    vision_msgs/Detection2DArray – YOLO sign outputs.
/traffic_light      std_msgs/String – state ("red|yellow|green|none").

Publication
-----------
/cmd_vel_rules      geometry_msgs/Twist – rule-filtered motion.
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

STOP_SIGN_CLASS_ID = 8          # COCO stop-sign
STOP_HOLD_SEC = 2.0
STOP_COOLDOWN_SEC = 5.0         # before we can stop again

class RuleNode(Node):
    def __init__(self):
        super().__init__('rules_of_road')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_rules', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd, 20)
        self.create_subscription(Detection2DArray, '/sign_detections', self._cb_signs, 10)
        self.create_subscription(Detection2DArray, '/detected_objects', self._cb_objects, 10)
        self.create_subscription(String, '/traffic_light', self._cb_light, 10)

        # Front-zone hold flags
        self._pedestrian_ahead: bool = False
        self._bike_ahead: bool = False
        self._car_oncoming: bool = False

        self._last_cmd: Twist = Twist()
        self._last_stop_time: float = 0.0
        self._stop_cooldown_until: float = 0.0
        self._light_state: str = 'none'

        # Timer 20 Hz to forward/hold commands
        self.create_timer(0.05, self._tick)

    # --------------------------------------------------
    def _cb_cmd(self, msg: Twist):
        self._last_cmd = msg

    # --------------------------------------------------
    def _cb_signs(self, msg: Detection2DArray):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now < self._stop_cooldown_until:
            return  # still cooling down from previous stop
        for det in msg.detections:
            for hyp in det.results:
                if hyp.id == STOP_SIGN_CLASS_ID and hyp.score > 0.5:
                    self._last_stop_time = now
                    self._stop_cooldown_until = now + STOP_COOLDOWN_SEC
                    self.get_logger().info('Stop-sign detected – pausing 2 s')
                    return

    # --------------------------------------------------
    def _cb_light(self, msg: String):
        self._light_state = msg.data.lower().strip()

    # --------------------------------------------------
    #   Object right-of-way logic
    # --------------------------------------------------
    # Agent IDs must match object_detection_node.py AGENT_TO_ID alphabetical order
    AGENT_ID = {
        'BICYCLE': 0,
        'CAR': 1,
        'CONE': 2,
        'PEDESTRIAN': 3,
        'SIGN': 4,
        'TRAFFIC_LIGHT': 5,
    }

    def _cb_objects(self, msg: Detection2DArray):
        """Set internal flags when a protected agent is directly ahead."""
        # Reset flags each frame; set to True if we see something dangerous
        self._pedestrian_ahead = False
        self._bike_ahead = False
        self._car_oncoming = False

        for det in msg.detections:
            if not det.results:
                continue
            cls_id = det.results[0].id  # highest-score hyp first

            # assume image width 640, treat central 1/3rd as directly ahead
            cx = det.bbox.center.x
            img_w = 640.0  # configurable later
            central = (img_w * 0.33) < cx < (img_w * 0.66)
            if not central:
                continue

            if cls_id == self.AGENT_ID['PEDESTRIAN']:
                self._pedestrian_ahead = True
            elif cls_id == self.AGENT_ID['BICYCLE']:
                self._bike_ahead = True
            elif cls_id == self.AGENT_ID['CAR']:
                self._car_oncoming = True

    # --------------------------------------------------
    def _tick(self):
        out = Twist()
        now = self.get_clock().now().seconds_nanoseconds()[0]

        # Rule 1: active stop-sign pause
        if now - self._last_stop_time < STOP_HOLD_SEC:
            # keep out zero
            self.cmd_pub.publish(out)
            return

        # Rule 2: traffic light states
        if self._light_state == 'red':
            # full freeze
            self.cmd_pub.publish(out)
            return
        elif self._light_state == 'yellow':
            # slow down – cap forward speed
            out = Twist()
            out.linear.x = min(self._last_cmd.linear.x, 0.05)
            out.angular.z = self._last_cmd.angular.z
            self.cmd_pub.publish(out)
            return
        # Rule 3: pedestrian or bike ahead – full stop
        if self._pedestrian_ahead or self._bike_ahead:
            self.cmd_pub.publish(out)
            return

        # Rule 4: oncoming car in lane – slow
        if self._car_oncoming:
            out = Twist()
            out.linear.x = min(self._last_cmd.linear.x, 0.05)
            out.angular.z = self._last_cmd.angular.z
            self.cmd_pub.publish(out)
            return

        # Green light / clear path → forward original cmd
        self.cmd_pub.publish(self._last_cmd)


def main():
    rclpy.init()
    node = RuleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 