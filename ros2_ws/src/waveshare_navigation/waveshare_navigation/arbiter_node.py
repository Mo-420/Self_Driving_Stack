#!/usr/bin/env python3
"""arbiter_node.py

Prioritises multiple velocity command sources and publishes one /cmd_vel.
Priority order (highest first):
1. E-stop asserted  -> publish zero
2. Road rules       -> /cmd_vel_rules
3. Nav2             -> /cmd_vel_nav2
4. Line follower    -> /cmd_vel_line
5. Remote control   -> /cmd_vel_remote

If a higher-priority topic has not published for `timeout_sec`, the arbiter
automatically falls back to the next priority.
"""

import time
from typing import Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

PRIORITY_TOPICS = [
    ('/cmd_vel_rules', 0),
    ('/cmd_vel_nav2', 1),
    ('/cmd_vel_line', 2),
    ('/cmd_vel_remote', 3),
]

class CmdArbiter(Node):
    def __init__(self):
        super().__init__('cmd_arbiter')
        self.declare_parameter('timeout_sec', 0.5)
        self.timeout = float(self.get_parameter('timeout_sec').value)

        # latest cmd and timestamp per topic
        self._cmd: Dict[str, Twist] = {t: Twist() for t, _ in PRIORITY_TOPICS}
        self._stamp: Dict[str, float] = {t: 0.0 for t, _ in PRIORITY_TOPICS}
        self._estop = False

        # subscriptions
        for topic, _ in PRIORITY_TOPICS:
            self.create_subscription(Twist, topic, self._make_cb(topic), 10)
        self.create_subscription(Bool, '/estop', self._cb_estop, 1)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info('Cmd arbiter active')

    def _make_cb(self, topic):
        def _cb(msg):
            self._cmd[topic] = msg
            self._stamp[topic] = time.time()
        return _cb

    def _cb_estop(self, msg: Bool):
        if msg.data:
            self._estop = True

    def _tick(self):
        out = Twist()
        if self._estop:
            self.pub.publish(out)
            return

        now = time.time()
        for topic, _ in PRIORITY_TOPICS:
            if now - self._stamp[topic] <= self.timeout:
                out = self._cmd[topic]
                break  # highest priority active
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 