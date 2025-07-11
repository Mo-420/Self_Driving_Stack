#!/usr/bin/env python3
"""
waveshare_safety/ultrasonic_safety_node.py
-----------------------------------------
ROS 2 node that wraps the UltrasonicSensor + CollisionPrevention logic already
implemented in `robotaxi_enhanced.py` and exposes:
  * /safety_status   (std_msgs/String JSON for now)
  * /cmd_vel_safe    (geometry_msgs/Twist) – filtered velocity command

Usage:
    ros2 run waveshare_safety ultrasonic_safety_node.py

This node subscribes to the raw `/cmd_vel` coming from Navigation2 or tele-op,
checks the ultrasonic sensors, and republishes a safe version on
`/cmd_vel_safe`.  Down-stream nodes (waveshare_base) should listen to the safe
variant.
"""

import json
from threading import Lock

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from robotaxi_enhanced import UltrasonicSensor  # reuse class


class CollisionPrevention(Node):
    def __init__(self):
        super().__init__('ultrasonic_safety')

        # Parameters ----------------------------------------------------
        self.declare_parameter('emergency_stop_distance', 0.20)  # metres
        self.declare_parameter('slow_down_distance',     0.50)  # metres

        self.em_stop_d = self.get_parameter('emergency_stop_distance').value
        self.slow_d    = self.get_parameter('slow_down_distance').value

        # Sensors -------------------------------------------------------
        # (use same GPIO map as earlier example)
        self.sensors = {
            'front': UltrasonicSensor(23, 24),
            'left':  UltrasonicSensor(17, 27),
            'right': UltrasonicSensor(22, 10),
            'rear':  UltrasonicSensor(9, 11),
        }

        # Publishers / Subscribers -------------------------------------
        qos_depth = 10
        self.cmd_sub  = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, qos_depth)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel_safe', qos_depth)
        self.status_pub = self.create_publisher(String, '/safety_status', qos_depth)

        # Internal state
        self._lock = Lock()
        self._raw_cmd = Twist()
        # Simple finite-state machine for avoidance
        self._mode = 'NORMAL'           # NORMAL or AVOIDING
        self._avoid_direction = None    # 'left' or 'right'

        # Timer to evaluate safety at 10 Hz
        self.create_timer(0.1, self._safety_loop)

    # --------------------------------------------------------------
    def _cmd_cb(self, msg: Twist):
        with self._lock:
            self._raw_cmd = msg

    # --------------------------------------------------------------
    def _safety_loop(self):
        # ----------------------------------------------------------
        # 1. Read sensor distances (m)
        distances = {name: sensor.get_distance()/100.0 for name, sensor in self.sensors.items()}
        front_d = distances.get('front', 1.0)
        left_d  = distances.get('left', 1.0)
        right_d = distances.get('right', 1.0)
        rear_d  = distances.get('rear', 1.0)
        min_d   = min(distances.values())

        # 2. Start with the user request
        with self._lock:
            safe_cmd = Twist()
            safe_cmd.linear.x  = self._raw_cmd.linear.x
            safe_cmd.angular.z = self._raw_cmd.angular.z

        override = ''

        # Emergency stop always wins
        if min_d <= self.em_stop_d:
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0
            self._mode = 'NORMAL'
            override = 'EMERGENCY_STOP'

        # Obstacle within avoidance zone directly ahead
        elif front_d <= self.slow_d:
            # If not already avoiding, choose side with more space
            if self._mode == 'NORMAL':
                self._mode = 'AVOIDING'
                self._avoid_direction = 'left' if left_d > right_d else 'right'

            # Produce avoidance twist
            safe_cmd.linear.x = 0.05  # creep forward
            turn_speed = 0.6
            safe_cmd.angular.z = turn_speed if self._avoid_direction == 'left' else -turn_speed
            if self._avoid_direction is not None:
                override = f'AVOID_{self._avoid_direction.upper()}'
            else:
                override = 'AVOID'

            # If path ahead is now clear, exit avoidance
            if front_d > self.slow_d and min(left_d, right_d) > self.slow_d:
                self._mode = 'NORMAL'

        else:
            # No obstacle ahead – normal operation
            self._mode = 'NORMAL'

            # Slow mode if any side object is close
            if min_d <= self.slow_d:
                safe_cmd.linear.x = min(safe_cmd.linear.x, 0.1)
                override = 'SLOW_MODE'

        # 3. Publish results
        self.cmd_pub.publish(safe_cmd)

        status_msg = {
            'override': override,
            'mode': self._mode,
            'min_distance': round(min_d, 2),
            'distances': {k: round(v, 2) for k, v in distances.items()}
        }
        self.status_pub.publish(String(data=json.dumps(status_msg)))


def main():
    rclpy.init()
    node = CollisionPrevention()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 