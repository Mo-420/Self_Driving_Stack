#!/usr/bin/env python3
"""path_sequencer_node.py

Listens for a nav_msgs/Path message on /route_waypoints (produced by
route_receiver_node.py).  It then sequentially publishes each PoseStamped as
/goal_pose so that goal_follower_node (or Nav2) can drive to the way-points
one at a time.

It monitors /goal_follower/status (std_msgs/String) for the string
"GOAL_REACHED" to know when to advance to the next waypoint.  When all
way-points are done it publishes /trip_state with "complete"; on cancel (empty
Path) it publishes "cancelled".
"""

from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

STATUS_REACHED = "GOAL_REACHED"

class PathSequencer(Node):
    def __init__(self):
        super().__init__('path_sequencer')

        # Parameters ------------------------------------------------------
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('status_topic', '/goal_follower/status')
        self.declare_parameter('trip_state_topic', '/trip_state')

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        trip_topic = self.get_parameter('trip_state_topic').get_parameter_value().string_value

        # Publishers / Subscribers ----------------------------------------
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.trip_pub = self.create_publisher(String, trip_topic, 10)

        self.create_subscription(Path, 'route_waypoints', self._cb_path, 10)
        self.create_subscription(String, status_topic, self._cb_status, 10)

        # Internal state ---------------------------------------------------
        self._poses: List[PoseStamped] = []
        self._index: int = 0
        self._active: bool = False

        self.get_logger().info('PathSequencer ready – waiting for /route_waypoints')

    # ------------------------------------------------------------------
    def _cb_path(self, msg: Path):
        num = len(msg.poses)
        if num == 0:
            self.get_logger().info('Received empty route – cancelling current trip')
            self._set_trip_state('cancelled')
            self._active = False
            self._poses.clear()
            return

        # If first pose equals current location we skip it; cheaper: skip index 0
        self._poses = list(msg.poses)[1:] if num > 1 else list(msg.poses)
        self._index = 0
        self._active = bool(self._poses)

        if not self._poses:
            self.get_logger().warn('Route contained only current pose – nothing to do')
            self._set_trip_state('complete')
            return

        self.get_logger().info(f'Received new route with {len(self._poses)} way-points')
        self._set_trip_state('en_route')
        self._publish_current_goal()

    # ------------------------------------------------------------------
    def _cb_status(self, msg: String):
        if not self._active:
            return
        if msg.data == STATUS_REACHED:
            self._index += 1
            if self._index < len(self._poses):
                self.get_logger().info(f'Waypoint {self._index}/{len(self._poses)} reached – proceeding to next')
                self._publish_current_goal()
            else:
                self.get_logger().info('All way-points completed')
                self._set_trip_state('complete')
                self._active = False
                self._poses.clear()

    # ------------------------------------------------------------------
    def _publish_current_goal(self):
        if self._index < len(self._poses):
            goal = self._poses[self._index]
            # refresh timestamp
            goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(goal)
            self.get_logger().info(f'Published goal {self._index+1}/{len(self._poses)}')

    # ------------------------------------------------------------------
    def _set_trip_state(self, state: str):
        self.trip_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = PathSequencer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 