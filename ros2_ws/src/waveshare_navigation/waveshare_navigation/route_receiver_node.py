#!/usr/bin/env python3
"""route_receiver_node.py

Listens to the Robotaxi web-app Socket.IO server for `route` events and
converts the received list of GPS way-points into a nav_msgs/Path in the
robot's local ENU (East-North-Up) frame.  The node does *not* perform path
following itself; it only publishes the path so other nodes (e.g. Nav2
FollowWaypoints or a custom goal follower) can consume it.

Expected Socket.IO payload emitted by the web-app:

    {
      "id": "trip-42",
      "waypoints": [
        [37.78009, -122.40413],  # lat, lon (degrees)
        [37.78123, -122.40250],
        ...
      ]
    }

The node converts those lat/lon pairs into a local planar (x, y) metres
using a simple equirectangular approximation centred on the first waypoint.
For city-scale trips (< 10 km) the error is < 0.1 %%.

Parameters
----------
server_url  : str  Socket.IO endpoint (e.g. "http://192.168.1.10:5000")
namespace    : str  Socket.IO namespace (default "/")
frame_id     : str  TF frame for output path (default "map")
"""

import math
import threading
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

try:
    import socketio  # python-socketio client
except ImportError as e:  # pragma: no cover
    raise SystemExit("python-socketio not installed.  Run: pip install \"python-socketio[client]\"") from e

R_EARTH = 6371000.0  # mean Earth radius in metres

def gps_to_local(lat0: float, lon0: float, lat: float, lon: float):
    """Convert WGS-84 lat/lon to local ENU using equirectangular proj."""
    # convert degrees to radians
    lat0_rad = math.radians(lat0)
    lat_rad  = math.radians(lat)
    d_lat    = lat_rad - lat0_rad
    d_lon    = math.radians(lon - lon0)
    x = d_lon * math.cos(lat0_rad) * R_EARTH
    y = d_lat * R_EARTH
    return x, y

class RouteReceiverNode(Node):
    def __init__(self):
        super().__init__('route_receiver')

        # Params
        self.declare_parameter('server_url', 'http://localhost:5000')
        self.declare_parameter('namespace',   '/')
        self.declare_parameter('frame_id',    'map')

        self.server_url = self.get_parameter('server_url').get_parameter_value().string_value
        self.namespace  = self.get_parameter('namespace').get_parameter_value().string_value
        self.frame_id   = self.get_parameter('frame_id').get_parameter_value().string_value

        # Publisher
        self.path_pub = self.create_publisher(Path, 'route_waypoints', 10)

        # Spin Socket.IO client in a background thread so rclpy can spin freely
        self.sio = socketio.Client(reconnection=True, logger=False, engineio_logger=False)
        self.sio.on('route', self._on_route, namespace=self.namespace)
        self.sio.on('cancel', self._on_cancel, namespace=self.namespace)

        threading.Thread(target=self._start_socketio, daemon=True).start()
        self.get_logger().info(f"RouteReceiverNode connected to {self.server_url}{self.namespace}")

    # ------------------------------------------------------------------
    # Socket.IO event handlers
    # ------------------------------------------------------------------
    def _start_socketio(self):
        try:
            self.sio.connect(self.server_url, namespaces=[self.namespace])
            self.sio.wait()  # blocks forever (in background thread)
        except Exception as e:
            self.get_logger().error(f"Socket.IO connection error: {e}")

    def _on_route(self, data):  # pylint: disable=unused-argument
        """Handle a 'route' event, convert to Path and publish."""
        try:
            wp_list: List[List[float]] = data.get('waypoints', [])
            if not wp_list:
                self.get_logger().warn('Received route with no waypoints')
                return

            lat0, lon0 = wp_list[0]
            path_msg = Path()
            path_msg.header.frame_id = self.frame_id
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for idx, (lat, lon) in enumerate(wp_list):
                x, y = gps_to_local(lat0, lon0, lat, lon)
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.header.stamp = path_msg.header.stamp
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.orientation.w = 1.0  # facing forward; refine later if needed
                path_msg.poses.append(pose)
                if idx == 0:
                    self.get_logger().info(f"Route received: {len(wp_list)} waypoints (origin lat={lat0:.5f}, lon={lon0:.5f})")

            self.path_pub.publish(path_msg)
        except Exception as ex:  # pragma: no cover
            self.get_logger().error(f"Error processing route: {ex}")

    def _on_cancel(self, data):  # pylint: disable=unused-argument
        self.get_logger().info('Trip cancelled by web-app')
        # Publish empty path to signal stop
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path_msg)

# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RouteReceiverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 