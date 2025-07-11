#!/usr/bin/env python3
"""
telemetry_sender_node.py
------------------------
Sends robot telemetry back to the web app via Socket.IO.
Subscribes to odometry, battery voltage, and system status,
then emits periodic updates to the server.

Telemetry payload:
{
    "robot_id": "robotaxi-001",
    "timestamp": 1234567890.123,
    "position": {
        "lat": 37.7801,
        "lon": -122.404,
        "heading": 1.57  # radians
    },
    "battery": {
        "voltage": 12.6,
        "percentage": 85
    },
    "status": {
        "state": "active",  # idle, active, emergency
        "speed": 0.25,      # m/s
        "trip_id": "trip-42"
    }
}
"""

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

try:
    import socketio
except ImportError as e:
    raise SystemExit("python-socketio not installed. Run: pip install \"python-socketio[client]\"") from e

# Constants for GPS conversion (same as route_receiver)
R_EARTH = 6371000.0

def local_to_gps(lat0: float, lon0: float, x: float, y: float):
    """Convert local ENU coordinates back to GPS"""
    lat0_rad = math.radians(lat0)
    d_lat = y / R_EARTH
    d_lon = x / (R_EARTH * math.cos(lat0_rad))
    lat = lat0 + math.degrees(d_lat)
    lon = lon0 + math.degrees(d_lon)
    return lat, lon

class TelemetrySenderNode(Node):
    def __init__(self):
        super().__init__('telemetry_sender')
        
        # Parameters
        self.declare_parameter('server_url', 'http://localhost:5000')
        self.declare_parameter('namespace', '/')
        self.declare_parameter('robot_id', 'robotaxi-001')
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('origin_lat', 37.7801)  # Default SF coordinates
        self.declare_parameter('origin_lon', -122.404)
        
        self.server_url = self.get_parameter('server_url').value
        self.namespace = self.get_parameter('namespace').value
        self.robot_id = self.get_parameter('robot_id').value
        self.update_rate = self.get_parameter('update_rate').value
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        
        # State
        self.position_x = 0.0
        self.position_y = 0.0
        self.heading = 0.0
        self.linear_speed = 0.0
        self.battery_voltage = 12.0
        self.robot_state = "idle"
        self.current_trip_id: Optional[str] = None
        
        # Subscribers
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_callback, qos
        )
        
        self.battery_sub = self.create_subscription(
            Float32, 'battery_voltage', self._battery_callback, qos
        )
        
        self.state_sub = self.create_subscription(
            String, 'robot_state', self._state_callback, qos
        )
        
        # Socket.IO client
        self.sio = socketio.Client(reconnection=True, logger=False, engineio_logger=False)
        self.sio.on('connect', self._on_connect, namespace=self.namespace)
        self.sio.on('trip_assigned', self._on_trip_assigned, namespace=self.namespace)
        
        # Start Socket.IO in background
        threading.Thread(target=self._start_socketio, daemon=True).start()
        
        # Timer for periodic telemetry
        self.create_timer(1.0 / self.update_rate, self._send_telemetry)
        
        self.get_logger().info(f"TelemetrySender started: {self.robot_id} -> {self.server_url}")
    
    def _start_socketio(self):
        """Connect to Socket.IO server"""
        try:
            self.sio.connect(self.server_url, namespaces=[self.namespace])
            self.sio.wait()
        except Exception as e:
            self.get_logger().error(f"Socket.IO connection error: {e}")
    
    def _on_connect(self):
        """Handle connection to server"""
        self.get_logger().info("Connected to telemetry server")
        # Register robot
        self.sio.emit('register_robot', {
            'robot_id': self.robot_id,
            'capabilities': ['autonomous', 'telemetry', 'remote_control']
        }, namespace=self.namespace)
    
    def _on_trip_assigned(self, data):
        """Handle trip assignment from server"""
        self.current_trip_id = data.get('trip_id')
        self.robot_state = "active"
        self.get_logger().info(f"Trip assigned: {self.current_trip_id}")
    
    def _odom_callback(self, msg: Odometry):
        """Update position from odometry"""
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.heading = math.atan2(siny_cosp, cosy_cosp)
        
        # Speed
        self.linear_speed = math.sqrt(
            msg.twist.twist.linear.x ** 2 + 
            msg.twist.twist.linear.y ** 2
        )
    
    def _battery_callback(self, msg: Float32):
        """Update battery voltage"""
        self.battery_voltage = msg.data
    
    def _state_callback(self, msg: String):
        """Update robot state"""
        self.robot_state = msg.data
    
    def _send_telemetry(self):
        """Send telemetry update to server"""
        if not self.sio.connected:
            return
        
        # Convert local position to GPS
        lat, lon = local_to_gps(
            self.origin_lat, self.origin_lon,
            self.position_x, self.position_y
        )
        
        # Calculate battery percentage (3S LiPo: 12.6V = 100%, 9.6V = 0%)
        battery_pct = max(0, min(100, 
            (self.battery_voltage - 9.6) / (12.6 - 9.6) * 100
        ))
        
        telemetry = {
            "robot_id": self.robot_id,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "position": {
                "lat": lat,
                "lon": lon,
                "heading": self.heading
            },
            "battery": {
                "voltage": self.battery_voltage,
                "percentage": int(battery_pct)
            },
            "status": {
                "state": self.robot_state,
                "speed": self.linear_speed,
                "trip_id": self.current_trip_id
            }
        }
        
        self.sio.emit('telemetry', telemetry, namespace=self.namespace)

def main(args=None):
    rclpy.init(args=args)
    node = TelemetrySenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 