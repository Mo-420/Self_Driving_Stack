#!/usr/bin/env python3
"""
Simple Robotaxi Simulation
Runs entirely on Mac without ROS 2 or Gazebo
Uses the mock environment we already have working
"""

import sys
import os
import time
import threading
import math
import json

# Add the src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

# Import our mock environment
from simple_test import *

class SimpleSimulation:
    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.robot_speed = 0.0
        self.robot_angular_speed = 0.0
        
        # Obstacles in the world
        self.obstacles = [
            {'x': 2.0, 'y': 0.0, 'radius': 0.3},  # Front obstacle
            {'x': -1.0, 'y': 1.0, 'radius': 0.2},  # Left obstacle
            {'x': -1.0, 'y': -1.0, 'radius': 0.2}, # Right obstacle
        ]
        
        # Simulation state
        self.running = True
        self.time_step = 0.1  # 10 Hz
        
        # Node instances
        self.wander_node = None
        self.base_node = None
        self.safety_node = None
        
        # Message queues
        self.cmd_vel_messages = []
        self.safety_messages = []
        
    def start(self):
        """Start the simulation"""
        print("🤖 Starting Robotaxi Simulation")
        print("=" * 50)
        
        # Create and start nodes
        self._create_nodes()
        
        # Start simulation loop
        self._simulation_loop()
        
    def _create_nodes(self):
        """Create and initialize all nodes"""
        try:
            # Create wander node
            from waveshare_navigation.waveshare_navigation.wander_node import WanderNode
            self.wander_node = WanderNode()
            print("✅ Wander node created")
            
            # Create base node
            from waveshare_base.waveshare_base.base_node import WaveshareBaseNode
            self.base_node = WaveshareBaseNode()
            print("✅ Base node created")
            
            # Create safety node
            from waveshare_safety.waveshare_safety.ultrasonic_safety_node import CollisionPrevention
            self.safety_node = CollisionPrevention()
            print("✅ Safety node created")
            
        except Exception as e:
            print(f"❌ Error creating nodes: {e}")
            print("Using simplified simulation mode...")
            self.wander_node = None
            self.base_node = None
            self.safety_node = None
            
    def _simulation_loop(self):
        """Main simulation loop"""
        print("\n🚗 Starting simulation loop...")
        print("Press Ctrl+C to stop")
        
        try:
            while self.running:
                # Update robot position based on current speeds
                self._update_robot_position()
                
                # Check for obstacles
                distances = self._get_sensor_distances()
                
                # Simulate sensor readings
                self._simulate_sensors(distances)
                
                # Let nodes process
                self._process_nodes()
                
                # Display current state
                self._display_state(distances)
                
                time.sleep(self.time_step)
                
        except KeyboardInterrupt:
            print("\n🛑 Simulation stopped by user")
            self.running = False
            
    def _update_robot_position(self):
        """Update robot position based on current speeds"""
        dt = self.time_step
        
        # Update position
        self.robot_x += self.robot_speed * math.cos(self.robot_theta) * dt
        self.robot_y += self.robot_speed * math.sin(self.robot_theta) * dt
        self.robot_theta += self.robot_angular_speed * dt
        
        # Keep theta in [-pi, pi]
        while self.robot_theta > math.pi:
            self.robot_theta -= 2 * math.pi
        while self.robot_theta < -math.pi:
            self.robot_theta += 2 * math.pi
            
    def _get_sensor_distances(self):
        """Calculate distances to obstacles from robot position"""
        distances = {}
        
        # Front sensor (0 degrees)
        front_dist = self._distance_to_nearest_obstacle(0)
        distances['front'] = front_dist
        
        # Left sensor (90 degrees)
        left_dist = self._distance_to_nearest_obstacle(90)
        distances['left'] = left_dist
        
        # Right sensor (-90 degrees)
        right_dist = self._distance_to_nearest_obstacle(-90)
        distances['right'] = right_dist
        
        # Rear sensor (180 degrees)
        rear_dist = self._distance_to_nearest_obstacle(180)
        distances['rear'] = rear_dist
        
        return distances
        
    def _distance_to_nearest_obstacle(self, angle_deg):
        """Calculate distance to nearest obstacle in given direction"""
        angle_rad = math.radians(angle_deg)
        sensor_x = self.robot_x + math.cos(self.robot_theta + angle_rad) * 0.1
        sensor_y = self.robot_y + math.sin(self.robot_theta + angle_rad) * 0.1
        
        min_distance = 10.0  # Large default distance
        
        for obstacle in self.obstacles:
            dx = obstacle['x'] - sensor_x
            dy = obstacle['y'] - sensor_y
            distance = math.sqrt(dx*dx + dy*dy) - obstacle['radius']
            min_distance = min(min_distance, distance)
            
        return max(0.0, min_distance)
        
    def _simulate_sensors(self, distances):
        """Simulate sensor readings and send to nodes"""
        # Create safety status message
        safety_msg = String(data=json.dumps({
            'distances': distances,
            'mode': 'NORMAL',
            'override': '',
            'min_distance': min(distances.values())
        }))
        
        # Send to safety node if it exists and has the callback
        if self.safety_node and hasattr(self.safety_node, '_cb_safety'):
            self.safety_node._cb_safety(safety_msg)
            
    def _process_nodes(self):
        """Let nodes process their logic"""
        # This would normally involve calling timer callbacks
        # For now, we'll just simulate the basic behavior
        
        # Get current command from wander node
        if self.wander_node and hasattr(self.wander_node, '_control_loop'):
            # Simulate wander behavior
            front_dist = self._distance_to_nearest_obstacle(0)
            
            if front_dist < 0.6:  # Obstacle ahead
                # Turn left or right
                left_dist = self._distance_to_nearest_obstacle(90)
                right_dist = self._distance_to_nearest_obstacle(-90)
                
                if left_dist > right_dist:
                    self.robot_angular_speed = 0.6  # Turn left
                else:
                    self.robot_angular_speed = -0.6  # Turn right
                self.robot_speed = 0.05  # Slow forward
            else:
                # Clear path ahead
                self.robot_speed = 0.2  # Normal forward speed
                self.robot_angular_speed = 0.0  # No turning
        else:
            # Simplified behavior when nodes aren't available
            front_dist = self._distance_to_nearest_obstacle(0)
            
            if front_dist < 0.6:  # Obstacle ahead
                # Turn left or right
                left_dist = self._distance_to_nearest_obstacle(90)
                right_dist = self._distance_to_nearest_obstacle(-90)
                
                if left_dist > right_dist:
                    self.robot_angular_speed = 0.6  # Turn left
                else:
                    self.robot_angular_speed = -0.6  # Turn right
                self.robot_speed = 0.05  # Slow forward
            else:
                # Clear path ahead
                self.robot_speed = 0.2  # Normal forward speed
                self.robot_angular_speed = 0.0  # No turning
                
    def _display_state(self, distances):
        """Display current simulation state"""
        print(f"\r🤖 Robot: ({self.robot_x:.2f}, {self.robot_y:.2f}) θ:{math.degrees(self.robot_theta):.1f}° "
              f"Speed: {self.robot_speed:.2f} m/s "
              f"Sensors: F{distances['front']:.2f} L{distances['left']:.2f} R{distances['right']:.2f} "
              f"R{distances['rear']:.2f}m", end='', flush=True)

def main():
    print("🚗 Robotaxi Simple Simulation")
    print("This runs entirely on your Mac without ROS 2 or Gazebo")
    print("=" * 60)
    
    # Create and start simulation
    sim = SimpleSimulation()
    sim.start()

if __name__ == "__main__":
    main() 