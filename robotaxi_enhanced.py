#!/usr/bin/env python3
"""
Enhanced WaveShare Robotaxi Controller
Builds on existing Waveshare UGV code with safety and navigation features
"""

import cv2
import numpy as np
import time
import threading
import json
import math
from collections import deque
import yaml
import os
import argparse

# Import existing Waveshare modules
from base_ctrl import BaseController
import cv_ctrl

class UltrasonicSensor:
    """Ultrasonic distance sensor interface"""
    
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.last_distance = 0
        self.sensor_ready = False
        
        # Try to initialize GPIO (will fail on non-Pi systems)
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            self.sensor_ready = True
            print(f"Ultrasonic sensor initialized on pins {trigger_pin}, {echo_pin}")
        except ImportError:
            print("GPIO not available - running in simulation mode")
            self.sensor_ready = False
    
    def get_distance(self):
        """Get distance measurement in cm"""
        if not self.sensor_ready:
            # Simulation mode - return random distance
            import random
            return random.uniform(30, 200)
        
        try:
            import RPi.GPIO as GPIO
            
            # Send trigger pulse
            GPIO.output(self.trigger_pin, False)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, True)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, False)
            
            # Measure echo time
            start_time = time.time()
            while GPIO.input(self.echo_pin) == 0:
                start_time = time.time()
                if time.time() - start_time > 0.1:  # Timeout
                    return self.last_distance
            
            stop_time = time.time()
            while GPIO.input(self.echo_pin) == 1:
                stop_time = time.time()
                if time.time() - start_time > 0.1:  # Timeout
                    return self.last_distance
            
            # Calculate distance (speed of sound = 34300 cm/s)
            duration = stop_time - start_time
            distance = (duration * 34300) / 2
            
            # Filter out invalid readings
            if 2 < distance < 400:  # Valid range 2cm to 4m
                self.last_distance = distance
                return distance
            else:
                return self.last_distance
                
        except Exception as e:
            print(f"Ultrasonic sensor error: {e}")
            return self.last_distance

class CollisionPrevention:
    """Collision prevention and safety system"""
    
    def __init__(self, base_controller):
        self.base_ctrl = base_controller
        
        # Safety thresholds
        self.emergency_stop_distance = 20  # cm
        self.slow_down_distance = 50       # cm
        self.warning_distance = 100        # cm
        
        # Sensor configuration
        self.sensors = {
            'front': UltrasonicSensor(23, 24),  # GPIO pins
            'left': UltrasonicSensor(17, 27),
            'right': UltrasonicSensor(22, 10),
            'rear': UltrasonicSensor(9, 11)
        }
        
        # Safety state
        self.emergency_stop_active = False
        self.slow_mode_active = False
        self.last_safety_check = time.time()
        
        # Start safety monitoring thread
        self.safety_thread = threading.Thread(target=self._safety_monitor, daemon=True)
        self.safety_thread.start()
    
    def _safety_monitor(self):
        """Continuous safety monitoring thread"""
        while True:
            try:
                self._check_safety()
                time.sleep(0.1)  # 10Hz safety checks
            except Exception as e:
                print(f"Safety monitor error: {e}")
    
    def _check_safety(self):
        """Check all sensors and apply safety measures"""
        min_distance = float('inf')
        closest_sensor = None
        
        # Check all sensors
        for sensor_name, sensor in self.sensors.items():
            distance = sensor.get_distance()
            if distance < min_distance:
                min_distance = distance
                closest_sensor = sensor_name
        
        # Apply safety measures
        if min_distance <= self.emergency_stop_distance:
            if not self.emergency_stop_active:
                print(f"EMERGENCY STOP: {closest_sensor} sensor detected {min_distance:.1f}cm")
                self._emergency_stop()
                self.emergency_stop_active = True
                self.slow_mode_active = False
                
        elif min_distance <= self.slow_down_distance:
            if not self.slow_mode_active:
                print(f"SLOW MODE: {closest_sensor} sensor detected {min_distance:.1f}cm")
                self.slow_mode_active = True
                self.emergency_stop_active = False
                
        else:
            # Clear safety states
            self.emergency_stop_active = False
            self.slow_mode_active = False
        
        self.last_safety_check = time.time()
    
    def _emergency_stop(self):
        """Emergency stop all motors"""
        try:
            self.base_ctrl.base_json_ctrl({"T": 13, "X": 0, "Z": 0})
            print("Emergency stop executed")
        except Exception as e:
            print(f"Emergency stop error: {e}")
    
    def is_safe_to_move(self, speed, direction):
        """Check if it's safe to move with given parameters"""
        if self.emergency_stop_active:
            return False, "Emergency stop active"
        
        if self.slow_mode_active and abs(speed) > 0.3:
            return False, "Slow mode active - reduce speed"
        
        return True, "Safe to move"
    
    def get_safety_status(self):
        """Get current safety status"""
        return {
            'emergency_stop': self.emergency_stop_active,
            'slow_mode': self.slow_mode_active,
            'sensor_distances': {
                name: sensor.get_distance() 
                for name, sensor in self.sensors.items()
            },
            'last_check': self.last_safety_check
        }

class EnhancedLineFollower:
    """Enhanced line following with obstacle integration"""
    
    def __init__(self, cv_controller, collision_prevention):
        self.cv_ctrl = cv_controller
        self.collision_prevention = collision_prevention
        
        # Enhanced parameters
        self.base_speed = 0.3
        self.max_speed = 0.5
        self.min_speed = 0.1
        self.turn_sensitivity = 1.5
        
        # Line detection parameters
        self.line_color_lower = np.array([25, 150, 70])   # Yellow
        self.line_color_upper = np.array([42, 255, 255])
        
        # Safety integration
        self.safety_override = False
        self.last_line_detected = time.time()
        self.line_lost_timeout = 2.0  # seconds
        
    def follow_line(self, frame):
        """Enhanced line following with safety checks"""
        # Get safety status
        safety_status = self.collision_prevention.get_safety_status()
        
        # Check if emergency stop is needed
        if safety_status['emergency_stop']:
            self._stop_movement()
            return frame, "Emergency stop - obstacle detected"
        
        # Detect line
        line_center, line_confidence = self._detect_line(frame)
        
        if line_confidence > 0.5:
            self.last_line_detected = time.time()
            
            # Calculate movement parameters
            speed, turning = self._calculate_movement(line_center, line_confidence)
            
            # Apply safety speed limits
            if safety_status['slow_mode']:
                speed *= 0.5  # Reduce speed in slow mode
            
            # Execute movement
            is_safe, message = self.collision_prevention.is_safe_to_move(speed, turning)
            if is_safe:
                self.cv_ctrl.base_ctrl.base_json_ctrl({"T": 13, "X": speed, "Z": turning})
                status = f"Following line - Speed: {speed:.2f}, Turn: {turning:.2f}"
            else:
                self._stop_movement()
                status = f"Safety override: {message}"
        else:
            # Line lost
            if time.time() - self.last_line_detected > self.line_lost_timeout:
                self._stop_movement()
                status = "Line lost - stopping"
            else:
                status = "Line lost - searching"
        
        # Add visual overlay
        overlay = self._create_overlay(frame, line_center, line_confidence, safety_status)
        return overlay, status
    
    def _detect_line(self, frame):
        """Detect line in the frame"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for line color
        mask = cv2.inRange(hsv, self.line_color_lower, self.line_color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find line contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour (main line)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 100:  # Minimum area threshold
                # Calculate line center
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Calculate confidence based on area and position
                    frame_center = frame.shape[1] // 2
                    center_offset = abs(cx - frame_center) / frame_center
                    confidence = min(1.0, area / 1000) * (1 - center_offset)
                    
                    return (cx, cy), confidence
        
        return None, 0.0
    
    def _calculate_movement(self, line_center, confidence):
        """Calculate speed and turning based on line position"""
        if line_center is None:
            return 0, 0
        
        frame_center = 320  # Assuming 640x480 frame
        line_x, line_y = line_center
        
        # Calculate turning (negative for left, positive for right)
        center_offset = (line_x - frame_center) / frame_center
        turning = -center_offset * self.turn_sensitivity
        
        # Calculate speed based on confidence and turning
        base_speed = self.base_speed * confidence
        turn_factor = 1 - abs(turning) * 0.5  # Reduce speed when turning
        speed = base_speed * turn_factor
        
        # Clamp values
        speed = np.clip(speed, self.min_speed, self.max_speed)
        turning = np.clip(turning, -1, 1)
        
        return speed, turning
    
    def _stop_movement(self):
        """Stop all movement"""
        self.cv_ctrl.base_ctrl.base_json_ctrl({"T": 13, "X": 0, "Z": 0})
    
    def _create_overlay(self, frame, line_center, confidence, safety_status):
        """Create visual overlay for debugging"""
        overlay = frame.copy()
        
        # Draw line center
        if line_center:
            cx, cy = line_center
            cv2.circle(overlay, (cx, cy), 10, (0, 255, 0), -1)
            cv2.putText(overlay, f"Line: ({cx}, {cy})", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw confidence
        cv2.putText(overlay, f"Confidence: {confidence:.2f}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Draw safety status
        y_pos = 90
        for sensor, distance in safety_status['sensor_distances'].items():
            color = (0, 255, 0) if distance > 50 else (0, 0, 255)
            cv2.putText(overlay, f"{sensor}: {distance:.1f}cm", (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            y_pos += 20
        
        # Draw safety warnings
        if safety_status['emergency_stop']:
            cv2.putText(overlay, "EMERGENCY STOP", (200, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        elif safety_status['slow_mode']:
            cv2.putText(overlay, "SLOW MODE", (200, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        return overlay

class WaypointNavigator:
    """Simple waypoint navigation system"""
    
    def __init__(self, base_controller):
        self.base_ctrl = base_controller
        self.waypoints = []
        self.current_waypoint = 0
        self.navigation_active = False
        
        # Navigation parameters
        self.waypoint_reached_threshold = 50  # cm
        self.max_navigation_speed = 0.4
        
    def add_waypoint(self, x, y, action="move"):
        """Add a waypoint to the navigation list"""
        waypoint = {
            'x': x,
            'y': y,
            'action': action,
            'reached': False
        }
        self.waypoints.append(waypoint)
        print(f"Added waypoint: ({x}, {y}) - {action}")
    
    def start_navigation(self):
        """Start navigating to waypoints"""
        if not self.waypoints:
            print("No waypoints defined")
            return False
        
        self.current_waypoint = 0
        self.navigation_active = True
        print(f"Starting navigation to {len(self.waypoints)} waypoints")
        return True
    
    def stop_navigation(self):
        """Stop navigation"""
        self.navigation_active = False
        self.base_ctrl.base_json_ctrl({"T": 13, "X": 0, "Z": 0})
        print("Navigation stopped")
    
    def navigate_step(self, current_position):
        """Execute one step of navigation"""
        if not self.navigation_active or self.current_waypoint >= len(self.waypoints):
            return False, "Navigation complete"
        
        waypoint = self.waypoints[self.current_waypoint]
        
        # Calculate distance to waypoint
        distance = math.sqrt((current_position[0] - waypoint['x'])**2 + 
                           (current_position[1] - waypoint['y'])**2)
        
        if distance <= self.waypoint_reached_threshold:
            # Waypoint reached
            waypoint['reached'] = True
            self.current_waypoint += 1
            
            if waypoint['action'] == "stop":
                self.base_ctrl.base_json_ctrl({"T": 13, "X": 0, "Z": 0})
                time.sleep(2)  # Wait at waypoint
            
            if self.current_waypoint >= len(self.waypoints):
                self.navigation_active = False
                return False, "All waypoints reached"
            else:
                return True, f"Waypoint {self.current_waypoint} reached, moving to next"
        
        # Calculate direction to waypoint
        dx = waypoint['x'] - current_position[0]
        dy = waypoint['y'] - current_position[1]
        angle = math.atan2(dy, dx)
        
        # Convert to robot commands
        speed = min(self.max_navigation_speed, distance / 100)  # Speed based on distance
        turning = math.sin(angle) * 0.5  # Simple proportional control
        
        # Send movement command
        self.base_ctrl.base_json_ctrl({"T": 13, "X": speed, "Z": turning})
        
        return True, f"Moving to waypoint {self.current_waypoint + 1}"

class EnhancedRobotaxiController:
    """Main enhanced robotaxi controller"""
    
    def __init__(self):
        # Initialize base controller
        self.base_ctrl = BaseController('/dev/serial0', 115200)
        
        # Initialize computer vision
        self.cv_ctrl = cv_ctrl.OpencvFuncs(os.getcwd(), self.base_ctrl)
        
        # Initialize safety systems
        self.collision_prevention = CollisionPrevention(self.base_ctrl)
        
        # Initialize enhanced features
        self.line_follower = EnhancedLineFollower(self.cv_ctrl, self.collision_prevention)
        self.waypoint_navigator = WaypointNavigator(self.base_ctrl)
        
        # Control state
        self.current_mode = "manual"  # manual, line_following, waypoint_navigation
        self.running = True
        
        # Start control thread
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
    
    def _control_loop(self):
        """Main control loop"""
        while self.running:
            try:
                # Get current frame
                frame = self.cv_ctrl.frame_process()
                
                if self.current_mode == "line_following":
                    overlay, status = self.line_follower.follow_line(frame)
                    print(f"Line following: {status}")
                
                elif self.current_mode == "waypoint_navigation":
                    # For now, use simulated position
                    current_pos = [0, 0]  # Would come from odometry/SLAM
                    active, status = self.waypoint_navigator.navigate_step(current_pos)
                    if not active:
                        self.current_mode = "manual"
                    print(f"Navigation: {status}")
                
                time.sleep(0.1)  # 10Hz control loop
                
            except Exception as e:
                print(f"Control loop error: {e}")
                time.sleep(1)
    
    def set_mode(self, mode):
        """Set control mode"""
        if mode in ["manual", "line_following", "waypoint_navigation"]:
            self.current_mode = mode
            print(f"Mode changed to: {mode}")
            
            # Stop movement when changing modes
            self.base_ctrl.base_json_ctrl({"T": 13, "X": 0, "Z": 0})
        else:
            print(f"Invalid mode: {mode}")
    
    def manual_control(self, speed, turning):
        """Manual control with safety checks"""
        if self.current_mode != "manual":
            print("Not in manual mode")
            return
        
        is_safe, message = self.collision_prevention.is_safe_to_move(speed, turning)
        if is_safe:
            self.base_ctrl.base_json_ctrl({"T": 13, "X": speed, "Z": turning})
        else:
            print(f"Safety override: {message}")
    
    def get_status(self):
        """Get current system status"""
        return {
            'mode': self.current_mode,
            'safety': self.collision_prevention.get_safety_status(),
            'waypoints': len(self.waypoint_navigator.waypoints),
            'current_waypoint': self.waypoint_navigator.current_waypoint,
            'navigation_active': self.waypoint_navigator.navigation_active
        }
    
    def shutdown(self):
        """Shutdown the controller"""
        self.running = False
        self.base_ctrl.base_json_ctrl({"T": 13, "X": 0, "Z": 0})
        print("Robotaxi controller shutdown")

# Example usage and testing
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Enhanced Robotaxi Controller")
    parser.add_argument("--mode", choices=["manual", "line_following", "waypoint_navigation"], default="line_following", help="Start-up mode")
    args = parser.parse_args()

    print("Enhanced Robotaxi Controller")
    print("============================")
    
    try:
        # Initialize controller
        robotaxi = EnhancedRobotaxiController()
        robotaxi.set_mode(args.mode)
        
        if args.mode == "waypoint_navigation":
            # Example default square path (optional)
            robotaxi.waypoint_navigator.add_waypoint(100, 0, "move")
            robotaxi.waypoint_navigator.add_waypoint(100, 100, "stop")
            robotaxi.waypoint_navigator.add_waypoint(0, 100, "move")
            robotaxi.waypoint_navigator.add_waypoint(0, 0, "stop")
            robotaxi.waypoint_navigator.start_navigation()

        print(f"Robotaxi initialized in {args.mode} mode (Ctrl-C to quit)")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        robotaxi.shutdown()
    except Exception as e:
        print(f"Error: {e}") 