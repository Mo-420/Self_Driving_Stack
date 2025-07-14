#!/usr/bin/env python3
"""
Simplified WaveShare Robotaxi Controller
Basic autonomous driving without complex CV dependencies
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

class SimpleLineFollower:
    """Simple line following using basic OpenCV"""
    
    def __init__(self, base_controller):
        self.base_ctrl = base_controller
        
        # Line following parameters
        self.base_speed = 0.3
        self.max_speed = 0.5
        self.min_speed = 0.1
        self.turn_sensitivity = 1.5
        
        # Camera setup
        self.camera = None
        self.frame_width = 640
        self.frame_height = 480
        
        # Line detection parameters
        self.roi_height = 0.6  # Region of interest (bottom 60% of frame)
        self.line_threshold = 50  # Brightness threshold for line detection
        
        # Control state
        self.is_running = False
        self.last_line_center = None
        self.line_lost_count = 0
        self.max_line_lost = 30  # Frames to wait before stopping
        
        print("Simple line follower initialized")
    
    def start_camera(self):
        """Initialize camera"""
        try:
            self.camera = cv2.VideoCapture(0)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            
            if not self.camera.isOpened():
                print("Warning: Could not open camera, running in simulation mode")
                self.camera = None
            else:
                print("Camera initialized successfully")
                
        except Exception as e:
            print(f"Camera initialization error: {e}")
            self.camera = None
    
    def detect_line(self, frame):
        """Detect line in the frame using simple thresholding"""
        if frame is None:
            return None, 0.0
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Define region of interest (bottom portion)
        roi_y = int(self.frame_height * (1 - self.roi_height))
        roi = gray[roi_y:, :]
        
        # Apply threshold to find dark line
        _, binary = cv2.threshold(roi, self.line_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest contour (assumed to be the line)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Calculate center of the contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Convert to frame coordinates
                line_center = cx
                confidence = min(1.0, cv2.contourArea(largest_contour) / 1000)
                
                return line_center, confidence
        
        return None, 0.0
    
    def calculate_movement(self, line_center, confidence):
        """Calculate motor commands based on line position"""
        if line_center is None or confidence < 0.1:
            # Line lost - slow down and search
            speed = self.min_speed
            turning = 0.0
            self.line_lost_count += 1
        else:
            # Line found - calculate steering
            self.line_lost_count = 0
            
            # Calculate error from center
            center_error = (line_center - self.frame_width // 2) / (self.frame_width // 2)
            
            # Calculate speed based on confidence and error
            speed = self.base_speed * confidence
            speed = max(self.min_speed, min(self.max_speed, speed))
            
            # Calculate turning based on error
            turning = -center_error * self.turn_sensitivity
            turning = max(-1.0, min(1.0, turning))
        
        return speed, turning
    
    def follow_line(self):
        """Main line following loop"""
        self.is_running = True
        print("Starting line following...")
        
        while self.is_running:
            try:
                # Get frame from camera
                if self.camera and self.camera.isOpened():
                    ret, frame = self.camera.read()
                    if not ret:
                        frame = None
                else:
                    frame = None
                
                # Detect line
                line_center, confidence = self.detect_line(frame)
                
                # Calculate movement
                speed, turning = self.calculate_movement(line_center, confidence)
                
                # Apply movement
                if self.line_lost_count < self.max_line_lost:
                    # Convert to motor commands
                    left_speed = speed - turning * speed
                    right_speed = speed + turning * speed
                    
                    # Send to base controller
                    self.base_ctrl.base_speed_ctrl(left_speed, right_speed)
                else:
                    # Stop if line lost for too long
                    self.base_ctrl.base_speed_ctrl(0, 0)
                    print("Line lost - stopping")
                
                # Display info
                if frame is not None:
                    self.display_info(frame, line_center, confidence, speed, turning)
                
                time.sleep(0.1)  # 10Hz control loop
                
            except Exception as e:
                print(f"Line following error: {e}")
                time.sleep(0.1)
    
    def display_info(self, frame, line_center, confidence, speed, turning):
        """Display information on frame"""
        # Draw line center if detected
        if line_center is not None:
            cv2.circle(frame, (line_center, int(self.frame_height * 0.8)), 10, (0, 255, 0), -1)
        
        # Draw center line
        cv2.line(frame, (self.frame_width // 2, 0), (self.frame_width // 2, self.frame_height), (255, 0, 0), 2)
        
        # Add text info
        cv2.putText(frame, f"Confidence: {confidence:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Speed: {speed:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Turning: {turning:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Show frame
        cv2.imshow('Line Following', frame)
        cv2.waitKey(1)
    
    def stop(self):
        """Stop line following"""
        self.is_running = False
        self.base_ctrl.base_speed_ctrl(0, 0)
        if self.camera:
            self.camera.release()
        cv2.destroyAllWindows()
        print("Line following stopped")

class SimpleRobotaxiController:
    """Simple robotaxi controller with basic autonomous features"""
    
    def __init__(self):
        # Initialize base controller
        try:
            self.base_ctrl = BaseController('/dev/serial0', 115200)
            print("✓ Connected to WaveShare base controller")
        except Exception as e:
            print(f"✗ Failed to connect to base controller: {e}")
            print("Running in simulation mode")
            self.base_ctrl = None
        
        # Initialize components
        self.line_follower = SimpleLineFollower(self.base_ctrl)
        
        # Control state
        self.mode = "idle"  # idle, line_following, manual
        self.is_running = False
        
        # Control thread
        self.control_thread = None
        
        print("Simple Robotaxi Controller initialized")
    
    def start(self):
        """Start the controller"""
        if self.is_running:
            print("Controller already running")
            return
        
        self.is_running = True
        
        # Start camera
        self.line_follower.start_camera()
        
        # Start control thread
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        print("Robotaxi controller started")
    
    def _control_loop(self):
        """Main control loop"""
        while self.is_running:
            try:
                if self.mode == "line_following":
                    self.line_follower.follow_line()
                elif self.mode == "manual":
                    # Manual mode - do nothing in control loop
                    time.sleep(0.1)
                else:
                    # Idle mode
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"Control loop error: {e}")
                time.sleep(0.1)
    
    def set_mode(self, mode):
        """Set operation mode"""
        if mode not in ["idle", "line_following", "manual"]:
            print(f"Invalid mode: {mode}")
            return
        
        print(f"Setting mode to: {mode}")
        self.mode = mode
        
        if mode == "idle":
            self.stop_movement()
        elif mode == "line_following":
            # Line following will start in the control loop
            pass
    
    def manual_control(self, speed, turning):
        """Manual control commands"""
        if self.mode != "manual":
            print("Not in manual mode")
            return
        
        if self.base_ctrl:
            left_speed = speed - turning * speed
            right_speed = speed + turning * speed
            self.base_ctrl.base_speed_ctrl(left_speed, right_speed)
    
    def stop_movement(self):
        """Stop all movement"""
        if self.base_ctrl:
            self.base_ctrl.base_speed_ctrl(0, 0)
        print("Movement stopped")
    
    def emergency_stop(self):
        """Emergency stop"""
        if self.base_ctrl:
            self.base_ctrl.gimbal_emergency_stop()
        print("EMERGENCY STOP")
    
    def get_status(self):
        """Get current status"""
        return {
            'mode': self.mode,
            'running': self.is_running,
            'hardware_connected': self.base_ctrl is not None
        }
    
    def shutdown(self):
        """Shutdown the controller"""
        print("Shutting down robotaxi controller...")
        self.is_running = False
        self.stop_movement()
        self.line_follower.stop()
        
        if self.control_thread:
            self.control_thread.join(timeout=2)
        
        print("Robotaxi controller shutdown complete")

def main():
    """Main function"""
    print("=== WaveShare Simple Robotaxi Controller ===")
    
    # Create controller
    controller = SimpleRobotaxiController()
    
    try:
        # Start controller
        controller.start()
        
        # Simple demo - start line following
        print("Starting line following demo...")
        controller.set_mode("line_following")
        
        # Run for 60 seconds
        time.sleep(60)
        
        # Stop
        controller.set_mode("idle")
        print("Demo complete")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main() 