#!/usr/bin/env python3
"""
Test script for WaveShare Robot-Chassis MP hardware
Tests basic motor control and feedback
"""

import sys
import time
import json

# Add ugv_rpi to path
sys.path.append('/home/pi/WaveShare/ugv_rpi')

try:
    from base_ctrl import BaseController
except ImportError:
    print("Error: Could not import base_ctrl. Make sure you're running this on the Pi with ugv_rpi installed.")
    sys.exit(1)

def test_waveshare_hardware():
    """Test basic WaveShare hardware functions"""
    
    print("WaveShare Hardware Test")
    print("=" * 50)
    
    # Initialize base controller
    try:
        # Use /dev/serial0 for Pi 4, /dev/ttyAMA0 for Pi 5
        base = BaseController('/dev/serial0', 115200)
        print("✓ Connected to WaveShare base controller")
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        return
    
    # Test 1: Emergency stop
    print("\nTest 1: Emergency Stop")
    base.gimbal_emergency_stop()
    print("✓ Emergency stop sent")
    time.sleep(1)
    
    # Test 2: Forward motion
    print("\nTest 2: Forward Motion (2 seconds)")
    base.base_speed_ctrl(0.2, 0.2)  # 0.2 m/s forward
    time.sleep(2)
    base.base_speed_ctrl(0, 0)  # Stop
    print("✓ Forward motion complete")
    time.sleep(1)
    
    # Test 3: Turn in place
    print("\nTest 3: Turn Right (2 seconds)")
    base.base_speed_ctrl(0.2, -0.2)  # Turn right
    time.sleep(2)
    base.base_speed_ctrl(0, 0)  # Stop
    print("✓ Turn complete")
    time.sleep(1)
    
    # Test 4: ROS-style control
    print("\nTest 4: ROS-style Control")
    cmd = {"T": 13, "X": 0.5, "Z": 0}  # Forward at 50% speed
    base.base_json_ctrl(cmd)
    time.sleep(2)
    cmd = {"T": 13, "X": 0, "Z": 0}  # Stop
    base.base_json_ctrl(cmd)
    print("✓ ROS control complete")
    time.sleep(1)
    
    # Test 5: Get feedback
    print("\nTest 5: Get Feedback Data")
    try:
        feedback = base.feedback_data()
        if feedback:
            print(f"✓ Feedback received: {json.dumps(feedback, indent=2)}")
        else:
            print("⚠ No feedback received")
    except Exception as e:
        print(f"⚠ Feedback error: {e}")
    
    # Test 6: OLED display
    print("\nTest 6: OLED Display")
    base.base_oled(0, "WaveShare Test")
    base.base_oled(1, "Motor Test OK")
    print("✓ OLED updated")
    time.sleep(2)
    
    # Test 7: Lights
    print("\nTest 7: LED Lights")
    base.lights_ctrl(255, 255)  # Full brightness
    print("✓ Lights ON")
    time.sleep(2)
    base.lights_ctrl(0, 0)  # Off
    print("✓ Lights OFF")
    
    # Final stop
    base.gimbal_emergency_stop()
    print("\n" + "=" * 50)
    print("Hardware test complete!")
    print("\nIf all tests passed, your WaveShare hardware is working correctly.")
    print("You can now run the ROS 2 nodes with confidence.")

if __name__ == "__main__":
    try:
        test_waveshare_hardware()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        # Emergency stop on interrupt
        try:
            base = BaseController('/dev/serial0', 115200)
            base.gimbal_emergency_stop()
        except:
            pass 