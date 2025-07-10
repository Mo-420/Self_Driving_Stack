#!/usr/bin/env python3
"""
Direct Motor Control for WaveShare Robot-Chassis MP
Controls motors directly via Raspberry Pi GPIO without ESP32
"""

import RPi.GPIO as GPIO
import time
import threading
import json
from typing import Dict, Any

class DirectMotorController:
    """Direct motor control using Raspberry Pi GPIO pins"""
    
    def __init__(self):
        # Motor control pins (adjust these based on your chassis wiring)
        self.LEFT_MOTOR_PIN1 = 17   # Left motor forward
        self.LEFT_MOTOR_PIN2 = 18   # Left motor backward
        self.RIGHT_MOTOR_PIN1 = 27  # Right motor forward
        self.RIGHT_MOTOR_PIN2 = 22  # Right motor backward
        
        # PWM pins for speed control
        self.LEFT_MOTOR_PWM = 23
        self.RIGHT_MOTOR_PWM = 24
        
        # PWM frequency
        self.PWM_FREQ = 1000
        
        # Initialize GPIO
        self.setup_gpio()
        
        # Motor state
        self.left_speed = 0
        self.right_speed = 0
        
        # Threading for smooth control
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        print("Direct Motor Controller initialized")
    
    def setup_gpio(self):
        """Setup GPIO pins for motor control"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup direction pins
        GPIO.setup(self.LEFT_MOTOR_PIN1, GPIO.OUT)
        GPIO.setup(self.LEFT_MOTOR_PIN2, GPIO.OUT)
        GPIO.setup(self.RIGHT_MOTOR_PIN1, GPIO.OUT)
        GPIO.setup(self.RIGHT_MOTOR_PIN2, GPIO.OUT)
        
        # Setup PWM pins
        GPIO.setup(self.LEFT_MOTOR_PWM, GPIO.OUT)
        GPIO.setup(self.RIGHT_MOTOR_PWM, GPIO.OUT)
        
        # Initialize PWM
        self.left_pwm = GPIO.PWM(self.LEFT_MOTOR_PWM, self.PWM_FREQ)
        self.right_pwm = GPIO.PWM(self.RIGHT_MOTOR_PWM, self.PWM_FREQ)
        
        # Start PWM
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
        # Stop motors initially
        self.stop()
    
    def set_motor_speed(self, left_speed: float, right_speed: float):
        """Set motor speeds (-1.0 to 1.0)"""
        self.left_speed = max(-1.0, min(1.0, left_speed))
        self.right_speed = max(-1.0, min(1.0, right_speed))
    
    def _control_loop(self):
        """Main control loop for smooth motor operation"""
        while True:
            # Control left motor
            if self.left_speed > 0:
                GPIO.output(self.LEFT_MOTOR_PIN1, GPIO.HIGH)
                GPIO.output(self.LEFT_MOTOR_PIN2, GPIO.LOW)
                self.left_pwm.ChangeDutyCycle(abs(self.left_speed) * 100)
            elif self.left_speed < 0:
                GPIO.output(self.LEFT_MOTOR_PIN1, GPIO.LOW)
                GPIO.output(self.LEFT_MOTOR_PIN2, GPIO.HIGH)
                self.left_pwm.ChangeDutyCycle(abs(self.left_speed) * 100)
            else:
                GPIO.output(self.LEFT_MOTOR_PIN1, GPIO.LOW)
                GPIO.output(self.LEFT_MOTOR_PIN2, GPIO.LOW)
                self.left_pwm.ChangeDutyCycle(0)
            
            # Control right motor
            if self.right_speed > 0:
                GPIO.output(self.RIGHT_MOTOR_PIN1, GPIO.HIGH)
                GPIO.output(self.RIGHT_MOTOR_PIN2, GPIO.LOW)
                self.right_pwm.ChangeDutyCycle(abs(self.right_speed) * 100)
            elif self.right_speed < 0:
                GPIO.output(self.RIGHT_MOTOR_PIN1, GPIO.LOW)
                GPIO.output(self.RIGHT_MOTOR_PIN2, GPIO.HIGH)
                self.right_pwm.ChangeDutyCycle(abs(self.right_speed) * 100)
            else:
                GPIO.output(self.RIGHT_MOTOR_PIN1, GPIO.LOW)
                GPIO.output(self.RIGHT_MOTOR_PIN2, GPIO.LOW)
                self.right_pwm.ChangeDutyCycle(0)
            
            time.sleep(0.01)  # 10ms control loop
    
    def stop(self):
        """Emergency stop - stop all motors"""
        self.set_motor_speed(0, 0)
        GPIO.output(self.LEFT_MOTOR_PIN1, GPIO.LOW)
        GPIO.output(self.LEFT_MOTOR_PIN2, GPIO.LOW)
        GPIO.output(self.RIGHT_MOTOR_PIN1, GPIO.LOW)
        GPIO.output(self.RIGHT_MOTOR_PIN2, GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)
    
    def cleanup(self):
        """Cleanup GPIO on shutdown"""
        self.stop()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()

# Compatibility layer for the existing Waveshare code
class DirectBaseController:
    """Drop-in replacement for BaseController that works without ESP32"""
    
    def __init__(self, uart_dev_set=None, buad_set=None):
        # Ignore UART parameters since we're not using them
        self.motor_controller = DirectMotorController()
        self.base_light_status = 0
        self.head_light_status = 0
        self.data_buffer = None
        self.base_data = None
        
        print("Direct Base Controller initialized (no ESP32 required)")
    
    def send_command(self, data: Dict[str, Any]):
        """Send motor commands (compatible with existing code)"""
        if isinstance(data, dict) and 'T' in data:
            if data['T'] == 1:  # Motor control command
                left_speed = data.get('L', 0)
                right_speed = data.get('R', 0)
                self.motor_controller.set_motor_speed(left_speed, right_speed)
                print(f"Motor command: L={left_speed}, R={right_speed}")
            elif data['T'] == 0:  # Emergency stop
                self.motor_controller.stop()
                print("Emergency stop")
    
    def base_speed_ctrl(self, input_left: float, input_right: float):
        """Direct speed control (compatible with existing code)"""
        self.motor_controller.set_motor_speed(input_left, input_right)
    
    def gimbal_emergency_stop(self):
        """Emergency stop (compatible with existing code)"""
        self.motor_controller.stop()
    
    def feedback_data(self):
        """Mock feedback data (compatible with existing code)"""
        # Return mock data to prevent errors
        return {
            "T": 1003,
            "L": self.motor_controller.left_speed,
            "R": self.motor_controller.right_speed,
            "r": 0, "p": 0, "v": 12,
            "pan": 0, "tilt": 0
        }
    
    def base_json_ctrl(self, input_json: Dict[str, Any]):
        """JSON control (compatible with existing code)"""
        self.send_command(input_json)
    
    def breath_light(self, input_time: int):
        """Mock breath light (compatible with existing code)"""
        # This would control LEDs if connected
        pass
    
    def base_oled(self, input_line: int, input_text: str):
        """Mock OLED display (compatible with existing code)"""
        print(f"OLED Line {input_line}: {input_text}")
    
    def base_default_oled(self):
        """Mock default OLED (compatible with existing code)"""
        pass
    
    def lights_ctrl(self, pwmA: int, pwmB: int):
        """Mock light control (compatible with existing code)"""
        self.base_light_status = pwmA
        self.head_light_status = pwmB
    
    def base_lights_ctrl(self):
        """Mock base light control (compatible with existing code)"""
        if self.base_light_status != 0:
            self.base_light_status = 0
        else:
            self.base_light_status = 255
        self.lights_ctrl(self.base_light_status, self.head_light_status)
    
    def gimbal_dev_close(self):
        """Cleanup on close"""
        self.motor_controller.cleanup()
    
    # Mock methods for compatibility with existing code
    def bus_servo_torque_lock(self, servo_id, status):
        """Mock servo torque lock (compatible with existing code)"""
        print(f"Mock servo torque lock: ID={servo_id}, Status={status}")
    
    def bus_servo_id_set(self, old_id, new_id):
        """Mock servo ID set (compatible with existing code)"""
        print(f"Mock servo ID set: Old={old_id}, New={new_id}")
    
    def bus_servo_mid_set(self, servo_id):
        """Mock servo mid set (compatible with existing code)"""
        print(f"Mock servo mid set: ID={servo_id}")

# Test function
if __name__ == "__main__":
    try:
        # Test the direct motor controller
        controller = DirectBaseController()
        
        print("Testing motor control...")
        print("Forward for 2 seconds...")
        controller.base_speed_ctrl(0.5, 0.5)
        time.sleep(2)
        
        print("Stop...")
        controller.base_speed_ctrl(0, 0)
        time.sleep(1)
        
        print("Turn left for 2 seconds...")
        controller.base_speed_ctrl(-0.3, 0.3)
        time.sleep(2)
        
        print("Stop...")
        controller.base_speed_ctrl(0, 0)
        time.sleep(1)
        
        print("Backward for 2 seconds...")
        controller.base_speed_ctrl(-0.5, -0.5)
        time.sleep(2)
        
        print("Emergency stop...")
        controller.gimbal_emergency_stop()
        
        print("Test completed successfully!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        if 'controller' in locals():
            controller.gimbal_dev_close() 