#!/usr/bin/env python3
"""
Simple test environment for robotaxi nodes without full ROS 2 stack
This allows testing the core logic of our nodes on Mac
"""

import sys
import os
import time
import threading
from typing import Dict, Any

# Mock ROS 2 classes for testing
class MockNode:
    def __init__(self, name):
        self.name = name
        self.publishers = {}
        self.subscribers = {}
        self.timers = {}
        print(f"Created mock node: {name}")
    
    def create_publisher(self, msg_type, topic, qos=10):
        print(f"Creating publisher: {topic}")
        self.publishers[topic] = MockPublisher(topic)
        return self.publishers[topic]
    
    def create_subscription(self, msg_type, topic, callback, qos=10):
        print(f"Creating subscription: {topic}")
        self.subscribers[topic] = MockSubscription(topic, callback)
        return self.subscribers[topic]
    
    def create_timer(self, period, callback):
        print(f"Creating timer with period: {period}")
        timer = MockTimer(period, callback)
        self.timers[id(timer)] = timer
        return timer
    
    def get_logger(self):
        return MockLogger()
    
    def destroy_node(self):
        print(f"Destroying node: {self.name}")

class MockPublisher:
    def __init__(self, topic):
        self.topic = topic
        self.published_messages = []
    
    def publish(self, msg):
        self.published_messages.append(msg)
        print(f"Published to {self.topic}: {msg}")

class MockSubscription:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback
        self.received_messages = []
    
    def receive_message(self, msg):
        self.received_messages.append(msg)
        self.callback(msg)

class MockTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.running = False
        self.thread = None
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()
    
    def _run(self):
        while self.running:
            self.callback()
            time.sleep(self.period)
    
    def cancel(self):
        self.running = False

class MockLogger:
    def info(self, msg):
        print(f"[INFO] {msg}")
    
    def warn(self, msg):
        print(f"[WARN] {msg}")
    
    def error(self, msg):
        print(f"[ERROR] {msg}")
    
    def debug(self, msg):
        print(f"[DEBUG] {msg}")

# Mock message types
class MockMsg:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)
    
    def __str__(self):
        attrs = [f"{k}={v}" for k, v in self.__dict__.items()]
        return f"{self.__class__.__name__}({', '.join(attrs)})"

class Twist(MockMsg):
    pass

class TwistStamped(MockMsg):
    pass

class PoseStamped(MockMsg):
    pass

class Odometry(MockMsg):
    pass

class LaserScan(MockMsg):
    pass

class Image(MockMsg):
    pass

class Bool(MockMsg):
    pass

class Float32(MockMsg):
    pass

class Int32(MockMsg):
    pass

class String(MockMsg):
    pass

# Mock QoS
class QoSProfile:
    def __init__(self, depth=10):
        self.depth = depth

# Mock rclpy module
class MockRclpy:
    @staticmethod
    def init():
        print("Mock rclpy.init() called")
    
    @staticmethod
    def shutdown():
        print("Mock rclpy.shutdown() called")
    
    @staticmethod
    def spin(node):
        print(f"Mock rclpy.spin() called for node: {node.name}")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Mock rclpy.spin() interrupted")

# Replace the real modules with mocks
sys.modules['rclpy'] = MockRclpy()
sys.modules['rclpy.node'] = type('MockNodeModule', (), {'Node': MockNode})()
sys.modules['rclpy.qos'] = type('MockQoSModule', (), {'QoSProfile': QoSProfile})()

# Mock message modules
geometry_msgs = type('MockGeometryMsgs', (), {
    'msg': type('MockGeometryMsgsMsg', (), {
        'Twist': Twist,
        'TwistStamped': TwistStamped,
        'PoseStamped': PoseStamped
    })()
})()

nav_msgs = type('MockNavMsgs', (), {
    'msg': type('MockNavMsgsMsg', (), {
        'Odometry': Odometry
    })()
})()

sensor_msgs = type('MockSensorMsgs', (), {
    'msg': type('MockSensorMsgsMsg', (), {
        'LaserScan': LaserScan,
        'Image': Image
    })()
})()

std_msgs = type('MockStdMsgs', (), {
    'msg': type('MockStdMsgsMsg', (), {
        'Bool': Bool,
        'Float32': Float32,
        'Int32': Int32,
        'String': String
    })()
})()

# Replace the message modules
sys.modules['geometry_msgs'] = geometry_msgs
sys.modules['nav_msgs'] = nav_msgs
sys.modules['sensor_msgs'] = sensor_msgs
sys.modules['std_msgs'] = std_msgs

def test_node(node_class, node_name="test_node"):
    """Test a node class with mock ROS 2 environment"""
    print(f"\n{'='*50}")
    print(f"Testing {node_class.__name__}")
    print(f"{'='*50}")
    
    try:
        # Create and run the node
        node = node_class()
        node.start()
        
        # Let it run for a few seconds
        time.sleep(5)
        
        # Stop the node
        node.stop()
        print(f"Test completed for {node_class.__name__}")
        
    except Exception as e:
        print(f"Error testing {node_class.__name__}: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("Robotaxi Test Environment")
    print("This allows testing nodes without full ROS 2 stack")
    
    # Add the src directory to Python path
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
    
    # Test each node
    try:
        from waveshare_base.waveshare_base.base_node import BaseNode
        test_node(BaseNode, "base_node")
    except ImportError as e:
        print(f"Could not import BaseNode: {e}")
    
    try:
        from waveshare_safety.waveshare_safety.ultrasonic_safety_node import UltrasonicSafetyNode
        test_node(UltrasonicSafetyNode, "ultrasonic_safety_node")
    except ImportError as e:
        print(f"Could not import UltrasonicSafetyNode: {e}")
    
    try:
        from waveshare_navigation.waveshare_navigation.wander_node import WanderNode
        test_node(WanderNode, "wander_node")
    except ImportError as e:
        print(f"Could not import WanderNode: {e}")
    
    print("\nTest environment completed!") 