#!/usr/bin/env python3
"""
Simple test script for robotaxi nodes
Tests the core logic without full ROS 2 dependencies
"""

import sys
import os
import time
import threading
import math
import types

# Add the project root to PYTHONPATH for ugv_rpi check
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Add the src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

# Mock ROS 2 classes
class MockTime:
    def __init__(self):
        self._start = time.time()
    @property
    def nanoseconds(self):
        return int((time.time() - self._start) * 1e9)
    def to_msg(self):
        return None

class MockClock:
    def now(self):
        return MockTime()

class MockParameter:
    def __init__(self, value):
        self.value = value

class MockNode:
    def __init__(self, name):
        self.name = name
        self.publishers = {}
        self.subscribers = {}
        self.timers = {}
        self.params = {}
        print(f"Created node: {name}")
    
    def declare_parameter(self, name, default):
        self.params[name] = default
        return MockParameter(default)
    
    def get_parameter(self, name):
        return MockParameter(self.params.get(name))
    
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
        # Not starting auto to avoid infinite loops during test
        return timer
    
    def get_logger(self):
        return MockLogger()
    
    def get_clock(self):
        return MockClock()
    
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

class Imu(MockMsg):
    pass

# Mock QoS
class QoSProfile:
    def __init__(self, *, history=None, depth=10, reliability=None):
        self.history = history
        self.depth = depth
        self.reliability = reliability

# Create mock modules
class MockRclpy:
    @staticmethod
    def init():
        print("rclpy.init() called")
    
    @staticmethod
    def shutdown():
        print("rclpy.shutdown() called")
    
    @staticmethod
    def spin(node):
        print(f"rclpy.spin() called for node: {node.name}")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("rclpy.spin() interrupted")

# Create mock message modules
class MockGeometryMsgs:
    class msg:
        Twist = Twist
        TwistStamped = TwistStamped
        PoseStamped = PoseStamped

class MockNavMsgs:
    class msg:
        Odometry = Odometry

class MockSensorMsgs:
    class msg:
        LaserScan = LaserScan
        Image = Image
        Imu = Imu

class MockStdMsgs:
    class msg:
        Bool = Bool
        Float32 = Float32
        Int32 = Int32
        String = String

# Replace modules
geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
geometry_msgs_msg.Twist = Twist
geometry_msgs_msg.TwistStamped = TwistStamped
geometry_msgs_msg.PoseStamped = PoseStamped
geometry_msgs = types.ModuleType('geometry_msgs')
geometry_msgs.msg = geometry_msgs_msg

nav_msgs_msg = types.ModuleType('nav_msgs.msg')
nav_msgs_msg.Odometry = Odometry
nav_msgs = types.ModuleType('nav_msgs')
nav_msgs.msg = nav_msgs_msg

sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
sensor_msgs_msg.LaserScan = LaserScan
sensor_msgs_msg.Image = Image
sensor_msgs_msg.Imu = Imu
sensor_msgs = types.ModuleType('sensor_msgs')
sensor_msgs.msg = sensor_msgs_msg

std_msgs_msg = types.ModuleType('std_msgs.msg')
std_msgs_msg.Bool = Bool
std_msgs_msg.Float32 = Float32
std_msgs_msg.Int32 = Int32
std_msgs_msg.String = String
std_msgs = types.ModuleType('std_msgs')
std_msgs.msg = std_msgs_msg

# Extend rclpy.qos mock
class MockQoSModule:
    QoSProfile = QoSProfile
    QoSReliabilityPolicy = type('QoSReliabilityPolicy', (), {'RELIABLE': 1, 'BEST_EFFORT': 2})
    QoSHistoryPolicy = type('QoSHistoryPolicy', (), {'KEEP_LAST': 1, 'KEEP_ALL': 2})

# Mock robotaxi_enhanced.UltrasonicSensor
robotaxi_enhanced = types.ModuleType('robotaxi_enhanced')
class UltrasonicSensor:
    def __init__(self, *a, **kw):
        print("Mock UltrasonicSensor created")
    def get_distance(self):
        return 100.0
robotaxi_enhanced.UltrasonicSensor = UltrasonicSensor

# Mock ugv_rpi.base_ctrl.BaseController so base_node can import
ugv_rpi = types.ModuleType('ugv_rpi')
base_ctrl_mod = types.ModuleType('ugv_rpi.base_ctrl')
class BaseController:
    def __init__(self, *a, **kw):
        print("Mock BaseController created with args", a, kw)
    def base_json_ctrl(self, cmd):
        print("Mock base_json_ctrl", cmd)
base_ctrl_mod.BaseController = BaseController
ugv_rpi.base_ctrl = base_ctrl_mod

# Register all mock modules and submodules in sys.modules immediately
sys.modules['rclpy'] = MockRclpy()
sys.modules['rclpy.node'] = type('MockNodeModule', (), {'Node': MockNode})()
sys.modules['rclpy.qos'] = MockQoSModule()
sys.modules['geometry_msgs'] = geometry_msgs
sys.modules['geometry_msgs.msg'] = geometry_msgs_msg
sys.modules['nav_msgs'] = nav_msgs
sys.modules['nav_msgs.msg'] = nav_msgs_msg
sys.modules['sensor_msgs'] = sensor_msgs
sys.modules['sensor_msgs.msg'] = sensor_msgs_msg
sys.modules['std_msgs'] = std_msgs
sys.modules['std_msgs.msg'] = std_msgs_msg
sys.modules['robotaxi_enhanced'] = robotaxi_enhanced
sys.modules['ugv_rpi'] = ugv_rpi
sys.modules['ugv_rpi.base_ctrl'] = base_ctrl_mod

def test_wander_node():
    """Test the wander node logic"""
    print("\n" + "="*50)
    print("Testing Wander Node")
    print("="*50)
    
    # Debug: print attributes of sensor_msgs.msg
    import sensor_msgs.msg
    print("sensor_msgs.msg attributes:", dir(sensor_msgs.msg))
    
    try:
        # Import and test wander node
        from waveshare_navigation.waveshare_navigation.wander_node import WanderNode
        
        # Create node
        node = WanderNode()
        print("Wander node created successfully")
        
        # Test basic functionality
        print("Testing wander behavior...")
        
        # Simulate some time passing
        time.sleep(2)
        
        print("Wander node test completed")
        
    except Exception as e:
        print(f"Error testing wander node: {e}")
        import traceback
        traceback.print_exc()

def test_base_node():
    """Test the base node logic"""
    print("\n" + "="*50)
    print("Testing Base Node")
    print("="*50)
    
    try:
        # Import and test base node
        from waveshare_base.waveshare_base.base_node import WaveshareBaseNode
        
        # Create node
        node = WaveshareBaseNode()
        print("Base node created successfully")
        
        # Test basic functionality
        print("Testing base control...")
        
        # Simulate some time passing
        time.sleep(2)
        
        print("Base node test completed")
        
    except Exception as e:
        print(f"Error testing base node: {e}")
        import traceback
        traceback.print_exc()

def test_safety_node():
    """Test the safety node logic"""
    print("\n" + "="*50)
    print("Testing Safety Node")
    print("="*50)
    
    try:
        # Import and test safety node
        from waveshare_safety.waveshare_safety.ultrasonic_safety_node import CollisionPrevention as UltrasonicSafetyNode
        
        # Create node
        node = UltrasonicSafetyNode()
        print("Safety node created successfully")
        
        # Test basic functionality
        print("Testing safety behavior...")
        
        # Simulate some time passing
        time.sleep(2)
        
        print("Safety node test completed")
        
    except Exception as e:
        print(f"Error testing safety node: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("Robotaxi Node Test Environment")
    print("Testing core logic without full ROS 2 stack")
    
    # Test each node
    test_wander_node()
    test_base_node()
    test_safety_node()
    
    print("\nAll tests completed!") 