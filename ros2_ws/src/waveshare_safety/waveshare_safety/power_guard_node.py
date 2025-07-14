#!/usr/bin/env python3
"""power_guard_node.py

Monitors battery voltage and an optional hardware E-stop GPIO topic.  If a
fault is detected it:
  * Publishes latched std_msgs/Bool `True` on /estop
  * Publishes zero geometry_msgs/Twist on /cmd_vel_safe every cycle so motors
    remain disabled even if upstream nodes misbehave.
  * Logs the fault reason.

Parameters
----------
low_voltage_threshold : float (default 6.4)  # volts for 2-S Li-ion
estop_topic           : str   default '/estop_switch' (std_msgs/Bool input)
cmd_out_topic         : str   default '/cmd_vel_safe'

Note: actual motor driver disable (MD13S EN pin) should be wired through a
hardware interlock driven by the /estop Boolean.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

class PowerGuard(Node):
    def __init__(self):
        super().__init__('power_guard')

        # ---------------- Parameters ---------------------------
        self.declare_parameter('low_voltage_threshold', 6.4)  # V
        self.declare_parameter('estop_topic', '/estop_switch')
        self.declare_parameter('cmd_out_topic', '/cmd_vel_safe')

        self.low_v = float(self.get_parameter('low_voltage_threshold').value)
        estop_in = self.get_parameter('estop_topic').get_parameter_value().string_value
        cmd_out = self.get_parameter('cmd_out_topic').get_parameter_value().string_value

        # ---------------- State --------------------------------
        self._estop = False
        self._last_voltage = 0.0

        # ---------------- Pub/Sub ------------------------------
        self.estop_pub = self.create_publisher(Bool, '/estop', 1)
        self.cmd_pub = self.create_publisher(Twist, cmd_out, 10)

        self.create_subscription(Float32, '/battery_voltage', self._cb_batt, 10)
        self.create_subscription(Bool, estop_in, self._cb_switch, 10)

        # 10 Hz loop to republish stop twist when estop asserted
        self.create_timer(0.1, self._tick)
        self.get_logger().info('PowerGuard online – watching battery & E-stop')

    # ----------------------------------------------------------
    def _cb_batt(self, msg: Float32):
        self._last_voltage = msg.data
        if not self._estop and msg.data <= self.low_v:
            self.get_logger().error(f'Battery low ( {msg.data:.2f} V ) – triggering E-STOP')
            self._trigger_estop()

    def _cb_switch(self, msg: Bool):
        if msg.data and not self._estop:
            self.get_logger().warning('Hardware E-stop asserted')
            self._trigger_estop()

    # ----------------------------------------------------------
    def _trigger_estop(self):
        self._estop = True
        self.estop_pub.publish(Bool(data=True))
        # immediate stop
        self.cmd_pub.publish(Twist())

    # ----------------------------------------------------------
    def _tick(self):
        if self._estop:
            # Keep sending zero command so motors stay off
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = PowerGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 