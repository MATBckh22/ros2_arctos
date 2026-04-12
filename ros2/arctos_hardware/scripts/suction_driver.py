#!/usr/bin/env python3
"""
Arctos Suction Gripper Driver — ROS 2 node.

Communicates with the Arduino Nano suction gripper over CAN bus (ID 7),
exposing ROS services and topics for suction control.

Interfaces:
  /suction/activate    (std_srvs/SetBool)    — True=on, False=off
  /suction/set_pump    (std_srvs/SetBool)    — pump on/off at default PWM
  /suction/command     (std_msgs/UInt8MultiArray) — [pump_pwm, valve_state]
  /suction/status      (std_msgs/UInt8MultiArray) — [pump_pwm, valve, faults, uptime_hi, uptime_lo]

Usage:
    ros2 run arctos_hardware suction_driver.py
"""

import time

import can
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import UInt8MultiArray

from arctos_hardware.suction_protocol import (
    SUCTION_CAN_ID,
    CMD_SET_SUCTION,
    CMD_SET_PUMP_PWM,
    CMD_SET_VALVE,
    CMD_QUERY_STATUS,
    CMD_SET_WATCHDOG,
    RSP_STATUS,
    RSP_FAULT_EVENT,
    build_frame,
    validate_crc,
)


class SuctionDriver(Node):
    def __init__(self):
        super().__init__('suction_driver')

        self.declare_parameter('can_device', 'can0')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('default_pump_pwm', 200)
        self.declare_parameter('status_poll_rate', 2.0)
        self.declare_parameter('watchdog_timeout_100ms', 10)

        can_device = self.get_parameter('can_device').value
        can_bitrate = self.get_parameter('can_bitrate').value
        self._default_pwm = self.get_parameter('default_pump_pwm').value
        status_rate = self.get_parameter('status_poll_rate').value
        watchdog_val = self.get_parameter('watchdog_timeout_100ms').value

        self._bus = None
        self._pump_pwm = 0
        self._valve_state = 0
        self._fault_bits = 0

        try:
            self._bus = can.Bus(interface='socketcan', channel=can_device)
            self.get_logger().info(
                f'Suction driver connected to {can_device} (CAN ID {SUCTION_CAN_ID})'
            )
        except Exception as e:
            self.get_logger().fatal(f'Failed to open CAN bus: {e}')
            raise RuntimeError(f'CAN connection failed: {e}')

        # Set watchdog on Arduino
        self._send_command(CMD_SET_WATCHDOG, [watchdog_val])

        # Services
        self.create_service(SetBool, '/suction/activate', self._handle_activate)
        self.create_service(SetBool, '/suction/set_pump', self._handle_set_pump)

        # Command topic for fine-grained control: [pump_pwm, valve_state]
        self.create_subscription(
            UInt8MultiArray,
            '/suction/command',
            self._on_command,
            10,
        )

        # Status publisher
        self._status_pub = self.create_publisher(UInt8MultiArray, '/suction/status', 10)

        # Poll status periodically (also catches fault events)
        if status_rate > 0:
            self.create_timer(1.0 / status_rate, self._poll_status)

        self.get_logger().info('Suction driver ready')

    def _send_command(self, opcode: int, data: list[int] | None = None) -> bytes | None:
        """Send a command to the suction gripper and wait for ACK."""
        frame = build_frame(SUCTION_CAN_ID, opcode, data)

        # Flush stale messages
        while True:
            stale = self._bus.recv(timeout=0.01)
            if stale is None:
                break

        msg = can.Message(
            arbitration_id=SUCTION_CAN_ID,
            data=frame,
            is_extended_id=False,
        )
        self._bus.send(msg)

        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            resp = self._bus.recv(timeout=0.1)
            if resp is None:
                continue
            if resp.arbitration_id != SUCTION_CAN_ID:
                continue
            if not validate_crc(SUCTION_CAN_ID, resp.data):
                self.get_logger().warn('Suction: bad CRC in response')
                continue
            # Handle unsolicited fault events inline
            if len(resp.data) >= 3 and resp.data[0] == RSP_FAULT_EVENT:
                self._fault_bits = resp.data[1]
                self.get_logger().error(
                    f'Suction FAULT event: 0x{self._fault_bits:02X}'
                )
                continue
            return bytes(resp.data)

        self.get_logger().warn('Suction: no response from CAN ID 7')
        return None

    def _set_suction(self, pump_pwm: int, valve: int) -> bool:
        pump_pwm = max(0, min(255, pump_pwm))
        valve = 1 if valve else 0

        resp = self._send_command(CMD_SET_SUCTION, [pump_pwm, valve])
        if resp is not None:
            self._pump_pwm = pump_pwm
            self._valve_state = valve
            return True
        return False

    def _handle_activate(self, request, response):
        """SetBool service: True = suction on (pump on, valve closed),
        False = release (pump off, valve open briefly then closed)."""
        if request.data:
            ok = self._set_suction(self._default_pwm, 0)
            response.message = f'Suction ON (pump={self._default_pwm})' if ok else 'Failed'
        else:
            # Open valve to break vacuum, then close
            self._set_suction(0, 1)
            time.sleep(0.5)
            ok = self._set_suction(0, 0)
            response.message = 'Suction OFF (released)' if ok else 'Failed'
        response.success = ok
        return response

    def _handle_set_pump(self, request, response):
        """SetBool service for pump only (valve unchanged)."""
        pwm = self._default_pwm if request.data else 0
        resp = self._send_command(CMD_SET_PUMP_PWM, [pwm])
        ok = resp is not None
        if ok:
            self._pump_pwm = pwm
        response.success = ok
        response.message = f'Pump {"ON" if request.data else "OFF"}' if ok else 'Failed'
        return response

    def _on_command(self, msg: UInt8MultiArray):
        """Topic callback for fine-grained control: data=[pump_pwm, valve_state]."""
        if len(msg.data) >= 2:
            self._set_suction(msg.data[0], msg.data[1])

    def _poll_status(self):
        """Periodically query gripper status."""
        resp = self._send_command(CMD_QUERY_STATUS)
        if resp and len(resp) >= 7 and resp[0] == RSP_STATUS:
            self._pump_pwm = resp[1]
            self._valve_state = resp[2]
            self._fault_bits = resp[3]

            status_msg = UInt8MultiArray()
            status_msg.data = list(resp[1:6])
            self._status_pub.publish(status_msg)

            if self._fault_bits:
                self.get_logger().warn(f'Suction faults: 0x{self._fault_bits:02X}')

    def destroy_node(self):
        # Turn off suction on shutdown
        try:
            self._set_suction(0, 0)
        except Exception:
            pass
        if self._bus:
            self._bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SuctionDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
