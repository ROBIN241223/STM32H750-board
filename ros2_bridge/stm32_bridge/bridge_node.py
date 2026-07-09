#!/usr/bin/env python3
"""
ROS2 Serial Bridge for STM32H750 Flight Controller

This node bridges between ROS2 topics and the STM32 via UART serial.
It reads JSON messages from the STM32 and publishes them as ROS2 topics,
and subscribes to ROS2 commands and sends them to the STM32.

STM32 -> Pi (published topics):
  /stm32/sensor    (Float32MultiArray)  - ADC channels, vbat, temperature
  /stm32/status    (String)             - Heartbeat, uptime, heap, firmware version
  /stm32/fdcan_rx  (Int32MultiArray)    - FDCAN messages received by STM32
  /stm32/ota_resp  (String)             - OTA status responses

Pi -> STM32 (subscribed topics):
  /stm32/cmd/motor     (Float32MultiArray)  - Motor PWM commands [m1,m2,m3,m4,armed]
  /stm32/cmd/gpio      (Int32MultiArray)    - GPIO commands [pin, value]
  /stm32/cmd/fdcan_tx  (Int32MultiArray)    - FDCAN TX [id, dlc, data...]
  /stm32/cmd/sdlog     (String)             - SD log control {"start":true/false}
  /stm32/ota           (String)             - OTA commands (JSON)
"""

import json
import threading
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray


class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('log_level', 'info')

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self.get_logger().info(f'Starting STM32 Bridge on {port} @ {baud}')

        # Serial connection
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Serial port {port} opened')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {port}: {e}')
            raise

        # Publishers (STM32 -> Pi)
        self.pub_sensor = self.create_publisher(Float32MultiArray, '/stm32/sensor', 10)
        self.pub_status = self.create_publisher(String, '/stm32/status', 10)
        self.pub_fdcan_rx = self.create_publisher(Int32MultiArray, '/stm32/fdcan_rx', 10)
        self.pub_ota_resp = self.create_publisher(String, '/stm32/ota_resp', 10)

        # Subscribers (Pi -> STM32)
        self.sub_motor = self.create_subscription(
            Float32MultiArray, '/stm32/cmd/motor', self.cmd_motor_cb, 10)
        self.sub_gpio = self.create_subscription(
            Int32MultiArray, '/stm32/cmd/gpio', self.cmd_gpio_cb, 10)
        self.sub_fdcan_tx = self.create_subscription(
            Int32MultiArray, '/stm32/cmd/fdcan_tx', self.cmd_fdcan_tx_cb, 10)
        self.sub_sdlog = self.create_subscription(
            String, '/stm32/cmd/sdlog', self.cmd_sdlog_cb, 10)
        self.sub_ota = self.create_subscription(
            String, '/stm32/ota', self.cmd_ota_cb, 10)

        # Serial reader thread
        self.running = True
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()

        self.get_logger().info('STM32 Bridge started successfully')

    def _serial_reader(self):
        """Background thread that reads serial data and dispatches."""
        line_buffer = ''
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    line_buffer += data

                    while '\n' in line_buffer:
                        line, line_buffer = line_buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._process_stm32_message(line)
                else:
                    threading.Event().wait(0.01)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                threading.Event().wait(0.5)
            except Exception as e:
                self.get_logger().error(f'Reader error: {e}')
                threading.Event().wait(0.1)

    def _process_stm32_message(self, line):
        """Parse JSON from STM32 and publish to ROS2 topics."""
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid JSON: {line[:80]}')
            return

        msg_type = msg.get('t', '')

        if msg_type == 's':
            self._handle_sensor(msg)
        elif msg_type == 'h':
            self._handle_heartbeat(msg)
        elif msg_type == 'f':
            self._handle_fdcan_rx(msg)
        elif msg_type == 'o':
            self._handle_ota_response(msg)
        else:
            self.get_logger().debug(f'Unknown message type: {msg_type}')

    def _handle_sensor(self, msg):
        pub = Float32MultiArray()
        pub.data = [
            msg.get('ts', 0),
            msg.get('v', 0),
            msg.get('tp', 0),
        ]
        # Add ADC channels
        adc = msg.get('a', [0, 0, 0, 0])
        pub.data.extend(adc)
        self.pub_sensor.publish(pub)

    def _handle_heartbeat(self, msg):
        pub = String()
        pub.data = json.dumps(msg)
        self.pub_status.publish(pub)

    def _handle_fdcan_rx(self, msg):
        pub = Int32MultiArray()
        pub.data = [msg.get('id', 0), msg.get('dlc', 0)]
        hex_data = msg.get('d', '')
        for i in range(0, len(hex_data), 2):
            pub.data.append(int(hex_data[i:i+2], 16))
        self.pub_fdcan_rx.publish(pub)

    def _handle_ota_response(self, msg):
        pub = String()
        pub.data = json.dumps(msg)
        self.pub_ota_resp.publish(pub)

    # --- Command Callbacks (Pi -> STM32) ---

    def _send_json(self, obj):
        line = json.dumps(obj, separators=(',', ':')) + '\n'
        try:
            self.ser.write(line.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def cmd_motor_cb(self, msg):
        data = msg.data
        if len(data) >= 5:
            self._send_json({
                't': 'm',
                'm1': int(data[0]),
                'm2': int(data[1]),
                'm3': int(data[2]),
                'm4': int(data[3]),
                'a': 1 if data[4] > 0.5 else 0,
            })

    def cmd_gpio_cb(self, msg):
        data = msg.data
        if len(data) >= 2:
            self._send_json({
                't': 'g',
                'pin': int(data[0]),
                'val': int(data[1]),
            })

    def cmd_fdcan_tx_cb(self, msg):
        data = msg.data
        if len(data) >= 2:
            self._send_json({
                't': 'c',
                'id': int(data[0]),
                'dlc': int(data[1]),
                'data': [int(d) for d in data[2:10]],
            })

    def cmd_sdlog_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
            self._send_json({
                't': 'l',
                'start': 1 if cmd.get('start', False) else 0,
            })
        except json.JSONDecodeError:
            pass

    def cmd_ota_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
            cmd['t'] = 'o'
            self._send_json(cmd)
        except json.JSONDecodeError:
            pass

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'reader_thread'):
            self.reader_thread.join(timeout=2)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
