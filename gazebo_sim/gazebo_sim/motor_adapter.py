#!/usr/bin/env python3
"""Convert the STM32 motor topic to Gazebo actuator velocities."""

import math

import rclpy
from actuator_msgs.msg import Actuators
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MotorAdapter(Node):
    """Bridge the existing STM32 motor command interface to Gazebo."""

    def __init__(self) -> None:
        super().__init__('motor_adapter')

        self.declare_parameter('max_rotor_velocity', 800.0)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('publish_rate', 100.0)

        self.max_rotor_velocity = float(
            self.get_parameter('max_rotor_velocity').value
        )
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.last_command = [0.0, 0.0, 0.0, 0.0]
        self.armed = False
        self.last_command_time = self.get_clock().now()

        self.command_sub = self.create_subscription(
            Float32MultiArray,
            '/stm32/cmd/motor',
            self.command_callback,
            10,
        )
        self.actuator_pub = self.create_publisher(
            Actuators,
            '/quadcopter/command/motor_speed',
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)

        self.get_logger().info(
            'Motor adapter ready: /stm32/cmd/motor -> '
            '/quadcopter/command/motor_speed'
        )

    def command_callback(self, message: Float32MultiArray) -> None:
        """Store a five-element [m1, m2, m3, m4, armed] command."""
        if len(message.data) < 5:
            self.get_logger().warning('Ignoring motor command with fewer than 5 values')
            return

        values = [float(value) for value in message.data[:4]]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning('Ignoring non-finite motor command')
            return

        self.last_command = [max(0.0, min(255.0, value)) for value in values]
        self.armed = float(message.data[4]) > 0.5
        self.last_command_time = self.get_clock().now()

    def publish_command(self) -> None:
        """Publish zero on disarm or timeout, otherwise publish rotor speeds."""
        age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        active = self.armed and age <= self.command_timeout
        scale = self.max_rotor_velocity / 255.0

        message = Actuators()
        message.velocity = [value * scale for value in self.last_command] if active else [0.0] * 4
        self.actuator_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
