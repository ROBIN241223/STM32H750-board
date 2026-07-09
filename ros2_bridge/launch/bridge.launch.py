#!/usr/bin/env python3
"""
Launch file for STM32 ROS2 Bridge
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for STM32 communication'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    bridge_node = Node(
        package='stm32_bridge',
        executable='bridge_node.py',
        name='stm32_bridge',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
        }],
        output='screen',
        remappings=[],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        bridge_node,
    ])
