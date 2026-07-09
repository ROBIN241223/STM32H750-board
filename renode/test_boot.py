"""
Renode test: STM32H750 Flight Controller boot verification.

Checks:
1. Debug UART (USART1) outputs initialization messages
2. Heartbeat task starts and sends status
"""

import sys
import os
from renode_test import *

class STM32BootTest(TestCase):
    def setUp(self):
        self.uart = None
        self.uart2 = None

    def test_boot_sequence(self):
        """Verify firmware boots and outputs expected debug messages."""
        # Load platform
        self.machine = self.emulation.Machines[0]
        self.machine.LoadPlatformDescription(
            os.path.join(self.renode_dir, "renode/stm32h750_board.repl")
        )

        # Get serial analyzers
        self.uart = self.machine.GetUart("usart1")
        self.uart2 = self.machine.GetUart("usart2")

        # Load ELF
        self.machine.LoadELF(
            os.path.join(self.renode_dir, "build/STM32H750.elf")
        )

        # Set vector table offset for QSPI execution
        self.machine.cpu.VectorTableOffset = 0x90000000

        # Start simulation
        self.machine.Start()

        # Wait for debug UART initialization message
        with self.uart.StartRecording() as records:
            self.uart.WaitFor(
                "Debug UART Initialized",
                timeout=5.0,
                message="Expected debug UART init message"
            )

        # Wait for FreeRTOS task creation messages
        with self.uart.StartRecording() as records:
            self.uart.WaitFor(
                "All tasks created",
                timeout=10.0,
                message="Expected all tasks created message"
            )

        # Wait for heartbeat task to start
        with self.uart.StartRecording() as records:
            self.uart.WaitFor(
                "Heartbeat",
                timeout=10.0,
                message="Expected heartbeat task message"
            )

        # Wait for motor task to start
        with self.uart.StartRecording() as records:
            self.uart.WaitFor(
                "Motor",
                timeout=10.0,
                message="Expected motor task message"
            )

        # Verify ROS2 UART outputs heartbeat JSON
        with self.uart2.StartRecording() as records:
            self.uart2.WaitFor(
                '"t":"h"',
                timeout=15.0,
                message="Expected heartbeat JSON on USART2"
            )

        # Check for sensor data JSON
        with self.uart2.StartRecording() as records:
            self.uart2.WaitFor(
                '"t":"s"',
                timeout=15.0,
                message="Expected sensor data JSON on USART2"
            )

        print("ALL BOOT TESTS PASSED")
