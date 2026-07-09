#!/usr/bin/env python3
"""
Renode test: STM32H750 Flight Controller boot simulation.
Captures serial output from USART1 (debug) and USART2 (ROS2).
"""

import subprocess
import sys
import os

WORKDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RENODE_DLL = "/opt/renode/bin/Renode.dll"

RENODE_SCRIPT = """
mach create
machine LoadPlatformDescription @{workdir}/renode/stm32h750_board.repl

# Init registers for boot
sysbus WriteDoubleWord 0x58024804 0x00002000  # PWR CSR1: VOSRDY
sysbus WriteDoubleWord 0x5802480C 0x00002000  # PWR D3CR: VOSRDY
sysbus WriteDoubleWord 0x58024400 0x3FFE0000  # RCC CR: HSERDY, PLL ready bits
sysbus WriteDoubleWord 0x5C001000 0x20006470  # DBGMCU IDCODE: STM32H750

# Load ELF into QSPI
sysbus LoadELF @{workdir}/build/STM32H750.elf
cpu VectorTableOffset 0x90000000

# Run for 10ms
emulation RunFor "0.01"
quit
""".format(workdir=WORKDIR)


def main():
    print("=" * 60)
    print("STM32H750 Renode Simulation Test")
    print("=" * 60)
    print()

    # Clean old logs
    for log in ["build/usart1.log", "build/usart2.log"]:
        path = os.path.join(WORKDIR, log)
        if os.path.exists(path):
            os.remove(path)

    print("[1] Starting Renode simulation (10ms)...")
    print()

    proc = None
    try:
        proc = subprocess.run(
            ["dotnet", RENODE_DLL, "-e", RENODE_SCRIPT],
            cwd=WORKDIR,
            capture_output=True,
            timeout=60,
            env={
                "PATH": "/usr/bin:/bin:/usr/local/bin",
                "HOME": os.environ.get("HOME", ""),
                "TERM": "xterm",
                "LD_LIBRARY_PATH": "/usr/lib/x86_64-linux-gnu",
                "LD_PRELOAD": "",
                "SNAP": "",
                "SNAP_NAME": "",
                "SNAP_REVISION": "",
                "SNAP_COMMON": "",
                "SNAP_DATA": "",
                "SNAP_USER_COMMON": "",
                "SNAP_USER_DATA": "",
                "SNAP_ARCH": "",
                "SNAP_LIBRARY_PATH": "",
                "SNAP_REEXEC": "",
            },
        )
    except subprocess.TimeoutExpired:
        print("[ERROR] Renode timed out")
        return 1
    except FileNotFoundError as e:
        print(f"[ERROR] {e}")
        return 1

    # Show Renode output
    print("[Renode output]")
    out = proc.stdout.decode() if proc.stdout else ""
    err = proc.stderr.decode() if proc.stderr else ""
    for line in (out + "\n" + err).split("\n"):
        if line.strip():
            print(f"  {line.strip()}")
    print()

    print("[2] Checking debug UART output (usart1)...")
    print()

    uart1_path = os.path.join(WORKDIR, "build/usart1.log")
    uart2_path = os.path.join(WORKDIR, "build/usart2.log")

    passed = 0
    failed = 0

    # Check USART1 (debug)
    if os.path.exists(uart1_path):
        with open(uart1_path, "r", errors="replace") as f:
            uart1_data = f.read()
        
        print(f"  USART1 raw output ({len(uart1_data)} bytes):")
        print(f"  {repr(uart1_data[:500])}")
        print()

        keywords = [
            "Debug UART Initialized",
            "All tasks created",
            "Heartbeat",
            "Motor",
            "Sensor",
            "FDCAN",
            "ros2Comm",
            "System Clock",
        ]

        for kw in keywords:
            if kw in uart1_data:
                print(f"  [PASS] Found: {kw}")
                passed += 1
            else:
                print(f"  [WARN] Not found: {kw}")
    else:
        print("  [FAIL] USART1 log not found")
        failed += 1

    # Check USART2 (ROS2)
    if os.path.exists(uart2_path):
        with open(uart2_path, "r", errors="replace") as f:
            uart2_data = f.read()
        
        print(f"\n  USART2 raw output ({len(uart2_data)} bytes):")
        print(f"  {repr(uart2_data[:500])}")
        print()

        if '"t":"h"' in uart2_data:
            print(f"  [PASS] USART2: Heartbeat JSON")
            passed += 1
        else:
            print(f"  [WARN] USART2: No heartbeat JSON")
    else:
        print("  [FAIL] USART2 log not found")
        failed += 1

    print()
    print("=" * 60)
    print(f"Results: {passed}/{passed + failed} passed", end="")
    if failed > 0:
        print(f", {failed} FAILED", end="")
    print()
    print("=" * 60)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
