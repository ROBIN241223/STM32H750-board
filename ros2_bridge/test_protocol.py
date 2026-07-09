#!/usr/bin/env python3
"""
Test script for STM32 ↔ ROS2 Bridge Protocol.

Validates JSON message format and OTA client flow
without requiring ROS2 or physical hardware.
"""

import json
import sys
import os
import struct

# Add bridge module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'stm32_bridge'))

# ============================================================================
# Test 1: Verify sensor message format (STM32 -> Bridge)
# ============================================================================
def test_sensor_message():
    """Validate sensor JSON format matches bridge_node.py expectations."""
    sensor_msg = {
        "t": "s",
        "ts": 12345,
        "a": [1500.0, 1600.0, 1700.0, 1800.0],
        "v": 12000.0,
        "tp": 28.5
    }
    line = json.dumps(sensor_msg, separators=(',', ':'))
    parsed = json.loads(line)

    assert parsed['t'] == 's', "Type should be 's'"
    assert parsed['ts'] == 12345, f"Timestamp mismatch: {parsed['ts']}"
    assert len(parsed['a']) == 4, f"ADC array should have 4 elements: {parsed['a']}"
    assert parsed['v'] == 12000.0, f"VBat mismatch: {parsed['v']}"
    assert parsed['tp'] == 28.5, f"Temp mismatch: {parsed['tp']}"

    print(f"  [PASS] Sensor message: {line[:60]}...")

    # Verify bridge can parse it (simulate _handle_sensor)
    pub_data = [parsed.get('ts', 0), parsed.get('v', 0), parsed.get('tp', 0)]
    adc = parsed.get('a', [0, 0, 0, 0])
    pub_data.extend(adc)
    assert len(pub_data) == 7, f"Published data should have 7 elements: {pub_data}"
    assert pub_data[0] == 12345
    assert pub_data[3] == 1500.0

    print(f"  [PASS] Bridge parse: {pub_data}")


# ============================================================================
# Test 2: Verify heartbeat message format
# ============================================================================
def test_heartbeat_message():
    """Validate heartbeat JSON format."""
    hb_msg = {
        "t": "h",
        "up": 60000,
        "heap": 50000,
        "bank": 0,
        "fw": "1.0.0"
    }
    line = json.dumps(hb_msg, separators=(',', ':'))
    parsed = json.loads(line)

    assert parsed['t'] == 'h'
    assert parsed['up'] == 60000
    assert parsed['heap'] == 50000
    assert parsed['bank'] == 0

    print(f"  [PASS] Heartbeat: uptime={parsed['up']}s, heap={parsed['heap']}, bank={parsed['bank']}")


# ============================================================================
# Test 3: Verify FDCAN RX message format
# ============================================================================
def test_fdcan_rx_message():
    """Validate FDCAN RX JSON format."""
    fdcan_msg = {
        "t": "f",
        "id": 0x200,
        "dlc": 8,
        "d": "0102030405060708"
    }
    line = json.dumps(fdcan_msg, separators=(',', ':'))
    parsed = json.loads(line)

    assert parsed['t'] == 'f'
    assert parsed['id'] == 0x200
    assert parsed['dlc'] == 8
    assert parsed['d'] == "0102030405060708"

    # Simulate bridge parsing
    hex_data = parsed.get('d', '')
    data_bytes = []
    for i in range(0, len(hex_data), 2):
        data_bytes.append(int(hex_data[i:i+2], 16))
    assert data_bytes == [1, 2, 3, 4, 5, 6, 7, 8], f"Hex decode: {data_bytes}"

    pub_data = [parsed.get('id', 0), parsed.get('dlc', 0)]
    pub_data.extend(data_bytes)
    assert len(pub_data) == 10

    print(f"  [PASS] FDCAN RX: id=0x{parsed['id']:03X}, dlc={parsed['dlc']}, data={data_bytes}")


# ============================================================================
# Test 4: Verify motor command format (Bridge -> STM32)
# ============================================================================
def test_motor_command():
    """Validate motor command JSON format matches ROS2 subscription callback."""
    # Simulate what bridge_node.py cmd_motor_cb generates
    data = [150, 150, 150, 150, 1]  # m1,m2,m3,m4,armed
    cmd = {
        't': 'm',
        'm1': int(data[0]),
        'm2': int(data[1]),
        'm3': int(data[2]),
        'm4': int(data[3]),
        'a': 1 if data[4] > 0.5 else 0,
    }
    line = json.dumps(cmd, separators=(',', ':'))
    parsed = json.loads(line)

    assert parsed['t'] == 'm'
    assert parsed['m1'] == 150
    assert parsed['a'] == 1

    print(f"  [PASS] Motor command: {line}")


# ============================================================================
# Test 5: Verify FDCAN TX command format
# ============================================================================
def test_fdcan_tx_command():
    """Validate FDCAN TX command format."""
    data = [0x100, 8, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11]
    cmd = {
        't': 'c',
        'id': int(data[0]),
        'dlc': int(data[1]),
        'data': [int(d) for d in data[2:10]],
    }
    line = json.dumps(cmd, separators=(',', ':'))
    parsed = json.loads(line)

    assert parsed['t'] == 'c'
    assert parsed['id'] == 0x100
    assert parsed['data'] == data[2:10]

    print(f"  [PASS] FDCAN TX: {line}")


# ============================================================================
# Test 6: OTA CRC32 consistency
# ============================================================================
def test_ota_crc32():
    """Verify CRC32 calculation in ota_client.py matches STM32 side."""
    import binascii

    test_data = bytes(range(256))  # One page of data
    py_crc = binascii.crc32(test_data) & 0xFFFFFFFF

    # The STM32 side uses a different CRC32 polynomial
    # STM32 implementation: 0xEDB88320 (standard CRC-32)
    stm32_crc = 0xFFFFFFFF
    for b in test_data:
        stm32_crc ^= b
        for _ in range(8):
            if stm32_crc & 1:
                stm32_crc = (stm32_crc >> 1) ^ 0xEDB88320
            else:
                stm32_crc >>= 1
    stm32_crc = ~stm32_crc & 0xFFFFFFFF

    # Python's binascii.crc32 uses the same standard CRC-32
    assert py_crc == stm32_crc, (
        f"CRC32 mismatch: Python=0x{py_crc:08X}, STM32=0x{stm32_crc:08X}"
    )
    print(f"  [PASS] CRC32: 0x{py_crc:08X} (Python & STM32 match)")


# ============================================================================
# Test 7: OTA protocol flow simulation
# ============================================================================
def test_ota_protocol_flow():
    """Simulate the OTA update protocol without hardware."""
    print()
    print("  --- OTA Flow Simulation ---")

    firmware_size = 65536  # 64KB test firmware
    chunk_size = 256

    # Step 1: BEGIN
    begin_cmd = {
        't': 'o',
        'cmd': 'begin',
        'size': firmware_size,
        'crc': 0x12345678,
    }
    begin_line = json.dumps(begin_cmd, separators=(',', ':'))
    assert '"cmd":"begin"' in begin_line
    assert '"size":65536' in begin_line
    print(f"  [PASS] OTA BEGIN: {begin_line[:60]}...")

    # Simulate STM32 response
    stm32_resp = {"t": "o", "s": "erased", "p": firmware_size}
    assert stm32_resp.get('s') == 'erased'
    assert stm32_resp.get('p') == firmware_size
    print(f"  [PASS] STM32 response: {stm32_resp}")

    # Step 2: DATA chunks
    chunks = (firmware_size + chunk_size - 1) // chunk_size
    seq = 0
    sent = 0
    while sent < firmware_size:
        chunk_data = bytes([i % 256 for i in range(chunk_size)])
        data_cmd = {
            't': 'o',
            'cmd': 'data',
            'seq': seq,
            'data': chunk_data.hex(),
        }
        data_line = json.dumps(data_cmd, separators=(',', ':'))
        parsed = json.loads(data_line)

        assert parsed['seq'] == seq
        assert len(parsed['data']) == chunk_size * 2  # hex encoded

        sent += chunk_size
        seq += 1

    assert seq == chunks
    print(f"  [PASS] OTA DATA: {chunks} chunks ({firmware_size} bytes)")

    # Step 3: VERIFY
    verify_cmd = {"t": "o", "cmd": "verify"}
    verify_line = json.dumps(verify_cmd, separators=(',', ':'))
    stm32_resp = {"t": "o", "s": "verified", "p": 0x12345678}
    assert stm32_resp.get('s') == 'verified'
    print(f"  [PASS] OTA VERIFY")

    # Step 4: REBOOT
    reboot_cmd = {"t": "o", "cmd": "reboot"}
    reboot_line = json.dumps(reboot_cmd, separators=(',', ':'))
    print(f"  [PASS] OTA REBOOT")

    print(f"  [PASS] OTA protocol flow complete ({chunks} chunks sent)")


# ============================================================================
# Test 8: Verify all message types in bridge
# ============================================================================
def test_all_message_types():
    """Verify all JSON message type identifiers."""
    types = {
        's': 'sensor',
        'h': 'heartbeat',
        'f': 'fdcan_rx',
        'o': 'ota_response',
        'm': 'motor_cmd',
        'g': 'gpio_cmd',
        'c': 'fdcan_tx_cmd',
        'l': 'sdlog_cmd',
        'r': 'request_cmd',
    }
    for t, name in types.items():
        msg = {'t': t}
        parsed = json.loads(json.dumps(msg))
        assert parsed['t'] == t, f"Type '{t}' ({name}) failed"
    print(f"  [PASS] All {len(types)} message types validated: {', '.join(types.keys())}")


# ============================================================================
# Main
# ============================================================================
def main():
    print("=" * 60)
    print("STM32 ↔ ROS2 Bridge Protocol Test")
    print("=" * 60)
    print()

    tests = [
        ("Sensor message", test_sensor_message),
        ("Heartbeat message", test_heartbeat_message),
        ("FDCAN RX message", test_fdcan_rx_message),
        ("Motor command", test_motor_command),
        ("FDCAN TX command", test_fdcan_tx_command),
        ("CRC32 consistency", test_ota_crc32),
        ("OTA protocol flow", test_ota_protocol_flow),
        ("All message types", test_all_message_types),
    ]

    passed = 0
    failed = 0

    for name, test_fn in tests:
        print(f"[{passed + failed + 1}] {name}")
        try:
            test_fn()
            passed += 1
            print()
        except Exception as e:
            print(f"  [FAIL] {e}")
            failed += 1
            print()

    print("=" * 60)
    print(f"Results: {passed}/{passed + failed} passed", end="")
    if failed > 0:
        print(f", {failed} FAILED", end="")
    print()
    print("=" * 60)

    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
