#!/usr/bin/env python3
"""
OTA Firmware Update Client for STM32H750

This node handles firmware updates over serial.
It reads a .bin firmware file and sends it to the STM32 via the OTA protocol.

Usage:
  ros2 run stm32_bridge ota_client.py --firmware /path/to/firmware.bin --port /dev/ttyUSB0

OTA Flow:
  1. Send BEGIN with size and CRC32
  2. Wait for ERASED response
  3. Send DATA chunks (256 bytes each)
  4. Send VERIFY
  5. Wait for VERIFIED response
  6. Send REBOOT
"""

import argparse
import binascii
import json
import sys
import time
import serial


def calculate_crc32(filepath):
    """Calculate CRC32 of a file."""
    crc = 0xFFFFFFFF
    with open(filepath, 'rb') as f:
        while True:
            chunk = f.read(4096)
            if not chunk:
                break
            crc = binascii.crc32(chunk, crc)
    return crc & 0xFFFFFFFF


def send_and_wait(ser, msg, expected_status=None, timeout=30):
    """Send JSON message and wait for response."""
    line = json.dumps(msg, separators=(',', ':')) + '\n'
    ser.write(line.encode('utf-8'))
    print(f'  TX: {line.strip()}')

    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                try:
                    resp = json.loads(data)
                    print(f'  RX: {resp}')
                    if expected_status and resp.get('t') == 'o':
                        if resp.get('s') == expected_status:
                            return resp
                        elif resp.get('s', '').startswith('error'):
                            print(f'  ERROR: {resp}')
                            return None
                except json.JSONDecodeError:
                    print(f'  RX (raw): {data}')
        time.sleep(0.01)

    print(f'  TIMEOUT waiting for {expected_status}')
    return None


def main():
    parser = argparse.ArgumentParser(description='STM32H750 OTA Firmware Updater')
    parser.add_argument('--firmware', '-f', required=True, help='Path to firmware .bin file')
    parser.add_argument('--port', '-p', default='/dev/ttyUSB0', help='Serial port')
    parser.add_argument('--baud', '-b', type=int, default=115200, help='Baud rate')
    parser.add_argument('--chunk-size', type=int, default=256, help='Chunk size (bytes)')
    parser.add_argument('--verify-only', action='store_true', help='Only verify, don\'t reboot')
    args = parser.parse_args()

    print(f'=== STM32H750 OTA Firmware Updater ===')
    print(f'Firmware: {args.firmware}')
    print(f'Port: {args.port} @ {args.baud}')
    print()

    # Read firmware
    print('[1] Reading firmware file...')
    with open(args.firmware, 'rb') as f:
        firmware_data = f.read()
    firmware_size = len(firmware_data)
    firmware_crc = calculate_crc32(args.firmware)
    print(f'  Size: {firmware_size} bytes ({firmware_size/1024:.1f} KB)')
    print(f'  CRC32: 0x{firmware_crc:08X}')
    print()

    # Connect to STM32
    print('[2] Connecting to STM32...')
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.5)  # Wait for connection to stabilize
    print('  Connected!')
    print()

    # Send OTA BEGIN
    print('[3] Sending OTA BEGIN...')
    resp = send_and_wait(ser, {
        't': 'o',
        'cmd': 'begin',
        'size': firmware_size,
        'crc': firmware_crc,
    }, expected_status='erased', timeout=60)

    if not resp:
        print('  FAILED: Could not erase flash')
        ser.close()
        sys.exit(1)
    print('  Flash erased successfully!')
    print()

    # Send firmware chunks
    print(f'[4] Sending firmware ({(firmware_size + args.chunk_size - 1) // args.chunk_size} chunks)...')
    seq = 0
    sent = 0
    start_time = time.time()

    while sent < firmware_size:
        chunk = firmware_data[sent:sent + args.chunk_size]
        chunk_data_hex = chunk.hex()

        resp = send_and_wait(ser, {
            't': 'o',
            'cmd': 'data',
            'seq': seq,
            'data': chunk_data_hex,
        }, expected_status='ok', timeout=10)

        if not resp:
            print(f'  FAILED at chunk {seq}')
            ser.close()
            sys.exit(1)

        sent += len(chunk)
        seq += 1

        # Progress bar
        pct = (sent * 100) // firmware_size
        elapsed = time.time() - start_time
        speed = sent / elapsed if elapsed > 0 else 0
        bar_len = 30
        filled = int(bar_len * sent / firmware_size)
        bar = '#' * filled + '-' * (bar_len - filled)
        print(f'\r  [{bar}] {pct}% ({sent}/{firmware_size}) {speed:.0f} B/s', end='', flush=True)

    print()
    print(f'  All chunks sent in {time.time() - start_time:.1f}s')
    print()

    # Verify
    print('[5] Verifying firmware...')
    resp = send_and_wait(ser, {
        't': 'o',
        'cmd': 'verify',
    }, expected_status='verified', timeout=120)

    if not resp:
        print('  FAILED: CRC verification failed')
        ser.close()
        sys.exit(1)
    print(f'  CRC verified: 0x{resp.get("p", 0):08X}')
    print()

    # Reboot
    if not args.verify_only:
        print('[6] Rebooting STM32...')
        send_and_wait(ser, {
            't': 'o',
            'cmd': 'reboot',
        }, timeout=5)
        print('  Reboot command sent!')
    else:
        print('[6] Skipping reboot (--verify-only)')

    ser.close()
    print()
    print('=== OTA Update Complete! ===')


if __name__ == '__main__':
    main()
