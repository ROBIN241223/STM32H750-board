# STM32 ROS2 Bridge

ROS2 serial bridge package cho STM32H750 Flight Controller.

## Cài đặt

```bash
# Yêu cầu: ROS2 Humble trên Raspberry Pi 5
sudo apt install ros-humble-desktop python3-pip
pip3 install pyserial

# Build
cd ~/stm32_ws/src
ln -s /path/to/ros2_bridge stm32_bridge
cd ~/stm32_ws
colcon build --packages-select stm32_bridge
source install/setup.bash
```

## Chạy

```bash
# Bridge (serial ↔ ROS2)
ros2 launch stm32_bridge bridge.launch.py serial_port:=/dev/ttyUSB0

# Hoặc trực tiếp
ros2 run stm32_bridge bridge_node.py --ros-args -p serial_port:=/dev/ttyUSB0
```

## ROS2 Topics

### STM32 → Pi (Published)

| Topic | Type | Nội dung |
|-------|------|----------|
| `/stm32/sensor` | `Float32MultiArray` | `[timestamp, vbat, temp, adc0, adc1, adc2, adc3]` |
| `/stm32/status` | `String` | JSON heartbeat: uptime, heap, bank, firmware |
| `/stm32/fdcan_rx` | `Int32MultiArray` | `[can_id, dlc, data_byte0, ...]` |
| `/stm32/ota_resp` | `String` | JSON OTA response |

### Pi → STM32 (Subscribed)

| Topic | Type | Nội dung |
|-------|------|----------|
| `/stm32/cmd/motor` | `Float32MultiArray` | `[m1, m2, m3, m4, armed]` (0-255 PWM) |
| `/stm32/cmd/gpio` | `Int32MultiArray` | `[pin_number, value]` |
| `/stm32/cmd/fdcan_tx` | `Int32MultiArray` | `[can_id, dlc, data0, ...]` |
| `/stm32/cmd/sdlog` | `String` | `{"start": true}` hoặc `{"start": false}` |
| `/stm32/ota` | `String` | JSON OTA command (xem bên dưới) |

## Ví dụ sử dụng

```bash
# Đọc sensor data
ros2 topic echo /stm32/sensor

# Kiểm tra hệ thống
ros2 topic echo /stm32/status

# Điều khiển motor (PWM 0-255, armed=1)
ros2 topic pub /stm32/cmd/motor std_msgs/Float32MultiArray "{data: [100, 100, 100, 100, 1]}"

# Bắt đầu ghi SD card
ros2 topic pub /stm32/cmd/sdlog std_msgs/String '{"data": "{\"start\":true}"}'

# Dừng ghi SD card
ros2 topic pub /stm32/cmd/sdlog std_msgs/String '{"data": "{\"start\":false}"}'

# Gửi FDCAN
ros2 topic pub /stm32/cmd/fdcan_tx std_msgs/Int32MultiArray "{data: [256, 8, 1, 2, 3, 4, 5, 6, 7, 8]}"

# Điều khiển GPIO (VD: toggle LED PD6)
ros2 topic pub /stm32/cmd/gpio std_msgs/Int32MultiArray "{data: [6, 1]}"
```

## OTA Firmware Update

```bash
# Cập nhật firmware qua serial
ros2 run stm32_bridge ota_client.py --firmware /path/to/STM32H750.bin --port /dev/ttyUSB0

# Hoặc qua ROS2 topic
ros2 topic pub /stm32/ota std_msgs/String '{"data": "{\"cmd\":\"begin\",\"size\":71212,\"crc\":12345}"}'
```

### OTA Flow

```
1. BEGIN  → Erase App B flash region
2. DATA   → Send firmware in 256-byte chunks
3. VERIFY → CRC32 verify entire image
4. REBOOT → Swap bank, reboot into new firmware
```

### OTA Safety

- **Dual-bank**: App A luôn chạy safe, firmware mới ghi vào App B
- **CRC32**: Verify toàn bộ firmware trước khi swap
- **Boot counter**: Nếu boot fail 3 lần → rollback về App_A
- **QSPI W25Q64**: 8MB external flash cho firmware storage

## Flash Layout (QSPI W25Q64)

```
0x000000 - 0x003FFF : Config/Status (16KB)
0x004000 - 0x0FBFFF : App A - current firmware (1008KB)
0x0FC000 - 0x1F7FFF : App B - OTA target firmware (1008KB)
0x1F8000 - 0x1FFFFF : Bootloader metadata (32KB)
0x200000 - 0x7FFFFF : Reserved
```

## Wiring

```
STM32H750              Raspberry Pi 5
──────────             ──────────────
PD5 (USART2 TX)  ──►  GPIO15 (RXD0)
PA3 (USART2 RX)  ◄──  GPIO14 (TXD0)
GND               ──  GND
```

**Lưu ý:** Nếu dùng USB-UART converter (CP2102, CH340, FT232), thay `/dev/ttyUSB0` bằng `/dev/ttyACM0` hoặc `/dev/ttyUSB1`.

## Troubleshooting

| Vấn đề | Giải pháp |
|--------|-----------|
| `Permission denied` | `sudo chmod 666 /dev/ttyUSB0` hoặc thêm user vào `dialout` group |
| `No data received` | Kiểm tra wiring TX/RX, baud rate |
| `Serial port not found` | `ls /dev/ttyUSB*` hoặc `ls /dev/ttyACM*` |
| `OTA timeout` | Đảm bảo STM32 đang chạy firmware mới nhất |
