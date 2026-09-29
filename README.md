# STM32H750 Flight Controller with ROS2

STM32H750VBTx flight controller firmware với tích hợp ROS2 qua UART trên Raspberry Pi 5.

## Kiến trúc hệ thống

```
┌─────────────────────────────────┐     UART2 (115200)     ┌──────────────────────┐
│       STM32H750 @ 480MHz        │◄──────────────────────►│   Raspberry Pi 5     │
│       FreeRTOS, 128KB heap      │   PD5(TX) / PA3(RX)   │   ROS2 Jazzy         │
│                                 │                        │                      │
│  Sensor Task (50Hz) ────────────┤                        │  /stm32/sensor       │
│  Motor Task (listen) ───────────┤                        │  /stm32/status       │
│  FDCAN Task (100Hz) ────────────┤                        │  /stm32/fdcan_rx     │
│  SD Logger Task (10Hz) ─────────┤                        │  /stm32/cmd/motor    │
│  ROS2 Comm Task ────────────────┤                        │  /stm32/cmd/gpio     │
│  Heartbeat Task (1Hz) ──────────┤                        │  /stm32/cmd/fdcan_tx │
│                                 │                        │  /stm32/cmd/sdlog    │
│  QSPI W25Q64 (OTA firmware) ────┤                        │  /stm32/ota          │
│  SD Card (data logging) ────────┤                        │                      │
└─────────────────────────────────┘                        └──────────────────────┘
```

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | STM32H750VBTx @ 480MHz |
| Flash | 128KB internal + W25Q64 8MB QSPI |
| RAM | 128KB DTCMRAM + 512KB RAM + 288KB RAM_D2 + 64KB RAM_D3 |
| UART Debug | USART1 (PA9/PA10, 115200) with RTS/CTS |
| UART ROS2 | USART2 (PD5/PA3, 115200) |
| Motor PWM | TIM2 4-channel (PA0, PA1, PA2, PB11) @ 50Hz |
| ADC | ADC3 16-bit, 4 channels (PC2_C) |
| FDCAN | FDCAN2 (PB13-TX, PB5-RX) @ 1.333 Mbit/s |
| SD Card | SDMMC1 4-bit (PD2, PC7-PC11) |
| IMU 0 (ICM-42688) | SPI1 (SCK:PA5, MISO:PB4, MOSI:PD7, CS:PA15) @ 16MHz |
| IMU 1 (ICM-42688) | SPI2 (SCK:PB10, MISO:PB14, MOSI:PB15, CS:PB12) @ 16MHz |
| Compass | BMM150 over I2C1 (SCL:PB8, SDA:PB9) |
| Camera | DCMI 8-bit parallel |
| RC radio | NRF24L01 (UART-transparent) over USART3 (PD8/PD9) + GPIO PD4/PD14 |
| GPS 1 | UART7 (PE7-RX, PE8-TX) |
| GPS 2 | UART4 (PD0-RX, PD1-TX) |
| RTOS | FreeRTOS 10.6.2 |

## Wiring (STM32 ↔ Raspberry Pi 5)

```
STM32 PD5 (USART2 TX)  ──►  RPi GPIO15 (RXD0)
STM32 PA3 (USART2 RX)  ◄──  RPi GPIO14 (TXD0)
STM32 GND               ──  RPi GND
```

## Pin Mapping

### Motor ESC (TIM2 PWM 50Hz)

| Motor | Channel | Pin | Pulse Range |
|-------|---------|-----|-------------|
| M1 | TIM2_CH1 | PA0 | 1000-2000us |
| M2 | TIM2_CH2 | PA1 | 1000-2000us |
| M3 | TIM2_CH3 | PA2 | 1000-2000us |
| M4 | TIM2_CH4 | PB11 | 1000-2000us |

### UART Interfaces

| Interface | Pins | Baud | Function |
|-----------|------|------|----------|
| USART1 | PA9/PA10 (RTS:PA11, CTS:PA12) | 115200 | Debug console |
| USART2 | PD5/PA3 | 115200 | ROS2 bridge |
| USART3 | PD8/PD9 | 115200 | NRF24L01 RC radio (UART-transparent) |
| UART4 | PD0-RX/PD1-TX | 115200 | GPS 2 |
| UART7 | PE7-RX/PE8-TX | 115200 | GPS 1 |

### IMU (2x ICM-42688, SPI)

Hai IMU dùng hai bus SPI độc lập (mỗi bus một CS riêng), mỗi line có trở nối tiếp 47R. Nguồn IMU lấy từ `VDD_3V_PERIPH`.

| Signal | IMU 0 (SPI1) | IMU 1 (SPI2) |
|--------|--------------|--------------|
| SCK    | PA5          | PB10         |
| MISO   | PB4          | PB14         |
| MOSI   | PD7          | PB15         |
| CS/NSS | PA15         | PB12         |

> Ghi chú: ICM-42688 SPI max 24 MHz; SPI1 và SPI2 đều set 16 MHz. INT1/INT2 (DRDY) và mapping trục của 2 IMU chưa xác nhận (ICM-42688 nằm ở board mezzanine, sheet chưa có trong `FMUH750.pdf`).

## Firmware Modules

### ros2_comm (UART2 Protocol)
- JSON-based protocol over USART2
- CRC16 integrity check
- TX: sensor data, heartbeat, FDCAN RX, OTA responses
- RX: motor commands, GPIO, FDCAN TX, SD log control, OTA commands

### ros2_sensor (ADC3 Reading)
- 16-bit ADC, 4 channels, 50Hz publish rate
- Battery voltage with voltage divider
- MCU internal temperature

### ros2_motor (ESC Control)
- TIM2 PWM 4-channel, 50Hz
- ARM/DISARM sequence
- Failsafe: auto-stop if no command for 500ms
- Emergency stop

### fdcan_comm (FDCAN2)
- Classic CAN mode, 1.333 Mbit/s
- Hardware RX FIFO with filter
- Forward FDCAN RX to ROS2, receive TX from ROS2

### sd_logger (SD Card)
- Binary record format (64 bytes/record)
- Buffered writes (16 records before flush)
- Start/stop via ROS2 command
- Records: timestamp, ADC, vbat, temp, FDCAN, motor PWM

### ota_update (QSPI Flash)
- W25Q64 8MB QSPI flash
- Dual-bank: App A (current) + App B (OTA target)
- Flash layout: Config(16KB) + AppA(1MB) + AppB(1MB) + BootMeta(32KB)
- CRC32 verification before bank swap
- Boot count rollback protection

### Flight control stack (`Core/Src/fc_*.c`)
PX4-style cascade, all hardware-independent so the same sources build for firmware and for
the Gazebo SIL (`controller/px4_x500.py` loads them via ctypes):

| Module | Role (PX4 equivalent) |
|--------|------------------------|
| `fc_sched` | Cooperative work-queue scheduler, 500 Hz source tick |
| `fc_orb` | uORB-style topic bus (10 topics) |
| `fc_param` | Param server with PX4 names (`MC_*`, `MPC_*`, `FD_*`, `LNDMC_*`) |
| `fc_estimator` | Quaternion complementary estimator, accel-gated correction |
| `fc_position` | Position/velocity loop, accel feed-forward, `MPC_ACC_HOR_MAX` clamp |
| `fc_attitude` | Attitude control, yaw-split, rate limit |
| `fc_rate` | Rate P/I/D + FF, anti-windup, yaw torque LPF |
| `fc_mixer` | Control allocator with `CA_ROTOR*` geometry, pseudo-inverse, desaturation |
| `fc_land` | Land detector (freefall → ground_contact → maybe_landed → landed) |
| `fc_guard` | STAB/RECOVER/teleport guard, per-axis thresholds + trigger time |
| `fc_mission` | Mission navigator (TAKEOFF/WAYPOINT/LOITER/RTL/LAND) |

`imu_icm42688` + `imu_port_spi` drive the two ICM-42688 over SPI with polling (the board
exposes no DRDY/INT net); `flight_controller` owns the pipeline, health flags and the
flight/manual motor handover, and `ROS2_Motor` is the only writer of TIM2.

## ROS2 Topics

| Topic | Direction | Message | Description |
|-------|-----------|---------|-------------|
| `/stm32/sensor` | STM32→Pi | `Float32MultiArray` | [ts, vbat, temp, adc0-3] |
| `/stm32/status` | STM32→Pi | `String` (JSON) | Heartbeat: uptime, heap, bank, firmware |
| `/stm32/fdcan_rx` | STM32→Pi | `Int32MultiArray` | [id, dlc, data...] |
| `/stm32/ota_resp` | STM32→Pi | `String` (JSON) | OTA status responses |
| `/stm32/cmd/motor` | Pi→STM32 | `Float32MultiArray` | [m1, m2, m3, m4, armed] |
| `/stm32/cmd/flight` | Pi→STM32 | `Float32MultiArray` | [thrust, roll_rad, pitch_rad, yaw_rate, armed] |
| `/stm32/cmd/gpio` | Pi→STM32 | `Int32MultiArray` | [pin, value] |
| `/stm32/cmd/fdcan_tx` | Pi→STM32 | `Int32MultiArray` | [id, dlc, data...] |
| `/stm32/cmd/sdlog` | Pi→STM32 | `String` (JSON) | `{"start":true/false}` |
| `/stm32/ota` | Pi→STM32 | `String` (JSON) | OTA commands |

## Build

```bash
# Clean build
make clean && make -j$(nproc)

# Output
# build/STM32H750.bin  - raw binary for flashing
# build/STM32H750.hex  - Intel HEX
# build/STM32H750.elf  - ELF with debug symbols
```

## Host Tests

The `fc_*` modules do not touch HAL or FreeRTOS, so they are tested with a host compiler:

```bash
gcc -std=c11 -Wall -Wextra -ICore/Inc \
  tests/test_fc_math.c Core/Src/fc_*.c \
  Core/Src/imu_icm42688.c Core/Src/imu_port_spi.c -lm -o /tmp/test_fc_math
/tmp/test_fc_math   # ends with ALL TESTS PASSED
```

## Gazebo Simulation (firmware-in-the-loop)

```bash
# Headless server, or server + GUI window
bash gazebo_sim/run_gazebo.sh
bash gazebo_sim/run_gazebo_gui.sh

# Fly a mission (same GZ_PARTITION as the sim)
export GZ_IP=127.0.0.1 GZ_PARTITION=stm32_h750_sim
GZ_MISSION='22 0 0 0 n 0 0 3.0 ; 16 30 0.8 0 n 0 0 3.0 ; 20 0 0 0 n 0 0 0 ; 21 0 0 0 n 0 0 0' \
  python3 -u gazebo_sim/controller/px4_x500.py 3.0 60.0 0.3 0.3

# Analyze the resulting flight log
python3 tools/analyze_flight.py gazebo_sim/logs/flight-*.csv
```

Mission commands are MAVLink codes: 22 TAKEOFF, 16 WAYPOINT, 18 LOITER_TURNS,
19 LOITER_TIME, 20 RTL, 21 LAND.

## Flash via ST-Link

```bash
# Using STM32CubeProgrammer CLI
STM32_Programmer_CLI -c port=SWD -w build/STM32H750.bin 0x08000000 -v -rst

# Using OpenOCD
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program build/STM32H750.elf verify reset exit"
```

## Raspberry Pi 5 Setup

```bash
# Install ROS2 Jazzy
sudo apt update && sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Build bridge workspace
mkdir -p ~/stm32_ws/src
ln -sf /path/to/STM32H750-board/ros2_bridge ~/stm32_ws/src/stm32_bridge
cd ~/stm32_ws
colcon build --packages-select stm32_bridge
source install/setup.bash

# Run bridge
ros2 launch stm32_bridge bridge.launch.py serial_port:=/dev/ttyUSB0

# OTA firmware update
ros2 run stm32_bridge ota_client.py -f build/STM32H750.bin -p /dev/ttyUSB0
```

## ROS2 Usage Examples

```bash
# Read sensor data
ros2 topic echo /stm32/sensor

# Control motors (PWM values 0-255, armed=1)
ros2 topic pub /stm32/cmd/motor std_msgs/Float32MultiArray "{data: [1500,1500,1500,1500,1]}"

# Start SD logging
ros2 topic pub /stm32/cmd/sdlog std_msgs/String '{"data": "{\"start\":true}"}'

# Send FDCAN message
ros2 topic pub /stm32/cmd/fdcan_tx std_msgs/Int32MultiArray "{data: [256, 8, 1, 2, 3, 4, 5, 6, 7, 8]}"

# Check system status
ros2 topic echo /stm32/status
```

## Project Structure

```
STM32H750-board/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── debug_uart.h          # Debug console (USART1)
│   │   ├── ros2_comm.h           # ROS2 UART protocol
│   │   ├── ros2_sensor.h         # ADC sensor reading
│   │   ├── ros2_motor.h          # ESC motor control + flight/manual owner
│   │   ├── fdcan_comm.h          # FDCAN2 communication
│   │   ├── sd_logger.h           # SD card logging
│   │   ├── ota_update.h          # OTA firmware update
│   │   ├── fc_types.h            # Shared flight types
│   │   ├── flight_controller.h   # FC pipeline + health/failsafe
│   │   ├── fc_estimator.h        # Quaternion complementary estimator
│   │   ├── fc_attitude.h         # Attitude control (PX4 cascade)
│   │   ├── fc_rate.h             # Rate control (PX4 P/I/D + FF)
│   │   ├── fc_mixer.h            # Control allocator (CA_* geometry)
│   │   ├── fc_position.h         # Position/velocity control
│   │   ├── fc_land.h             # Land detector
│   │   ├── fc_guard.h            # STAB/RECOVER/teleport guard
│   │   ├── fc_mission.h          # MAVLink-style mission navigator
│   │   ├── fc_sched.h            # Cooperative work-queue scheduler
│   │   ├── fc_orb.h              # uORB-style topic bus
│   │   ├── fc_param.h            # PX4-style param server
│   │   ├── imu_port.h            # IMU port interface
│   │   ├── imu_port_spi.h        # SPI port adapter (2 buses)
│   │   └── imu_icm42688.h        # ICM-42688 driver (SPI polling)
│   └── Src/                      # matching .c files
├── Bootloader/                   # Dual-bank bootloader
├── Drivers/                      # STM32 HAL + CMSIS
├── Middlewares/                   # FreeRTOS
├── ros2_bridge/                  # Raspberry Pi ROS2 package
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── build.sh                  # Auto-source ROS 2 + colcon build
│   ├── setup.sh
│   ├── launch/
│   │   └── bridge.launch.py
│   ├── stm32_bridge/
│   │   ├── __init__.py
│   │   ├── bridge_node.py        # Serial ↔ ROS2 bridge
│   │   └── ota_client.py         # OTA firmware updater
│   └── test_protocol.py          # Protocol unit test (8/8 pass)
├── gazebo_sim/                   # Gazebo Harmonic SIL (firmware-in-the-loop)
│   ├── run_gazebo.sh             # Headless server
│   ├── run_gazebo_gui.sh         # Server + GUI client
│   ├── run_paused.sh             # Paused world
│   ├── worlds/test.world.sdf     # 3D world with obstacles
│   ├── models/x500*/             # Quadrotor model + sensors
│   ├── imu_plugin/               # DART IMU plugin (~1 kHz)
│   └── controller/px4_x500.py    # SIL controller (loads the C firmware via ctypes)
├── renode/                       # Renode simulation
│   ├── stm32h750_board.repl      # Platform description
│   ├── simulate.resc             # Simulation script
│   ├── run.sh                    # Simulation launcher
│   ├── test_sim.py               # Simulation test
│   └── README.md                 # Simulation docs
├── tests/                        # Host tests for the fc_* modules
│   └── test_fc_math.c
├── tools/analyze_flight.py       # Flight-log analysis
├── Makefile
├── STM32H750XX_FLASH.ld         # Linker script
├── STM32H750.ioc                 # CubeMX project
└── PROJECT_LOG.md                # Implementation + verification log
```
