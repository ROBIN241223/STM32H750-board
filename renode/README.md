# STM32H750 Renode Simulation

## Cấu trúc file

| File | Mục đích |
|---|---|
| `stm32h750_board.repl` | Platform description - định nghĩa các peripheral giả lập (PWR, DBGMCU, QSPI 8MB) |
| `simulate.resc` | Script chạy simulation thủ công (có showAnalyzer) |
| `run.sh` | Wrapper dọn snap env trước khi chạy Renode |
| `test_sim.py` | Test script tự động (load ELF, chạy 10ms, đọc thanh ghi USART) |

## Cách chạy

```bash
# Build firmware (từ thư mục gốc project)
make -j4

# Chạy simulation thủ công (có console UART)
./renode/run.sh

# Chạy simulation test (đọc USART register sau 1ms)
env -i PATH="/usr/bin:/bin:/usr/local/bin" HOME="$HOME" \
  LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu" \
  dotnet /opt/renode/bin/Renode.dll -e "
mach create
machine LoadPlatformDescription @renode/stm32h750_board.repl
sysbus WriteDoubleWord 0x58024804 0x00002000
sysbus WriteDoubleWord 0x5802480C 0x00002000
sysbus WriteDoubleWord 0x58024400 0x3FFE0000
sysbus WriteDoubleWord 0x5C001000 0x20006470
sysbus LoadELF @build/STM32H750.elf
cpu VectorTableOffset 0x90000000
emulation RunFor \"0.001\"
sysbus ReadDoubleWord 0x40011000
sysbus ReadDoubleWord 0x4001101C
quit
"
```

## Register pre-set (bỏ qua polling trong SystemClock_Config)

| Address | Register | Giá trị | Tác dụng |
|---|---|---|---|
| `0x58024804` | PWR CSR1 | `0x2000` | VOSRDY=1 (bỏ qua poll voltage) |
| `0x5802480C` | PWR D3CR | `0x2000` | VOSRDY=1 |
| `0x58024400` | RCC CR | `0x3FFE0000` | HSERDY, PLLRDY bits (bỏ qua poll clock) |
| `0x5C001000` | DBGMCU IDCODE | `0x20006470` | Chip ID cho STM32H750 |

## Check UART

Sau khi chạy simulation, đọc register:

```bash
# USART1 (debug) CR1: 0x40011000 - nếu bit 0=1 là đã enable
# USART2 (ROS2) CR1: 0x40004400 - nếu bit 0=1 là đã enable
# USART1 TDR (dữ liệu TX gần nhất): 0x40011028
```

## Protocol test

```bash
python3 ros2_bridge/test_protocol.py
```

## Hạn chế

- Simulation rất chậm (~2-3 phút cho 1ms mô phỏng) do thermal throttling CPU
- Firmware cần ~3-5ms simulated time để tới UART init
- Chưa capture UART output ra file (thiếu `UartFileLog` backend trong Renode)
- Chưa implement bỏ qua SystemClock_Config bằng cách set PC thẳng tới main

## Cải thiện tốc độ

Cách nhanh nhất: bỏ qua SystemClock_Config bằng cách set PC thẳng tới `main()`:

```
cpu PC 0x90002c78
```

Nhưng cần pre-set `SystemCoreClock` và các RCC register liên quan.
