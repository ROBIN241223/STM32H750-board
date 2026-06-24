# STM32H750 Flight Controller - Configuration & Code Fixes Summary

**Date:** June 24, 2026  
**Project:** STM32H750VBTx Custom Drone Flight Controller  
**Status:** ✅ All Critical Fixes Applied

---

## 📋 FIXES APPLIED

### ✅ FIX 1: Enable FPU Support (CRITICAL)
**File:** `Core/Inc/FreeRTOSConfig.h` (Line 59)

```c
// BEFORE (WRONG):
#define configENABLE_FPU 0

// AFTER (CORRECT):
#define configENABLE_FPU 1
```

**Why:** STM32H7 has built-in Floating Point Unit (FPU). Disabling it causes:
- Inefficient floating-point math in flight controller
- Potential context switch errors
- Performance loss in sensor processing

**Impact:** ✅ Full FPU support, better performance, safer context switching

---

### ✅ FIX 2: Reduce SPI1 Clock Speed (RECOMMENDED)
**File:** `STM32H750.ioc` (Lines 558-560)

```
// BEFORE:
SPI1.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2  → 32 MHz
SPI1.CalculateBaudRate = 32.0 MBits/s

// AFTER:
SPI1.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4  → 16 MHz
SPI1.CalculateBaudRate = 16.0 MBits/s
```

**Why:** Most IMU sensors (BMI088, ICM-20948) max at 10-20 MHz:
- 32 MHz can cause timing issues
- 16 MHz is safer and more compatible
- Still plenty fast for real-time sensor fusion

**Pins:** PA5(SCK), PB4(MISO), PD7(MOSI), PA15(NSS)

**Impact:** ✅ Better IMU compatibility, stable SPI communication

---

### ✅ FIX 3: Added Debug UART for PC Communication

**New Files Created:**

#### 1. `Core/Inc/debug_uart.h` - Debug UART Header
- Printf-style formatted output: `Debug_Printf()`
- Raw data send/receive: `Debug_Send()`, `Debug_Receive()`
- Hex dump support: `Debug_PrintHex()`
- Convenient macros: `DEBUG_LOG()`, `DEBUG_INFO()`, `DEBUG_WARN()`, `DEBUG_ERR()`
- **Interface:** USART1 (PA9=TX, PA10=RX @ 115200 bps)

#### 2. `Core/Src/debug_uart.c` - Debug UART Implementation
- Ring buffer for RX (256 bytes)
- Interrupt-driven reception
- HAL callbacks for UART events
- libc integration (printf redirection)
- Full error handling

#### 3. `Core/Src/debug_example.c` - Example Tasks
- System status monitoring
- Sensor data logging
- Motor control examples
- Command processing from PC
- Performance monitoring

#### 4. `PC_CONNECTION_GUIDE.md` - Complete Documentation
- Hardware setup (USB adapter wiring)
- PC terminal software (PuTTY, TeraTerm, minicom, Python)
- Expected output after flashing
- Troubleshooting guide
- Usage examples

---

### ✅ FIX 4: Integrated Debug UART into Main
**File:** `Core/Src/main.c`

```c
// Added include:
#include "debug_uart.h"

// Added initialization in main():
Debug_UART_Init();      // Initialize UART interface
Debug_Test();           // Print test messages
```

**Features:**
- Automatic startup messages
- Real-time system info printing
- FreeRTOS heap monitoring
- Task communication interface

---

## 🔧 CONFIGURATION SUMMARY

### System Clock
- **SYSCLK:** 480 MHz (PLL from 25 MHz HSE)
- **AHB (HCLK):** 240 MHz
- **APB1/2/3/4:** 120 MHz
- **Supply:** LDO, Scale 0 (maximum performance)

### Memory
- **Flash:** 128 KB (code)
- **SRAM:** 1 MB (runtime data)
- **External Flash:** QUADSPI (up to 512 MB)
- **SD Card:** SDMMC1 4-bit mode

### FreeRTOS
- **Heap:** 128 KB (for tasks, queues, etc.)
- **Tick Rate:** 1000 Hz (1 ms)
- **FPU:** ✅ ENABLED
- **Scheduler:** Preemptive

### Peripheral Configuration

| Peripheral | Pins | Config | Status |
|------------|------|--------|--------|
| **PWM (Motors)** | PA0-2, PB11 | TIM2, 20kHz, 1000µs | ✅ Ready |
| **IMU SPI1** | PA5, PB4, PD7, PA15 | 16MHz ↓ (was 32) | ✅ Fixed |
| **SPI2** | PB10, PB14-15, PB12 | 32MHz, NSS_HW | ✅ OK |
| **SPI4** | PE12-14 | 60MHz | ✅ OK |
| **I2C1** | PB8-9 | 400kHz standard | ✅ OK |
| **USART1** | PA9-10 | **115200 - DEBUG** | ✅ Active |
| **USART2** | PA3, PD5 | 115200 - RC/Telemetry | ✅ OK |
| **UART4** | PD0-1 | 115200 | ✅ OK |
| **UART7** | PE7-8 | 115200 RS485 | ✅ OK |
| **FDCAN2** | PB5, PB13 | 1333 kbaud | ✅ OK |
| **USB FS** | PA11-12 | Mass Storage Device | ✅ OK |
| **DCMI** | 8 data pins | Camera interface | ✅ OK |
| **SDMMC1** | PC8-12, PD2 | 4-bit SD card | ✅ OK |
| **Debug** | PA13-14, PB3 | SWD + SWO tracing | ✅ OK |

---

## 📡 QUICK START - PC COMMUNICATION

### Hardware Required
- STM32H750 development board
- **USB-to-Serial adapter (3.3V)** ← **IMPORTANT: 3.3V, NOT 5V!**
- Micro-USB cable

### Wiring
```
STM32H750      USB-Serial
─────────────  ──────────────
PA9 (USART1_TX) → RX
PA10 (USART1_RX) ← TX
GND → GND
```

### PC Terminal Setup
1. Connect adapter to PC
2. Identify COM port (Device Manager)
3. Open terminal at **115200 8N1**:
   - **PuTTY:** `putty.exe -serial COM3 -sercfg 115200,8,1,N`
   - **Linux:** `minicom -s` then configure
   - **Python:** See `PC_CONNECTION_GUIDE.md`

### Expected Output
```
=== STM32H750 Flight Controller Debug UART Initialized ===
[INF] USART1: 115200 8N1
[INF] TX: PA9, RX: PA10
=== Debug UART Test ===
[INF] System Clock: 480 MHz
[INF] FreeRTOS Heap: 131072 bytes
[INF] Tick Rate: 1000 Hz
[WRN] This is a warning message
[ERR] This is an error message
[INF] Hex dump test:
01 02 03 04 05 06 07 08
=== Test Complete ===
```

---

## 🚀 NEXT STEPS

### 1. Regenerate Code from .ioc
```bash
# In STM32CubeIDE:
# Right-click project → Generate Code
# OR via Makefile
make regenerate  # if applicable
```

### 2. Add Debug Headers
```c
#include "debug_uart.h"

// In your tasks:
DEBUG_INFO("Sensor reading: %.2f", sensor_value);
DEBUG_HEX(raw_data, 6);
```

### 3. Build & Flash
```bash
make clean
make
make flash

# OR use STM32CubeIDE:
# Build All (Ctrl+B)
# Debug (F11) or Run (Ctrl+F11)
```

### 4. Verify Connection
- Open terminal at 115200 bps
- Reset board
- You should see debug output!

### 5. Next Development
- [ ] Implement IMU reading task on SPI1
- [ ] Implement altitude control on I2C barometer
- [ ] Test motor PWM outputs
- [ ] Implement RC receiver on USART2
- [ ] Build flight control loop (PID)
- [ ] Implement telemetry logging

---

## 📚 File Locations

| File | Purpose |
|------|---------|
| `STM32H750.ioc` | CubeMX project (SPI1 speed reduced) |
| `Core/Inc/FreeRTOSConfig.h` | FreeRTOS config (FPU enabled) |
| `Core/Inc/debug_uart.h` | Debug UART header |
| `Core/Src/debug_uart.c` | Debug UART implementation |
| `Core/Src/debug_example.c` | Example tasks (optional) |
| `Core/Src/main.c` | Main code (debug init added) |
| `PC_CONNECTION_GUIDE.md` | PC setup guide |

---

## ⚠️ IMPORTANT NOTES

1. **FPU Must Stay Enabled** - Don't disable in FreeRTOSConfig.h
2. **3.3V Only** - USB adapter must be 3.3V logic level
3. **Regenerate After .ioc Changes** - If you modify .ioc, regenerate code
4. **Debug Output Goes to USART1** - Not suitable for real-time-critical code
5. **SPI1 Now 16MHz** - Check IMU datasheet if different speed needed

---

## 🔍 Verification Checklist

- [ ] FreeRTOSConfig.h: `configENABLE_FPU = 1`
- [ ] STM32H750.ioc: SPI1 BaudRate = 16MHz (prescaler=4)
- [ ] debug_uart.h and .c files present
- [ ] main.c includes debug_uart.h
- [ ] main.c calls Debug_UART_Init() and Debug_Test()
- [ ] Build succeeds without errors
- [ ] Flash to STM32H750
- [ ] See debug output on terminal

---

## 📞 Troubleshooting

**Q: No output on terminal?**  
A: 
- Check COM port is correct (Device Manager)
- Check baudrate is 115200 8N1
- Check USB adapter power (should light up LED)
- Try different terminal software

**Q: Garbled text?**  
A: Wrong baudrate - confirm 115200 in terminal settings

**Q: Compilation errors?**  
A: 
- Make sure debug_uart.h and .c are in Core folders
- Run `make clean && make` to rebuild
- Check includes in main.c

**Q: Keeps printing same message?**  
A: Normal - Debug_Test() is called in main(). Remove if unwanted.

---

**✅ Configuration Complete - Ready for Development!**
