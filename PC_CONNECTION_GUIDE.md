# STM32H750 Flight Controller - PC Connection Guide

## 🔌 Hardware Connection

### USART1 Debug Port (Primary Communication)
- **TX (Output):** PA9
- **RX (Input):** PA10
- **Baudrate:** 115200 bps
- **Format:** 8-N-1 (8 bits, No parity, 1 stop bit)

**USB to Serial Adapter Wiring:**
```
STM32H750      USB-Serial Adapter
─────────────  ──────────────────
PA9 (TX)   →   RX
PA10 (RX)  ←   TX
GND        →   GND
```

⚠️ **IMPORTANT:** Use 3.3V USB-to-Serial adapter, NOT 5V!

---

## 💻 PC Setup

### Option 1: Using PuTTY (Windows/Linux/Mac)
1. **Install PuTTY**: https://www.putty.org/
2. **Connect USB-to-Serial adapter** to PC
3. **Check COM port** (Device Manager on Windows)
4. **Open PuTTY:**
   - Connection type: Serial
   - Serial line: COM3 (or your COM port)
   - Speed: 115200
   - Data bits: 8
   - Stop bits: 1
   - Parity: None
5. **Click Open**

### Option 2: Using TeraTerm (Windows)
1. **Install TeraTerm**: https://ttssh2.osdn.jp/
2. **File** → **New Connection**
3. **Select Serial port** (auto-detected)
4. **Setup** → **Serial port**:
   - Baud rate: 115200
   - Data: 8 bit
   - Parity: None
   - Stop: 1 bit
5. **OK**

### Option 3: Using minicom (Linux)
```bash
sudo minicom -s
# Configure:
# - Serial device: /dev/ttyUSB0 (or /dev/ttyACM0)
# - Speed: 115200
# - Hardware Flow Control: No
# - Save configuration as dfl
```

### Option 4: Using Python Script
```python
import serial
import time

# Configure
PORT = "COM3"          # Windows: COM3, Linux: /dev/ttyUSB0
BAUDRATE = 115200

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Wait for connection
    
    print(f"Connected to {PORT} at {BAUDRATE} bps")
    
    # Read data from STM32
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8', errors='ignore')
            print(data, end='')
        
        # Send command (optional)
        user_input = input().strip()
        if user_input:
            ser.write((user_input + '\r\n').encode())
            
except KeyboardInterrupt:
    print("\nDisconnected")
    ser.close()
except Exception as e:
    print(f"Error: {e}")
```

---

## 📊 What You'll See After Flashing

When you flash the code and reset STM32H750, you should see:

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

## 🚀 Using Debug Output in Your Code

### Example: Print sensor data
```c
#include "debug_uart.h"

void read_imu_task(void *argument)
{
    while(1) {
        // Read IMU via SPI1
        float accel_x = read_imu_x();
        float accel_y = read_imu_y();
        float accel_z = read_imu_z();
        
        // Print via debug UART
        DEBUG_INFO("IMU: X=%.2f Y=%.2f Z=%.2f", accel_x, accel_y, accel_z);
        
        // Print hex dump
        uint8_t imu_data[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
        DEBUG_HEX(imu_data, 6);
        
        osDelay(100);  // 100ms
    }
}
```

### Available Macros
```c
DEBUG_LOG(fmt, ...)    // General log
DEBUG_INFO(fmt, ...)   // Information
DEBUG_WARN(fmt, ...)   // Warning
DEBUG_ERR(fmt, ...)    // Error
DEBUG_HEX(data, size)  // Hex dump
```

### Direct printf Support
```c
#include <stdio.h>
#include "debug_uart.h"

int main(void)
{
    // ...
    
    // Works directly with printf (redirected to UART)
    printf("Hello from STM32H750!\r\n");
    printf("Hex: 0x%02X\r\n", 0xAB);
    
    // ...
}
```

---

## 🔧 Building & Flashing

### Using STM32CubeIDE
1. **Open Project:** `STM32H750.ioc` project folder
2. **Build:** Project → Build All (Ctrl+B)
3. **Flash:** Run → Debug (F11) or Run → Run (Ctrl+F11)
4. **Open debug terminal** (see PC Setup above)
5. **Reset board** - you should see debug output

### Using Makefile
```bash
cd /path/to/STM32H750
make clean
make
make flash
```

---

## ✅ Verify Connection

### Test 1: Hardware Loopback
1. **Connect TX and RX pins together** (PA9 to PA10)
2. **Send data from PC terminal**
3. **You should see your data echoed back**

### Test 2: Check UART Interrupt
1. **Open terminal at 115200 bps**
2. **Reset board**
3. **You should see debug output immediately**

### Test 3: Send Command
```c
// In main or task
if (Debug_Available() > 0)
{
    uint8_t cmd[10];
    uint32_t len = Debug_Receive(cmd, 10);
    DEBUG_INFO("Received %lu bytes: ", len);
    Debug_PrintHex(cmd, len);
}
```

Then type in terminal: `hello` → You'll see hex dump on STM32

---

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| **No output** | Check USB adapter connection, verify COM port in Device Manager |
| **Garbled text** | Wrong baudrate - check it's 115200 |
| **Only random characters** | Voltage mismatch - use 3.3V adapter, not 5V |
| **Missing includes** | Add `#include "debug_uart.h"` to your files |
| **Doesn't compile** | Rebuild project: `make clean && make` |
| **Adapter not recognized** | Install CH340/CP2102 drivers if needed |

---

## 📡 Other Communication Interfaces

You also have:
- **USART2** (PA3 RX, PD5 TX): RC receiver / telemetry
- **UART4** (PD0 RX, PD1 TX): Alternative serial
- **UART7** (PE7 RX, PE8 TX): RS485 mode (sensors)
- **I2C1** (PB8, PB9): External sensors
- **SPI1** (PA5, PB4, PD7, PA15): IMU
- **SPI2** (PB10, PB14, PB15, PB12): Additional sensors
- **USB FS** (PA11, PA12): Mass Storage Device
- **CAN2** (PB5, PB13): Vehicle network

---

## 📋 Next Steps

1. ✅ Build and flash firmware
2. ✅ Verify debug UART connection
3. ✅ Add sensor reading tasks (IMU, barometer, compass)
4. ✅ Implement flight control algorithm
5. ✅ Test motor outputs (PWM on PA0, PA1, PA2, PB11)
6. ✅ Calibrate sensors
7. ✅ Tune PID controllers

---

## 📞 Support

If you encounter issues:
1. Check `.ioc` file configuration (all pins assigned correctly)
2. Verify generated code compiles without errors
3. Check `debug_uart.c` callbacks are called from interrupt handlers
4. See troubleshooting section above

---

**Good luck with your flight controller! 🚁**
