# 🎯 QUICK SYNC GUIDE - CubeMX đồng bộ Chi Tiết Nhất

## 🚀 TL;DR (Nhanh Gọn)

Nếu bạn chỉ có 10 phút:

```
1. Mở STM32H750.ioc bằng CubeMX
2. Kiểm tra: SPI1 Prescaler = /4 (16MHz) ✓
3. Project → Generate Code (Alt+K)
4. Chạy: restore_debug.ps1 hoặc chỉnh tay main.c
5. Build: make clean && make
6. Flash: make flash
7. Terminal: 115200 bps
```

---

## 📋 DETAILED CHECKLIST - Từng Bước

### ✅ PHASE 1: Backup & Prepare (5 phút)

#### Step 1a: Backup các file quan trọng
```powershell
# PowerShell (Windows)
Copy-Item "Core\Src\main.c" "Core\Src\main.c.backup"
Copy-Item "Core\Src\stm32h7xx_it.c" "Core\Src\stm32h7xx_it.c.backup"
Copy-Item "Core\Inc\FreeRTOSConfig.h" "Core\Inc\FreeRTOSConfig.h.backup"
```

#### Step 1b: Verify các file debug có tồn tại
```
Kiểm tra các file này có trong project:
☐ Core/Inc/debug_uart.h
☐ Core/Src/debug_uart.c
☐ Core/Src/debug_example.c

Nếu không có → Tôi đã tạo trong bước trước
```

---

### ✅ PHASE 2: Verify Configuration trong CubeMX (10 phút)

#### Step 2a: Mở CubeMX với .ioc file

```
CÁCH 1 - Từ File Explorer:
  Double-click: STM32H750.ioc
  → Tự động mở CubeMX

CÁCH 2 - Từ CubeMX:
  File → Open Project
  → Select: C:\Users\ducnt\Documents\STM32H750\STM32H750.ioc
  → Click Open

CÁCH 3 - Từ STM32CubeIDE:
  Project Explorer
  → Right-click project
  → Open With → STM32CubeMX
```

**Sau 1-2 phút, CubeMX sẽ hiện project (có thể hỏi "Save modifications?"→ Chọn Yes/OK)**

---

#### Step 2b: Verify SPI1 Speed = 16MHz ⚡

```
CubeMX GUI:
├─ Left: "Connectivity" → Expand (click ▼)
│  └─ "SPI1" ← Click vào
└─ Right: Configuration panel sẽ hiện

Expected Output:
┌─────────────────────────────────┐
│ SPI1 Configuration              │
├─────────────────────────────────┤
│ Mode: Full-Duplex Master        │
│ Prescaler: /4 ← MUST BE THIS!   │
│ BaudRate: 16.0 MBits/s ← Check  │
│ Data Size: 8 bits               │
│ CRC: Disabled                   │
│ NSS: Hard Output (PA15)         │
└─────────────────────────────────┘
```

**❌ Nếu Prescaler vẫn là "/2" (32MHz):**
```
1. Click dropdown "Prescaler"
2. Select "/4"
3. Verify "BaudRate" thay đổi thành 16.0 MBits/s
4. (Tự động save vào .ioc)
```

---

#### Step 2c: Verify TIM2 PWM = 20kHz ⚡

```
CubeMX GUI:
├─ Left: "Timers" → Expand
│  └─ "TIM2" ← Click vào
└─ Right: Configuration panel

Expected Output:
┌─────────────────────────────────┐
│ TIM2 Configuration              │
├─────────────────────────────────┤
│ Prescaler: 5 (tức 6-1)          │
│ Period: 999 (tức 1000-1)        │
│ Frequency: 20 kHz ← Check này!  │
│ Auto-Reload: Enabled            │
│                                 │
│ PWM Channels:                   │
│ ✓ CH1: PA0 (PWM Gen)            │
│ ✓ CH2: PA1 (PWM Gen)            │
│ ✓ CH3: PA2 (PWM Gen)            │
│ ✓ CH4: PB11 (PWM Gen)           │
└─────────────────────────────────┘
```

✅ Nếu đúng như trên → Tiếp tục

---

#### Step 2d: Verify I2C1 Pins ⚡

```
CubeMX GUI:
├─ Left: "Connectivity" → "I2C1"
└─ Right: Configuration panel

Expected:
┌─────────────────────────────────┐
│ I2C1 Configuration              │
├─────────────────────────────────┤
│ Timing: 0x307075B1 (400kHz)     │
│ Mode: I2C                       │
│ Address Mode: 7-bit             │
│                                 │
│ Pins:                           │
│ ✓ SCL: PB8                      │
│ ✓ SDA: PB9                      │
└─────────────────────────────────┘
```

---

#### Step 2e: Verify USART1 Debug Port ⚡

```
CubeMX GUI:
├─ Left: "Connectivity" → "USART1"
└─ Right: Configuration

Expected:
┌─────────────────────────────────┐
│ USART1 Configuration            │
├─────────────────────────────────┤
│ Mode: Asynchronous              │
│ Baud Rate: 115200               │
│ Word Length: 8 Bits             │
│ Stop Bits: 1                    │
│ Parity: None                    │
│                                 │
│ Pins:                           │
│ ✓ TX: PA9                       │
│ ✓ RX: PA10                      │
└─────────────────────────────────┘
```

---

#### Step 2f: Verify Debug Pins (SWD + SWO) ⚡

```
CubeMX GUI:
├─ Left: "System Core" → "SYS"
├─ Right: "Debug" tab
└─ Click "Trace Asynchronous SWO"

Expected Pins:
✓ PA13: SWDIO
✓ PA14: SWCLK
✓ PB3: SWO (Trace output)
```

---

### ✅ PHASE 3: Generate Code (5 phút)

#### Step 3a: Regenerate Code

```
STM32CubeMX Menu:
  Project → Generate Code
  
Hoặc nhấn: Alt+K

Dialog sẽ hiện:
┌─────────────────────────┐
│ ⏳ Generating Code...   │
│                         │
│ ✓ Cleanup              │
│ ✓ Generating MSP       │
│ ✓ Generating HAL       │
│ ✓ Generation Complete  │
│                         │
│        [OK]             │
└─────────────────────────┘
```

**⏱️ Chờ 30-60 giây cho tới khi thấy "Generation Complete"**

---

#### Step 3b: Kiểm tra có error không?

```
Nếu xuất hiện lỗi:
├─ Click [Details]
├─ Đọc error message
└─ Giải quyết (thường do missing toolchain)

Nếu thành công:
└─ Close dialog, tiếp tục Step 4
```

---

### ✅ PHASE 4: Restore Debug UART (3-5 phút)

**CubeMX sẽ xóa những gì tôi thêm vào main.c, phải restore lại!**

#### Cách A: Tự động (PowerShell Script)

```powershell
# Chạy script (Windows PowerShell)
.\restore_debug.ps1

# Hoặc từ Git Bash/Linux:
pwsh restore_debug.ps1
```

**Output:**
```
============================================================
  STM32H750 - Restore Debug UART
============================================================

Step 1: Checking for debug_uart.h include...
        ✓ Include added successfully!

Step 2: Checking for Debug_UART_Init()...
        ✓ Debug initialization added successfully!

Step 3: Checking FreeRTOSConfig.h...
        ✓ FPU enabled successfully!

Step 4: Verifying all fixes...
✓ Include debug_uart.h
✓ Debug_UART_Init() call
✓ Debug_Test() call
✓ FPU enabled

============================================================
  ✓ All Restorations Complete!
============================================================
```

---

#### Cách B: Manual (Tay)

**Nếu script không chạy được, làm tay:**

##### B1: Thêm include debug_uart.h

**File:** `Core/Src/main.c` (Line ~26)

**Tìm:**
```c
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
```

**Sửa thành:**
```c
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug_uart.h"
/* USER CODE END Includes */
```

**Cách sửa trong STM32CubeIDE:**
```
1. Open Core/Src/main.c
2. Ctrl+G → Go to Line 26
3. Position cursor sau "/* USER CODE BEGIN Includes */"
4. Press Enter
5. Type: #include "debug_uart.h"
6. Ctrl+S to save
```

---

##### B2: Thêm Debug_UART_Init() call

**File:** `Core/Src/main.c` (Line ~168)

**Tìm:**
```c
   MX_TIM2_Init();
   /* USER CODE BEGIN 2 */

   /* USER CODE END 2 */
```

**Sửa thành:**
```c
   MX_TIM2_Init();
   /* USER CODE BEGIN 2 */
   
   /* Initialize Debug UART for PC communication */
   Debug_UART_Init();
   Debug_Test();
   
   /* USER CODE END 2 */
```

**Cách sửa:**
```
1. Open Core/Src/main.c
2. Ctrl+F (Find)
3. Search: "MX_TIM2_Init();"
4. Thêm dòng sau đó:
   - Debug_UART_Init();
   - Debug_Test();
5. Ctrl+S
```

---

##### B3: Enable FPU

**File:** `Core/Inc/FreeRTOSConfig.h` (Line 59)

**Tìm:**
```c
#define configENABLE_FPU                         0
```

**Sửa thành:**
```c
#define configENABLE_FPU                         1
```

**Cách sửa:**
```
1. Open Core/Inc/FreeRTOSConfig.h
2. Ctrl+G → Line 59
3. Change "0" to "1"
4. Ctrl+S
```

---

### ✅ PHASE 5: Build & Verify (5 phút)

#### Step 5a: Build Project

```powershell
# Cách 1: Makefile
make clean
make

# Cách 2: STM32CubeIDE
Ctrl+B  (Build All)
```

**Expected Output:**
```
make[1]: Entering directory '...'

Compiling file: Core/Src/main.c
Compiling file: Core/Src/debug_uart.c
Compiling file: Core/Src/stm32h7xx_hal_msp.c
Compiling file: Core/Src/stm32h7xx_it.c
Assembling file: startup_stm32h750xx.s
Linking...

Finished successfully

Build time: 45 seconds
```

**❌ Nếu có error:**
```
Typical errors:
├─ "undefined reference to 'Debug_UART_Init'"
│  → Quay lại B1: add #include "debug_uart.h"
├─ "'debug_uart.h': No such file or directory"
│  → Verify file exists: Core/Inc/debug_uart.h
└─ Compilation error in debug_uart.c
   → Check syntax in Core/Src/debug_uart.c
```

---

#### Step 5b: Flash to Board

```powershell
# Cách 1: Makefile
make flash

# Cách 2: STM32CubeIDE
F11 (Debug)  hoặc  Ctrl+F11 (Run)

# Cách 3: STM32 Programmer
- Open STM32CubeProgrammer
- Select ST-Link or UART
- Load: build/STM32H750.elf
- Click [Download]
```

---

#### Step 5c: Test Output

```
1. Mở terminal software (PuTTY, TeraTerm, minicom)
2. Select COM port (nếu dùng USB-Serial adapter)
3. Set 115200 8N1
4. Click Open
5. RESET the STM32H750 board (press RESET button)
6. Bạn sẽ thấy:

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

✅ **Nếu thấy output → ALL FIXES APPLIED SUCCESSFULLY!**

---

## 🔍 Final Verification Checklist

```
STM32H750.ioc (Open with Text Editor):
☐ Line 558: SPI1.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_4
☐ Line 560: SPI1.CalculateBaudRate=16.0 MBits/s

Core/Inc/FreeRTOSConfig.h:
☐ Line 59: #define configENABLE_FPU 1

Core/Src/main.c:
☐ Include: #include "debug_uart.h"
☐ Call: Debug_UART_Init();
☐ Call: Debug_Test();

Build:
☐ make clean && make → No errors
☐ make flash → Uploaded successfully

Terminal @ 115200:
☐ See debug output after board reset
```

---

## 🚀 Summary Flow Chart

```
START
  ↓
[1] Backup files
  ↓
[2] Mở CubeMX với STM32H750.ioc
  ↓
[3] Verify: SPI1=16MHz, TIM2=20kHz, etc
  ↓
[4] Project → Generate Code (Alt+K)
  ↓
[5] Chạy restore_debug.ps1
     (hoặc manual edit main.c)
  ↓
[6] make clean && make
  ↓
[7] make flash
  ↓
[8] Terminal 115200 bps
  ↓
[9] Reset board → See output ✅
  ↓
END (Ready to develop!)
```

---

## ⏱️ Total Time Estimate

```
Backup:              ~2 min
Verify in CubeMX:    ~5 min
Generate Code:       ~2 min
Restore Debug:       ~3 min
Build:               ~1 min
Flash:               ~1 min
Test:                ~1 min
─────────────────────────
Total:               ~15 minutes
```

---

## 📞 If Something Goes Wrong

| Problem | Solution |
|---------|----------|
| CubeMX won't open .ioc | Install latest CubeMX v6.x |
| "Can't generate code" | Close project, delete .mxproject folder, reopen |
| Build errors after generate | Run restore_debug.ps1 again |
| "Undefined reference" | Check #include "debug_uart.h" is there |
| No terminal output | Verify 115200 bps, check COM port |
| SPI1 still 32MHz | Manual fix .ioc file (notepad), SPI1.BaudRatePrescaler line |

---

**✅ Now you're ready to sync with CubeMX!**
