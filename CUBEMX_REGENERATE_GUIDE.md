# 📖 Hướng Dẫn Đồng Bộ Code với CubeMX - Chi Tiết Từng Bước

**Mục đích:** Regenerate code trong STM32CubeMX để đồng bộ các fix tôi đã thực hiện

---

## ⚠️ QUAN TRỌNG - Đọc Trước Khi Làm!

### ❌ CÓ THỂ MẤT FILE
Khi regenerate code, CubeMX sẽ **overwrite** các file này:
- `Core/Src/main.c` - ⚠️ CHỨA fix debug_uart.h include
- `Core/Src/stm32h7xx_hal_msp.c`
- `Core/Src/stm32h7xx_it.c`
- `Core/Src/system_stm32h7xx.c`

### ✅ KHÔNG BỊ MẤT
Những file này an toàn:
- `Core/Inc/debug_uart.h` ✅ (không thuộc CubeMX)
- `Core/Src/debug_uart.c` ✅ (không thuộc CubeMX)
- `Core/Src/debug_example.c` ✅ (không thuộc CubeMX)
- Tất cả file trong `Drivers/` ✅

---

## 🔧 BƯỚC 1: Backup Các File Quan Trọng

Trước khi regenerate, **BACKUP** những gì sẽ bị overwrite:

### Windows PowerShell
```powershell
# Copy main.c với tên backup
Copy-Item "C:\Users\ducnt\Documents\STM32H750\Core\Src\main.c" `
          "C:\Users\ducnt\Documents\STM32H750\Core\Src\main.c.backup"

Copy-Item "C:\Users\ducnt\Documents\STM32H750\Core\Src\stm32h7xx_it.c" `
          "C:\Users\ducnt\Documents\STM32H750\Core\Src\stm32h7xx_it.c.backup"
```

### Linux/Mac
```bash
cd /path/to/STM32H750
cp Core/Src/main.c Core/Src/main.c.backup
cp Core/Src/stm32h7xx_it.c Core/Src/stm32h7xx_it.c.backup
cp Core/Src/stm32h7xx_hal_msp.c Core/Src/stm32h7xx_hal_msp.c.backup
```

---

## 📂 BƯỚC 2: Mở .ioc File trong STM32CubeMX

### Cách 1: Từ CubeMX
```
File → Open Project
→ Chọn: C:\Users\ducnt\Documents\STM32H750\STM32H750.ioc
→ Click "Open"
```

### Cách 2: Từ File Explorer
```
Double-click: STM32H750.ioc
→ Tự động mở trong STM32CubeMX
```

### Cách 3: Từ STM32CubeIDE
```
Project Explorer
→ Right-click project folder
→ Open With → STM32CubeMX
```

---

## 🔍 BƯỚC 3: Xác Minh Các Fix Đã Được Áp Dụng

### ✅ Check 1: SPI1 Speed = 16MHz

**Vị trí:** Pinout & Configuration tab → Connectivity → SPI1

```
Trong CubeMX GUI:
1. Click "Connectivity" expand menu trên trái
2. Click "SPI1"
3. Xem "Configuration" panel bên phải
4. Tìm "BaudRate Prescaler": phải là "Prescaler_4" (16 MHz)
```

**Expected:**
```
SPI1:
├── Mode: Full-Duplex Master
├── Prescaler: /4  ← CỤC QUAN TRỌNG (là 32MHz thì sửa lại!)
├── Data Size: 8 bits
├── NSS Signal: Hard Output ← PA15
└── CRC: Disabled
```

**Nếu còn là Prescaler_2 (32MHz):**
```
1. Click dropdown "Prescaler" 
2. Select "Prescaler_4"
3. Verify "CalculateBaudRate" = 16.0 MBits/s
```

---

### ✅ Check 2: Xác Nhận Các Pin Đúng

**SPI1 Pins (xem Pinout tab):**
```
Tìm trong bảng Pin Configuration:
- PA5: SPI1_SCK ✅ Full_Duplex_Master, GPIO_SPEED_VERY_HIGH
- PB4: SPI1_MISO ✅ Full_Duplex_Master
- PD7: SPI1_MOSI ✅ Full_Duplex_Master
- PA15: SPI1_NSS ✅ NSS_Signal_Hard_Output, VERY_HIGH
```

**I2C1 Pins:**
```
- PB8: I2C1_SCL ✅ I2C mode
- PB9: I2C1_SDA ✅ I2C mode
```

**TIM2 PWM (Motor):**
```
- PA0: TIM2_CH1 ✅ PWM Generation
- PA1: TIM2_CH2 ✅ PWM Generation
- PA2: TIM2_CH3 ✅ PWM Generation
- PB11: TIM2_CH4 ✅ PWM Generation
```

---

### ✅ Check 3: TIM2 Frequency = 20kHz

**Vị trị:** Timers → TIM2

```
CubeMX:
1. Click "Timers" → expand
2. Click "TIM2"
3. Check "Configuration" panel:
   - Prescaler: 5 (là 6-1)
   - Period: 999 (là 1000-1)
   - → Tính = 120MHz / 6 / 1000 = 20kHz ✅
```

---

### ✅ Check 4: Debug Pins (SWD + SWO)

**Vị trí:** Debug tab

```
System Core → SYS → Debug → Mode: Trace Asynchronous SWO
```

**Expected pins:**
```
- PA13: DEBUG_JTMS-SWDIO ✅
- PA14: DEBUG_JTCK-SWCLK ✅
- PB3: DEBUG_JTDO-SWO ✅ (Trace_Asynchronous_SW)
```

---

## ⚙️ BƯỚC 4: Regenerate Code

### Cách 1: Menu CubeMX

```
STM32CubeMX Menu
↓
Project → Generate Code (Alt+K)
↓
Chờ cho tới khi hoàn thành
```

### Cách 2: Toolbar Button
```
Click icon: ⚙️ (Generate Code)
hoặc nhấn Alt+K
```

**Dialog sẽ hiện lên:**
```
┌─────────────────────────────────┐
│ Generate Code for STM32H750.ioc │
├─────────────────────────────────┤
│                                 │
│ ✓ Regenerating STM32CubeMX      │
│   project...                    │
│                                 │
│ ✓ Compiling STM32CubeMX project│
│   for toolchain verification... │
│                                 │
│ ✓ Generation finished           │
│                                 │
│              [OK]               │
└─────────────────────────────────┘
```

**Nếu có lỗi, click [Details] để xem:**
```
Có thể là do project path chứa space
hoặc toolchain không cài đầy đủ
```

---

## 📝 BƯỚC 5: Restore Debug UART Initialization

CubeMX sẽ **xóa** dòng include và debug init mà tôi thêm vào `main.c`.

**Phải restore lại:**

### File: `Core/Src/main.c` (Line ~26)

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

---

### File: `Core/Src/main.c` (Line ~169)

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

---

## 🔧 BƯỚC 6: Build & Verify

### Build Project
```bash
# Cách 1: STM32CubeIDE
Project → Build All
(Hoặc Ctrl+B)

# Cách 2: Makefile
cd C:\Users\ducnt\Documents\STM32H750
make clean
make
```

**Expected output:**
```
make[1]: Entering directory '...'
Compiling file: Core/Src/main.c
Compiling file: Core/Src/debug_uart.c
Compiling file: Core/Src/stm32h7xx_hal_msp.c
...
Build succeeded
```

---

## ✅ BƯỚC 7: Xác Nhận Tất Cả Fix Được Áp Dụng

### Check File `.ioc` (Text Editor)

**Mở:** `STM32H750.ioc` với Notepad++

**Tìm dòng:**
```
SPI1.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_4
```

✅ Nếu thấy dòng này → OK

❌ Nếu vẫn là:
```
SPI1.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_2
```

→ **Phải sửa lại trong CubeMX!**

---

### Check File `FreeRTOSConfig.h`

**Mở:** `Core/Inc/FreeRTOSConfig.h`

**Tìm dòng 59:**
```c
#define configENABLE_FPU                         1
```

✅ Nếu là `1` → OK
❌ Nếu vẫn là `0` → Sửa lại

---

### Check File `main.c` 

**Tìm dòng với include:**
```c
#include "debug_uart.h"
```

✅ Có → OK
❌ Không có → Thêm lại theo Bước 5

---

## 🚀 BƯỚC 8: Flash & Test

```bash
# Flash to STM32H750
STM32CubeIDE: Run → Debug (F11)

Hoặc từ Makefile:
make flash

Hoặc từ STM32 Programmer Tool
```

**Kết quả:**
```
Terminal @ 115200 bps:

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

✅ Nếu thấy output → **All fixes applied successfully!**

---

## 📊 FULL CHECKLIST - Regenerate Code

```
Trước Regenerate:
☐ Backup Core/Src/main.c
☐ Backup Core/Src/stm32h7xx_it.c
☐ Mở STM32H750.ioc trong CubeMX

Xác Minh Fix:
☐ SPI1 Prescaler = 4 (16MHz)
☐ TIM2 Period = 999, Prescaler = 5 (20kHz)
☐ I2C1, SPI2, SPI4 pins OK
☐ Debug pins: PA13, PA14, PB3 OK

Regenerate:
☐ Project → Generate Code (Alt+K)
☐ Chờ xong (30-60 giây)
☐ Không có error

Restore Debug:
☐ main.c: Add #include "debug_uart.h"
☐ main.c: Add Debug_UART_Init() & Debug_Test()
☐ FreeRTOSConfig.h: configENABLE_FPU = 1

Build & Test:
☐ Build All (Ctrl+B) → No errors
☐ Flash to board
☐ Open terminal 115200 bps
☐ Reset board
☐ See debug output ✅
```

---

## 🐛 TROUBLESHOOTING - Nếu Có Vấn đề

### ❌ Vấn đề 1: CubeMX không generate được

**Giải pháp:**
```
1. Đóng tất cả CubeMX instances
2. Xóa thư mục: STM32H750/.mxproject
3. Mở lại STM32H750.ioc
4. Try: Project → Generate Code
```

### ❌ Vấn đề 2: Build error sau regenerate

**Giải pháp:**
```
Nếu lỗi như "undefined reference to 'Debug_UART_Init'":
→ Bạn quên thêm #include "debug_uart.h"
→ Quay lại Bước 5: Restore Debug UART Initialization
```

### ❌ Vấn đề 3: SPI1 speed vẫn 32MHz

**Giải pháp:**
```
1. Đóng CubeMX
2. Mở STM32H750.ioc với text editor
3. Tìm: SPI1.BaudRatePrescaler=
4. Sửa: SPI1.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_4
5. Save file
6. Mở lại trong CubeMX
7. Verify rồi Generate Code
```

---

## 💾 Tóm Tắt Các File Cần Bảo Vệ

### ✅ SAFE - Không bị xóa khi regenerate
```
✓ Core/Inc/debug_uart.h
✓ Core/Src/debug_uart.c
✓ Core/Src/debug_example.c
✓ PC_CONNECTION_GUIDE.md
✓ FIXES_AND_SUMMARY.md
```

### ⚠️ CAREFUL - Sẽ bị overwrite
```
! Core/Src/main.c (thêm debug init lại)
! Core/Src/stm32h7xx_it.c
! Core/Src/stm32h7xx_hal_msp.c
! Core/Src/system_stm32h7xx.c
! Tất cả .h files trong Core/Inc
```

---

## 📸 Visual Guide - CubeMX Interface

### Timers Tab (xem TIM2)
```
┌──────────────────────────────────────────┐
│ CubeMX - Timers Configuration            │
├──────────────────────────────────────────┤
│                                          │
│ Left Panel:        │ Configuration Panel:│
│ • Connectivity    │ TIM2:               │
│ • Timers ▼        │ ├─ Prescaler: 5     │
│   • TIM1          │ ├─ Period: 999      │
│   • TIM2 ← HERE   │ ├─ Frequency: 20kHz │
│   • TIM16         │ └─ [PWM channels]  │
│                    │                    │
│                    │ ✓ PWM CH1: PA0     │
│                    │ ✓ PWM CH2: PA1     │
│                    │ ✓ PWM CH3: PA2     │
│                    │ ✓ PWM CH4: PB11    │
│                                          │
└──────────────────────────────────────────┘
```

### Connectivity Tab (xem SPI1)
```
┌──────────────────────────────────────────┐
│ CubeMX - SPI1 Configuration              │
├──────────────────────────────────────────┤
│                                          │
│ Left Panel:        │ Configuration:      │
│ • Connectivity ▼   │ SPI1:               │
│   • I2C1          │ ├─ Mode: Master     │
│   • SPI1 ← HERE   │ ├─ Prescaler: /4    │
│   • SPI2          │ ├─ Freq: 16.0 Mbps  │
│   • SPI4          │ ├─ Data: 8 bit      │
│   • USART1        │ └─ NSS: Hard Out    │
│   • ...            │                    │
│                                          │
└──────────────────────────────────────────┘
```

---

## 🎯 NEXT: Sau Khi Regenerate Xong

1. ✅ Build project: `make clean && make`
2. ✅ Flash to board: F11 (Debug) hoặc `make flash`
3. ✅ Open terminal: 115200 8N1
4. ✅ Reset board → see output
5. ✅ Start developing your flight controller!

---

**Bất kỳ lúc nào muốn update CubeMX, chỉ cần:**
```
Làm lại Bước 1-8 (mất ~10 phút)
Tất cả fixes sẽ vẫn còn, regenerate chỉ cập nhật code từ .ioc
```
