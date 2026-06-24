# 📚 STM32H750 Documentation Index

Tất cả các hướng dẫn và file tôi đã tạo cho bạn. Chọn file phù hợp với nhu cầu của bạn.

---

## 🎯 Quick Navigation

**Bạn muốn làm gì?**

1. **Lần đầu setup & kết nối máy tính?**
   → [`PC_CONNECTION_GUIDE.md`](#1-pc-connection-guideumd)

2. **Regenerate code từ CubeMX?**
   → [`QUICK_SYNC_CUBEMX.md`](#2-quick-sync-cubemxmd-⚡-fastest)

3. **Chi tiết tất cả fixes tôi thực hiện?**
   → [`FIXES_AND_SUMMARY.md`](#3-fixes-and-summarymd)

4. **Hướng dẫn CubeMX chi tiết nhất?**
   → [`CUBEMX_REGENERATE_GUIDE.md`](#4-cubemx-regenerate-guidemd-detailed)

5. **Dùng debug UART trong code?**
   → [`Core/Src/debug_example.c`](#5-debug-example-codeufc)

---

## 📄 Danh Sách Tất Cả File

### Documentation Files

| # | File | Purpose | Read Time | Difficulty |
|---|------|---------|-----------|------------|
| 1 | `PC_CONNECTION_GUIDE.md` | Setup kết nối PC | 5 min | Easy |
| 2 | `QUICK_SYNC_CUBEMX.md` | Quick CubeMX sync | 10 min | Easy |
| 3 | `FIXES_AND_SUMMARY.md` | Tóm tắt all fixes | 10 min | Medium |
| 4 | `CUBEMX_REGENERATE_GUIDE.md` | Chi tiết CubeMX | 20 min | Medium |
| 5 | `README_DOCUMENTATION.md` | File này | - | Easy |

### Code Files (New/Modified)

| File | Type | Purpose |
|------|------|---------|
| `Core/Inc/debug_uart.h` | Header | Debug UART interface |
| `Core/Src/debug_uart.c` | Implementation | Debug UART code |
| `Core/Src/debug_example.c` | Example | 5 ví dụ task sử dụng debug UART |
| `Core/Src/main.c` | ✏️ Modified | Added debug init |
| `Core/Inc/FreeRTOSConfig.h` | ✏️ Modified | FPU enabled |
| `STM32H750.ioc` | ✏️ Modified | SPI1 speed reduced |

### Helper Scripts

| File | Platform | Purpose |
|------|----------|---------|
| `restore_debug.ps1` | Windows PowerShell | Auto-restore debug after CubeMX generate |
| `restore_debug.bat` | Windows CMD | Alternative restore script |

---

## 📖 Chi Tiết Từng File

### 1. PC_CONNECTION_GUIDE.md 🌐
**Bạn cần:** Kết nối máy tính để debug

**Nội dung:**
- Hardware setup (USB adapter wiring)
- PC terminal software (PuTTY, TeraTerm, minicom, Python)
- Expected output
- Troubleshooting

**Ví dụ:**
```
STM32H750      USB-Serial
PA9 (TX)   →   RX
PA10 (RX)  ←   TX
GND        →   GND
```

**Start here if:** Bạn mới setup project lần đầu

---

### 2. QUICK_SYNC_CUBEMX.md ⚡ (FASTEST)
**Bạn cần:** Nhanh nhất, đi thẳng vào việc

**Nội dung:**
- TL;DR (2 phút)
- 5 PHASE với checklist
- Visual guides
- ~15 minute total

**Checklist:**
```
☐ Backup files
☐ Verify SPI1 = 16MHz
☐ Generate Code (Alt+K)
☐ Restore debug (script)
☐ Build & Flash
☐ Test
```

**Start here if:** Bạn muốn chọn con đường nhanh nhất

---

### 3. FIXES_AND_SUMMARY.md 📋
**Bạn cần:** Biết tôi sửa gì, tại sao, cách thế nào

**Nội dung:**
- All 4 fixes with detailed explanation
- Configuration summary table
- File locations
- Verification checklist

**Các fix:**
1. ✅ FPU enabled (CRITICAL)
2. ✅ SPI1 speed 32MHz → 16MHz
3. ✅ Added debug UART
4. ✅ Integrated into main.c

**Start here if:** Bạn muốn hiểu chi tiết mỗi fix

---

### 4. CUBEMX_REGENERATE_GUIDE.md (DETAILED)
**Bạn cần:** Hướng dẫn chi tiết từng bước

**Nội dung:**
- 8 bước detailed từng bước
- Backup strategy
- Screenshot descriptions
- Step-by-step verification

**8 Bước:**
1. Backup files
2. Mở .ioc trong CubeMX
3. Xác minh fix được áp dụng
4. Regenerate code
5. Restore debug init
6. Build & verify
7. Flash & test
8. Checkpoints

**Start here if:** Bạn muốn chỉ dẫn rõ ràng, tỳ mỳ

---

### 5. debug_example.c 💡 (Example Code)
**Bạn cần:** Biết cách dùng debug_uart trong code

**Nội dung:**
- 5 Example tasks:
  1. SystemStatus_Task
  2. SensorData_Task
  3. MotorControl_Task
  4. CommandProcessor_Task
  5. PerformanceMonitor_Task

**Ví dụ:**
```c
#include "debug_uart.h"

void my_task(void) {
    float temp = 25.5;
    DEBUG_INFO("Temperature: %.2f °C", temp);
}
```

**Start here if:** Bạn muốn example code để copy-paste

---

## 🚀 Recommended Reading Order

### Scenario 1: "Mình hoàn toàn mới"
```
1. PC_CONNECTION_GUIDE.md (5 min) ← Setup hardware
2. QUICK_SYNC_CUBEMX.md (10 min) ← Sync with CubeMX
3. debug_example.c (5 min) ← See examples
→ START DEVELOPING!
```

### Scenario 2: "Mình update CubeMX thường xuyên"
```
1. QUICK_SYNC_CUBEMX.md (10 min) ← Đây là workflow bạn
2. restore_debug.ps1 (automatic) ← Chạy script
3. Done!
```

### Scenario 3: "Mình muốn hiểu tất cả"
```
1. FIXES_AND_SUMMARY.md (10 min)
2. CUBEMX_REGENERATE_GUIDE.md (20 min)
3. PC_CONNECTION_GUIDE.md (5 min)
4. debug_example.c (10 min)
→ COMPLETE UNDERSTANDING!
```

### Scenario 4: "Mình muốn làm nhanh nhất"
```
1. Chạy: restore_debug.ps1
2. Chạy: make clean && make && make flash
3. Terminal 115200 bps
4. DONE!
```

---

## 🔧 Helper Scripts Usage

### restore_debug.ps1 (PowerShell)

**Khi dùng:**
Sau khi regenerate code từ CubeMX, tự động restore debug UART

**Cách chạy:**
```powershell
# Windows PowerShell
cd C:\Users\ducnt\Documents\STM32H750
.\restore_debug.ps1

# Git Bash (nếu không có PowerShell)
pwsh restore_debug.ps1
```

**Output:**
```
✓ Include debug_uart.h
✓ Debug_UART_Init() call
✓ Debug_Test() call
✓ FPU enabled
```

---

## 📋 Verification Checklist - Before Final Build

```
Project Setup:
☐ STM32H750.ioc file present
☐ Core/Inc/ folder exists
☐ Core/Src/ folder exists
☐ Makefile or STM32CubeIDE project configured

New Files Added:
☐ Core/Inc/debug_uart.h exists
☐ Core/Src/debug_uart.c exists
☐ Core/Src/debug_example.c exists

Code Modifications:
☐ Core/Src/main.c: #include "debug_uart.h" present
☐ Core/Src/main.c: Debug_UART_Init() called
☐ Core/Src/main.c: Debug_Test() called
☐ Core/Inc/FreeRTOSConfig.h: configENABLE_FPU = 1

CubeMX Configuration:
☐ SPI1.BaudRatePrescaler = /4 (16 MHz)
☐ TIM2: Prescaler=5, Period=999, Frequency=20kHz
☐ USART1: 115200 bps, PA9(TX), PA10(RX)
☐ Debug: SWD+SWO enabled

Build & Flash:
☐ make clean && make → No errors
☐ make flash → Success
☐ Terminal @ 115200 bps
☐ Reset board → See debug output ✅
```

---

## 📞 Common Issues & Where to Find Answers

| Issue | Guide to Read |
|-------|---------------|
| "No output on terminal" | PC_CONNECTION_GUIDE.md → Troubleshooting |
| "Garbled text" | PC_CONNECTION_GUIDE.md → Troubleshooting |
| "Can't compile" | QUICK_SYNC_CUBEMX.md → Phase 5 |
| "SPI1 still 32MHz" | CUBEMX_REGENERATE_GUIDE.md → Step 3 |
| "How to use debug_uart?" | debug_example.c → Examples |
| "Need full details" | FIXES_AND_SUMMARY.md → All sections |

---

## 🎯 Quick Command Reference

### Build & Flash
```bash
# Clean & Build
make clean && make

# Just Flash
make flash

# Or from STM32CubeIDE
Ctrl+B              # Build
F11 or Ctrl+F11     # Flash & Run
```

### Debug Terminal
```bash
# Windows PuTTY
putty.exe -serial COM3 -sercfg 115200,8,1,N

# Linux minicom
sudo minicom -s

# Python
python3 serial_monitor.py
```

### Restore After CubeMX
```powershell
# Automatic
.\restore_debug.ps1

# Manual: Edit Core/Src/main.c & Core/Inc/FreeRTOSConfig.h
# See: QUICK_SYNC_CUBEMX.md → Phase 4B
```

---

## 📊 File Statistics

```
Documentation:
- Total: 5 markdown files
- Total lines: ~2000+ lines
- Total size: ~200 KB

Code:
- New files: 3 (debug_uart.h, .c, example)
- Modified files: 3 (.ioc, FreeRTOSConfig.h, main.c)
- Total new code: ~600 lines

Scripts:
- Helper scripts: 2 (PowerShell, Batch)
```

---

## ✅ You're All Set!

Pick the guide you need and start working. Good luck with your flight controller development! 🚁

**Questions?** Refer to the specific guide for your scenario, or contact support.

---

**Last Updated:** June 24, 2026
**Project:** STM32H750VBTx Custom Drone Flight Controller
**Status:** ✅ All fixes applied, ready for development
