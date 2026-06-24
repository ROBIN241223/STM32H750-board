# Huong Dan Dong Bo CubeMX - Tieng Viet Khong Dau

## TOM TAT NHANH (10 PHUT)

Neu ban chi co 10 phut, lam theo cac buoc nay:

```
1. Mo STM32H750.ioc bang CubeMX
2. Kiem tra: SPI1 Prescaler = /4 (16MHz) OK
3. Project → Generate Code (Alt+K)
4. Chay: restore_debug.ps1
5. Build: make clean && make
6. Flash: make flash
7. Terminal: 115200 bps
```

---

## DANH SACH TOAN BO CAC BUOC

### BUOC 1: SAO LUU CAC FILE QUAN TRONG (5 phut)

#### 1a: Sao luu tren Windows (PowerShell)

```powershell
cd "C:\Users\ducnt\Documents\STM32H750"

Copy-Item "Core\Src\main.c" "Core\Src\main.c.backup"
Copy-Item "Core\Src\stm32h7xx_it.c" "Core\Src\stm32h7xx_it.c.backup"
Copy-Item "Core\Inc\FreeRTOSConfig.h" "Core\Inc\FreeRTOSConfig.h.backup"

echo "Sao luu xong!"
```

#### 1b: Sao luu tren Linux/Mac

```bash
cd /path/to/STM32H750
cp Core/Src/main.c Core/Src/main.c.backup
cp Core/Src/stm32h7xx_it.c Core/Src/stm32h7xx_it.c.backup
cp Core/Inc/FreeRTOSConfig.h Core/Inc/FreeRTOSConfig.h.backup
echo "Sao luu xong!"
```

---

### BUOC 2: MO FILE .ioc TRONG CubeMX (2 phut)

#### Cach 1: Tu File Explorer
```
Double-click: STM32H750.ioc
→ CubeMX tu dong mo
```

#### Cach 2: Tu CubeMX
```
CubeMX → File → Open Project
→ Tim: C:\Users\ducnt\Documents\STM32H750\STM32H750.ioc
→ Click Open
```

#### Cach 3: Tu STM32CubeIDE
```
Project Explorer
→ Right-click project
→ Open With → STM32CubeMX
```

**Cho 1-2 phut cho toi khi CubeMX load xong (co the hoi "Save modifications?" → Click Yes)**

---

### BUOC 3: KIEM TRA CACH FIX DA DUOC AP DUNG (10 phut)

#### KT 1: SPI1 Toc Do = 16MHz

**Vi tri trong CubeMX:**
```
Ben trai: Connectivity (click expand)
  → SPI1 (click vao)
Ben phai: Configuration panel se hien
```

**Kiem tra:**
```
Tim dong sau trong panel:
├─ Mode: Full-Duplex Master
├─ Prescaler: /4 ← CAI NAY QUAN TRONG!
│   (neu van la /2 tuc 32MHz, phai sua thanh /4)
├─ BaudRate: 16.0 MBits/s ← Check day
├─ Data Size: 8 bits
├─ CRC: Disabled
└─ NSS: Hard Output (PA15)
```

**Neu Prescaler van la /2 (32MHz):**
```
1. Click dropdown "Prescaler"
2. Chon "/4"
3. Verify "BaudRate" thay doi thanh 16.0 MBits/s
4. Nhan Ctrl+S de luu (hoac tu dong luu)
```

---

#### KT 2: TIM2 PWM = 20kHz

**Vi tri:**
```
Ben trai: Timers (click expand)
  → TIM2 (click vao)
Ben phai: Configuration panel
```

**Kiem tra:**
```
├─ Prescaler: 5 (tinh toan: 6-1)
├─ Period: 999 (tinh toan: 1000-1)
├─ Frequency: 20 kHz ← Quan trong
├─ Auto-Reload: Enabled
│
└─ PWM Channels:
   ├─ CH1: PA0 (PWM Generation)
   ├─ CH2: PA1 (PWM Generation)
   ├─ CH3: PA2 (PWM Generation)
   └─ CH4: PB11 (PWM Generation)
```

**OK chu ky tinh toan:**
```
120MHz (APB1 clock) / 6 (prescaler) / 1000 (period) = 20kHz
```

---

#### KT 3: I2C1 Pin

**Vi tri:**
```
Ben trai: Connectivity
  → I2C1 (click vao)
```

**Kiem tra:**
```
├─ Timing: 0x307075B1 (400kHz standard)
├─ Mode: I2C
├─ Address Mode: 7-bit
│
└─ Pins:
   ├─ SCL: PB8 ← OK
   └─ SDA: PB9 ← OK
```

---

#### KT 4: USART1 Debug Port

**Vi tri:**
```
Ben trai: Connectivity
  → USART1 (click vao)
```

**Kiem tra:**
```
├─ Mode: Asynchronous
├─ Baud Rate: 115200 ← Quan trong!
├─ Word Length: 8 Bits
├─ Stop Bits: 1
├─ Parity: None
│
└─ Pins:
   ├─ TX: PA9 ← OK
   └─ RX: PA10 ← OK
```

---

#### KT 5: Debug Pin (SWD + SWO)

**Vi tri:**
```
Ben trai: System Core
  → SYS (click vao)
Ben phai: Click tab "Debug"
  → Chon "Trace Asynchronous SWO"
```

**Kiem tra:**
```
├─ PA13: SWDIO ← OK
├─ PA14: SWCLK ← OK
└─ PB3: SWO (Trace output) ← OK
```

---

### BUOC 4: TIEN HANH GENERATE CODE (5 phut)

#### 4a: Click Generate

**Cach 1 - Menu:**
```
STM32CubeMX
  → Project
    → Generate Code
```

**Cach 2 - Phim tat:**
```
Alt+K
```

**Dialog se hien:**
```
Dang tao code...
✓ Cleanup
✓ Tao MSP
✓ Tao HAL
✓ Hoan thanh

[OK]
```

**Cho 30-60 giay cho toi khi thay "Generation Complete"**

---

#### 4b: Kiem tra co loi khong

```
Neu co loi:
  ← Click [Details] de xem chi tiet
  ← Thong thuong la do toolchain khong day du

Neu thanh cong:
  ← Close dialog
  ← Tiep tuc BUOC 5
```

---

### BUOC 5: PHUC HOI DEBUG UART (3-5 phut)

**Dac biet quan trong:** CubeMX se xoa cac dieu toi them vao main.c, phai phuc hoi lai!

#### Cach A: Tu Dong (PowerShell Script) - TUC NHAT

```powershell
# Mo PowerShell va chay script
cd "C:\Users\ducnt\Documents\STM32H750"
.\restore_debug.ps1
```

**Output mong doi:**
```
============================================================
  STM32H750 - Phuc Hoi Debug UART
============================================================

Buoc 1: Kiem tra include debug_uart.h...
        ✓ Da them include!

Buoc 2: Kiem tra Debug_UART_Init()...
        ✓ Da them Debug_UART_Init()!

Buoc 3: Kiem tra FreeRTOSConfig.h...
        ✓ Da bat FPU!

Buoc 4: Xac minh toan bo...
✓ Include debug_uart.h
✓ Debug_UART_Init() call
✓ Debug_Test() call
✓ FPU enabled

============================================================
  ✓ Phuc Hoi Thanh Cong!
============================================================
```

---

#### Cach B: Tay (Neu script khong chay)

##### B1: Them include debug_uart.h

**File:** `Core/Src/main.c` (dong khoang 26)

**Tim:**
```c
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
```

**Sua thanh:**
```c
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug_uart.h"
/* USER CODE END Includes */
```

**Cach sua trong STM32CubeIDE:**
```
1. Open Core/Src/main.c
2. Ctrl+G → Go to Line 26
3. Dua con tro sau "/* USER CODE BEGIN Includes */"
4. Nhan Enter
5. Ghi: #include "debug_uart.h"
6. Ctrl+S luu
```

---

##### B2: Them Debug_UART_Init() call

**File:** `Core/Src/main.c` (dong khoang 168)

**Tim:**
```c
   MX_TIM2_Init();
   /* USER CODE BEGIN 2 */

   /* USER CODE END 2 */
```

**Sua thanh:**
```c
   MX_TIM2_Init();
   /* USER CODE BEGIN 2 */
   
   /* Khoi tao Debug UART cho ket noi voi PC */
   Debug_UART_Init();
   Debug_Test();
   
   /* USER CODE END 2 */
```

**Cach sua trong STM32CubeIDE:**
```
1. Open Core/Src/main.c
2. Ctrl+F (Tim) → Tim "MX_TIM2_Init();"
3. Them 2 dong sau no:
   - Debug_UART_Init();
   - Debug_Test();
4. Ctrl+S luu
```

---

##### B3: Bat FPU

**File:** `Core/Inc/FreeRTOSConfig.h` (dong 59)

**Tim:**
```c
#define configENABLE_FPU                         0
```

**Sua thanh:**
```c
#define configENABLE_FPU                         1
```

**Cach sua:**
```
1. Open Core/Inc/FreeRTOSConfig.h
2. Ctrl+G → Line 59
3. Doi "0" thanh "1"
4. Ctrl+S luu
```

---

### BUOC 6: BUILD PROJECT (5 phut)

#### 6a: Build

```bash
# Cach 1: Dung Makefile
cd "C:\Users\ducnt\Documents\STM32H750"
make clean
make

# Cach 2: STM32CubeIDE
Ctrl+B  (Build All)
```

**Output mong doi:**
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

**Neu co loi:**
```
Loi "undefined reference to 'Debug_UART_Init'"
→ Quay lai B1: sua #include "debug_uart.h"

Loi "'debug_uart.h': No such file"
→ Kiem tra file ton tai: Core/Inc/debug_uart.h
```

---

#### 6b: Flash len board

```bash
# Cach 1: Makefile
make flash

# Cach 2: STM32CubeIDE
F11 (Debug Mode)
hoac Ctrl+F11 (Run Mode)

# Cach 3: STM32CubeProgrammer
- Open STM32CubeProgrammer
- Select ST-Link hoac UART
- Load: build/STM32H750.elf
- Click [Download]
```

---

### BUOC 7: KIEM TRA OUTPUT (2 phut)

```
1. Mo terminal (PuTTY, TeraTerm, minicom)
2. Chon COM port (neu dung USB-Serial adapter)
3. Dat 115200 8N1
4. Click Open
5. Nhan nut RESET tren board STM32H750
6. Ban se thay:

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

**✓ Neu thay output → THANH CONG! TAT CA FIX DA DUOC AP DUNG!**

---

## DANH SACH KIEM TRA CUOI CUNG

```
Truoc Tien:
☐ Sao luu: main.c, stm32h7xx_it.c, FreeRTOSConfig.h

Trong CubeMX:
☐ SPI1 Prescaler = /4 (16MHz)
☐ TIM2: Prescaler=5, Period=999, Frequency=20kHz
☐ I2C1: SCL=PB8, SDA=PB9
☐ USART1: 115200 bps, TX=PA9, RX=PA10
☐ Debug: PA13 SWDIO, PA14 SWCLK, PB3 SWO

Generate Code:
☐ Project → Generate Code (Alt+K) → Thanh cong

Phuc Hoi Debug:
☐ #include "debug_uart.h" co mat trong main.c
☐ Debug_UART_Init(); co dat trong main()
☐ Debug_Test(); co dat trong main()
☐ FreeRTOSConfig.h: configENABLE_FPU = 1

Build:
☐ make clean && make → Khong co loi
☐ make flash → Thanh cong

Kiem Tra:
☐ Terminal 115200 bps
☐ Nhan nut RESET tren board
☐ Thay output debug ✓
```

---

## LUONG TIEN HANH TONG THE

```
             START
               ↓
        [1] Sao Luu File
               ↓
     [2] Mo CubeMX voi .ioc
               ↓
   [3] Kiem Tra: SPI1=16MHz, TIM2=20kHz
               ↓
  [4] Project → Generate Code (Alt+K)
               ↓
  [5] Chay restore_debug.ps1 (hoac sua tay main.c)
               ↓
      [6] make clean && make
               ↓
         [7] make flash
               ↓
      [8] Terminal 115200 bps
               ↓
   [9] Nhan RESET → Thay output ✓
               ↓
         THANH CONG!
               ↓
   San sang phat trien dung dung!
```

---

## THOI GIAN THAM KHAO

```
Sao luu:           ~2 phut
Kiem tra CubeMX:   ~5 phut
Generate Code:     ~2 phut
Phuc hoi Debug:    ~3 phut
Build:             ~1 phut
Flash:             ~1 phut
Test:              ~1 phut
─────────────────────────
Tong cong:         ~15 phut
```

---

## GHI CHU QUAN TRONG

### NHAP KHAU (Dieu quan trong trc khi lam)

1. **TRAI MAT CAN LOI SAO LUU**
   - Neu cai gi sai, ban co the phuc hoi tu .backup file

2. **KIEM TRA TUNG BUOC**
   - Dung bo qua buoc kiem tra
   - Moi buoc quan trong

3. **SCRIPT RESTORE TIEN LOI**
   - Neu co PowerShell, dung script (nhanh)
   - Neu khong, sua tay (mat thoi gian)

---

## CAC VAN DE THUONG GAP VA GIAI PHAP

| Van De | Giai Phap |
|--------|-----------|
| CubeMX khong mo duoc .ioc | Cai dat CubeMX 6.x moi nhat |
| "Can't generate code" | Dong project, xoa .mxproject folder, mo lai |
| Build error sau generate | Chay restore_debug.ps1 lai lan nua |
| "Undefined reference" | Kiem tra #include "debug_uart.h" co trong main.c |
| Khong co output terminal | Kiem tra 115200 bps, COM port dung |
| SPI1 van la 32MHz | Sua tay .ioc: SPI1.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_4 |

---

## LENH NHANH

### Build & Flash

```bash
# Lam sach va build
make clean
make

# Chi flash thoi
make flash

# hoac tu STM32CubeIDE
Ctrl+B              # Build
F11 hoac Ctrl+F11   # Flash
```

### Terminal Debug

```bash
# Windows PuTTY
putty.exe -serial COM3 -sercfg 115200,8,1,N

# Linux minicom
sudo minicom -s

# Python monitor
python3 serial_monitor.py
```

### Phuc Hoi Sau Generate

```powershell
# Tu dong
.\restore_debug.ps1

# hoac sua tay
# Xem BUOC 5 - CACH B
```

---

## TINH TOAN KIEM TRA

### SPI1 Speed = 16MHz

```
Prescaler = /4
APB1 Clock = 120 MHz
16 = 120 / (4 / 1) ✓
```

### TIM2 PWM = 20kHz

```
Prescaler = 6 (hay ghi: 5 trong CubeMX, tinh 6-1)
Period = 1000 (hay ghi: 999 trong CubeMX, tinh 1000-1)
Clock = 120 MHz

Toc do = 120MHz / 6 / 1000 = 20kHz ✓
```

---

## TRANG THAI XONG

```
✓ Sao luu file
✓ Kiem tra cau hinh CubeMX
✓ Generate code tu .ioc
✓ Phuc hoi debug UART
✓ Build thanh cong
✓ Flash len board
✓ Test output debug
✓ San sang phat trien!
```

---

## NEXT: SAU KHI HOAN THANH

1. Sua code cua ban (add tasks, sensors, etc)
2. Them DEBUG_INFO() in cac diem quan trong
3. Build va Flash lai
4. Dung terminal de theo doi output
5. Phat trien flight controller!

---

**CHUC BAN THÀNH CÔNG! 🚁**

Chi can lam lai buoc nay ~15 phut moi lan update CubeMX.
