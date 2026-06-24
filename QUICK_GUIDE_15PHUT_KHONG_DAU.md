# QUICK GUIDE - DONG BO CUBEMX (15 PHUT) - KHONG DAU

## BUOC 1: SAO LUU (1 phut)

```powershell
# Windows PowerShell
cd "C:\Users\ducnt\Documents\STM32H750"
Copy-Item "Core\Src\main.c" "Core\Src\main.c.backup"
Copy-Item "Core\Inc\FreeRTOSConfig.h" "Core\Inc\FreeRTOSConfig.h.backup"
```

---

## BUOC 2: MO .ioc TRONG CubeMX (1 phut)

**Double-click:** `STM32H750.ioc`

hoac

**CubeMX** → `File` → `Open Project` → `STM32H750.ioc`

---

## BUOC 3: KIEM TRA 4 DIEU (5 phut)

### Kiem Tra 1: SPI1 Speed = 16MHz

```
CubeMX Ben Trai: Connectivity → SPI1

Kiem tra Ben Phai:
├─ Prescaler: /4 ← QUAN TRONG!
└─ BaudRate: 16.0 MBits/s

Neu van la /2 (32MHz):
  → Click dropdown "Prescaler"
  → Chon "/4"
  → Nhan Ctrl+S luu
```

### Kiem Tra 2: TIM2 PWM = 20kHz

```
CubeMX Ben Trai: Timers → TIM2

Kiem tra Ben Phai:
├─ Prescaler: 5
├─ Period: 999
└─ Frequency: 20 kHz ← DUNG ROI
```

### Kiem Tra 3: USART1

```
CubeMX Ben Trai: Connectivity → USART1

Kiem tra Ben Phai:
├─ Baud Rate: 115200
├─ TX: PA9
└─ RX: PA10
```

### Kiem Tra 4: Debug SWD+SWO

```
CubeMX Ben Trai: System Core → SYS
Ben Phai: Click tab "Debug"

Kiem tra: Trace Asynchronous SWO da chon
Pins: PA13(SWDIO), PA14(SWCLK), PB3(SWO)
```

---

## BUOC 4: GENERATE CODE (2 phut)

```
CubeMX Menu:
  Project → Generate Code

hoac nhan: Alt+K

Cho 30-60 giay cho toi khi thay "Generation Complete"
```

---

## BUOC 5: PHUC HOI DEBUG (3 phut)

### Cach A: Tu Dong (NHANH NHAT)

```powershell
cd "C:\Users\ducnt\Documents\STM32H750"
.\restore_debug.ps1

# Xong! Tat ca da duoc them lai
```

### Cach B: Tay (Neu A khong chay)

#### B1: Them include trong main.c (dong 26)

**Tim:**
```c
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
```

**Sua thanh:**
```c
/* USER CODE BEGIN Includes */
#include "debug_uart.h"
/* USER CODE END Includes */
```

#### B2: Them Debug_UART_Init() trong main.c (dong 168)

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
   
   Debug_UART_Init();
   Debug_Test();
   
   /* USER CODE END 2 */
```

#### B3: Bat FPU trong FreeRTOSConfig.h (dong 59)

**Tim:**
```c
#define configENABLE_FPU                         0
```

**Sua thanh:**
```c
#define configENABLE_FPU                         1
```

---

## BUOC 6: BUILD (1 phut)

```bash
make clean
make

# hoac Ctrl+B trong STM32CubeIDE
```

**Ket qua:**
```
Build succeeded
```

---

## BUOC 7: FLASH (1 phut)

```bash
make flash

# hoac F11 trong STM32CubeIDE
```

---

## BUOC 8: TEST (1 phut)

```
1. Mo terminal (PuTTY, TeraTerm, hoac minicom)
2. Dat 115200 8N1
3. Select COM port
4. Nhan nut RESET tren board
5. Ban se thay:

=== STM32H750 Flight Controller Debug UART Initialized ===
[INF] System Clock: 480 MHz
[INF] FreeRTOS Heap: 131072 bytes
[INF] Tick Rate: 1000 Hz
...

✓ XONG! TAT CA FIX DA DUOC AP DUNG!
```

---

## KIEM TRA CUOI CUNG

```
☐ SPI1: Prescaler = /4 (16MHz)
☐ TIM2: 20kHz
☐ USART1: 115200 bps, PA9/PA10
☐ generate Code: Thanh cong
☐ main.c: #include "debug_uart.h"
☐ main.c: Debug_UART_Init() co goi
☐ FreeRTOSConfig.h: configENABLE_FPU = 1
☐ Build: Khong co loi
☐ Flash: Thanh cong
☐ Terminal: Thay output ✓
```

---

## LENH NHANH

```bash
# Toan bo quy trinh
make clean
make
make flash

# hoac
Ctrl+B (build)
F11 (flash)
```

---

## VAN DE THUONG GAP

| Van De | Giai Phap |
|--------|-----------|
| Build error | Chay restore_debug.ps1 lai |
| Khong co output | Kiem tra 115200 bps, COM port |
| Garbled text | Sai baudrate |
| SPI1 van 32MHz | Sua tay .ioc: SPI1.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_4 |

---

**XONG! Thanh cong trong 15 phut! 🚁**
