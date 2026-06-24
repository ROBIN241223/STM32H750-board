# ⭐ START HERE - BAT DAU TAI DAY!

Xin chao! Ban co day du tat ca nhung can thiet de phat trien Flight Controller.

---

## 🎯 BAN CAN LAM NGAY BAY GIO?

### Chon 1 trong 3 duong dan duoi:

#### 🚀 DUONG 1: Lam NHANH (15 phut)

Ban muon bat dau ngay va lam xong trong 15 phut.

**Chon file nay:**
```
→ QUICK_GUIDE_15PHUT_KHONG_DAU.md
```

**Noi dung:** 8 buoc duy nhat, tom tat, khong chi tiet

**Thoi gian:** 15 phut

---

#### 📖 DUONG 2: Lam CHI TIET (30-40 phut)

Ban muon hieu ro tung chi tiet, lan dau tien, khong muon loi.

**Chon file nay:**
```
→ HUONG_DAN_DONGBO_CUBEMX_KHONG_DAU.md
```

**Noi dung:** Chi tiet, co code mau, vi du, hinh anh

**Thoi gian:** 30-40 phut

**Loi ich:** Sau lan nay, ban se biet het va lan sau chi can 15 phut

---

#### 🔍 DUONG 3: Hoc LY THUYET TRUOC (10 phut)

Ban muon biet "toi sua gi", "tai sao", trc khi lam.

**Chon file nay:**
```
→ FIXES_AND_SUMMARY.md
```

**Noi dung:** Tat ca 4 fix, lam sao, tai sao

**Sau do:** Lam theo QUICK_GUIDE hoac HUONG_DAN

---

## 📋 BANG KIEM TRA TOI THIEU

Truoc khi bat dau, chac chan ban co:

```
Hardware:
☐ STM32H750 board
☐ USB-to-Serial adapter (3.3V - KHONG 5V!)
☐ Micro USB cable
☐ May tinh voi port USB

Software:
☐ STM32CubeMX (6.x tro len)
☐ STM32CubeIDE hoac Makefile
☐ Terminal software (PuTTY, TeraTerm, minicom)

File:
☐ STM32H750.ioc (da sua)
☐ debug_uart.h (tao moi)
☐ debug_uart.c (tao moi)
☐ Huong dan (ban da doc trang nay!)
```

---

## 🚀 KHI NAO BAT DAU?

### Neu ban muon lam NGAY:

```
1. Doc: QUICK_GUIDE_15PHUT_KHONG_DAU.md
2. Lam theo huong dan
3. Xong trong 15 phut
4. San sang phat trien!
```

### Neu ban la LAN DAU va muon CHAC CHAN:

```
1. Doc: HUONG_DAN_DONGBO_CUBEMX_KHONG_DAU.md
2. Lam tung buoc nhu huong dan
3. Xong trong 30-40 phut
4. San sang phat trien!
```

### Neu ban muon THAM KHAO SAU:

```
1. Luu: HUONG_DAN_DONGBO_CUBEMX_KHONG_DAU.md (chi tiet)
2. Luu: QUICK_GUIDE_15PHUT_KHONG_DAU.md (nhanh)
3. Luu: PC_CONNECTION_GUIDE.md (USB setup)
4. Luu: FIXES_AND_SUMMARY.md (tham khao)
```

---

## 📚 TAT CA FILE HUONG DAN

### Tieng Viet KHONG DAU (CHINH - HAN DUNG):

```
1. QUICK_GUIDE_15PHUT_KHONG_DAU.md ← CHON CAI NAY TRUOC!
2. HUONG_DAN_DONGBO_CUBEMX_KHONG_DAU.md ← CHI TIET TOAN BO
3. TOM_TAT_VE_GUIDE_KHONG_DAU.md ← CHON FILE NAO?
```

### Tieng Viet CO DAU va Tieng Anh (THAM KHAO):

```
4. FIXES_AND_SUMMARY.md ← Biet loi gi bi sua
5. PC_CONNECTION_GUIDE.md ← Ket noi USB debug
6. CUBEMX_REGENERATE_GUIDE.md ← Chi tiet (tieng Anh)
7. QUICK_SYNC_CUBEMX.md ← Nhanh (tieng Anh)
8. README_DOCUMENTATION.md ← Tim file nao (tieng Anh)
```

---

## 📄 TAT CA FILE CODE DA SUB VA TAO MOI

### Code Files (Chuẩn bị sẵn):

```
✓ Core/Inc/debug_uart.h ← Debug interface
✓ Core/Src/debug_uart.c ← Implementation
✓ Core/Src/debug_example.c ← 5 vi du

✏️ Core/Src/main.c (sua: them debug)
✏️ Core/Inc/FreeRTOSConfig.h (sua: bat FPU)
✏️ STM32H750.ioc (sua: SPI1=16MHz)
```

### Scripts (Auto Restore):

```
⚙️ restore_debug.ps1 ← Chay sau Generate Code
⚙️ restore_debug.bat ← Alternative (Windows CMD)
```

---

## ⚡ LENH NHANH NHAT

Neu ban biet roi va chi can co loi:

```bash
# 1. Regenerate code
# (CubeMX: Alt+K)

# 2. Phuc hoi
.\restore_debug.ps1

# 3. Build
make clean && make

# 4. Flash
make flash

# 5. Test
# Terminal 115200 bps, nhan RESET tren board
```

---

## ✅ XUAT PHAT NGAY!

### CHON DUONG DI:

```
Neu ban: Muon lam nhanh (15 phut)
→ Mo: QUICK_GUIDE_15PHUT_KHONG_DAU.md

Neu ban: Lan dau + muon hieu (30-40 phut)
→ Mo: HUONG_DAN_DONGBO_CUBEMX_KHONG_DAU.md

Neu ban: Muon hieu ly thuyet truoc
→ Mo: FIXES_AND_SUMMARY.md
```

---

## 🎯 MAT TIEU

Sau khi lam xong:

1. ✓ Build project thanh cong
2. ✓ Flash vao board STM32H750
3. ✓ Terminal thay debug output
4. ✓ Tat ca fix da duoc ap dung
5. ✓ San sang viet code phat trien!

---

## 🆘 CO THAM HOI TI MAT?

| Cau Hoi | Dap An |
|--------|--------|
| "Toi can bat dau tu dau?" | Doc: QUICK_GUIDE_15PHUT_KHONG_DAU.md |
| "Toi muon hieu chi tiet?" | Doc: HUONG_DAN_DONGBO_CUBEMX_KHONG_DAU.md |
| "Toi muon biet "toi sua gi"?" | Doc: FIXES_AND_SUMMARY.md |
| "Co loi khi setup USB?" | Doc: PC_CONNECTION_GUIDE.md |
| "Khong biet dung file nao?" | Doc: TOM_TAT_VE_GUIDE_KHONG_DAU.md |

---

## 🚀 SAN SANG CHUA?

**Neu co:** Xuat phat!
```
→ QUICK_GUIDE_15PHUT_KHONG_DAU.md
```

**Neu chua:** Tham khao trc
```
→ TOM_TAT_VE_GUIDE_KHONG_DAU.md
```

---

## 💡 MOT SO LUU Y

1. **Backup trc khi generate** - Luon sao luu main.c
2. **3.3V USB adapter** - Khong 5V!
3. **115200 bps** - Terminal phai dung toc do nay
4. **Restore sau generate** - Chay restore_debug.ps1 sau khi generate
5. **Test sau moi lan** - Kiem tra output terminal

---

## 🎉 CHUC BAN THANH CONG!

Chon duong dan ban thich va bat dau!

```
15 PHUT:    QUICK_GUIDE_15PHUT_KHONG_DAU.md
40 PHUT:    HUONG_DAN_DONGBO_CUBEMX_KHONG_DAU.md
```

**LET'S BUILD! 🚁**

---

**Last Updated: June 24, 2026**
**Status: ✅ Ready to Go!**
