# ✅ BUILD THÀNH CÔNG! - HƯỚNG DẪN UPLOAD

## 🎉 Kết Quả Build

```
[SUCCESS] Took 80.45 seconds
Creating esp32 image...
```

Code đã được compile thành công! Bây giờ bạn có thể upload lên ESP32.

---

## 🚀 Bước Tiếp Theo: UPLOAD LÊN ESP32

### Cách 1: Upload Qua PlatformIO (VS Code)

1. **Cắm ESP32 vào máy tính** qua cáp USB
2. **Kiểm tra COM port:**
   - Windows: Device Manager → Ports (COM & LPT)
   - Tìm "Silicon Labs CP210x" hoặc "CH340"
   - Ghi nhớ số COM (ví dụ: COM3, COM5)

3. **Upload code:**
   ```bash
   # Cách 1: Dùng lệnh
   C:\Users\USER\.platformio\penv\Scripts\platformio.exe run --target upload
   
   # Cách 2: Trong VS Code
   # Nhấn nút Upload (→) trên thanh PlatformIO
   # Hoặc nhấn Ctrl+Alt+U
   ```

4. **Mở Serial Monitor:**
   ```bash
   # Cách 1: Dùng lệnh
   C:\Users\USER\.platformio\penv\Scripts\platformio.exe device monitor
   
   # Cách 2: Trong VS Code
   # Nhấn nút Serial Monitor (plug icon)
   # Hoặc nhấn Ctrl+Alt+S
   ```

---

## 📺 Kết Quả Mong Đợi Trên Serial Monitor

Sau khi upload xong và ESP32 khởi động, bạn sẽ thấy:

```
========================================
ESP32 BLUETOOTH + BUTTON TEST
========================================
✓ Button GPIO16 initialized (Pull-up)

Initializing BLE...
✓ BLE Service started
✓ Advertising as: ESP32_Gimbal_Test

========================================
READY!
1. Connect from Android app
2. Press button on GPIO16 to send CAPTURE
========================================
```

---

## 🔌 Kết Nối Phần Cứng

**QUAN TRỌNG:** Kết nối phần cứng TRƯỚC KHI test!

```
ESP32 GPIO16 ────[Button]──── GND
```

**Chi tiết:**
1. Một đầu nút nhấn → GPIO16
2. Đầu kia của nút nhấn → GND
3. Không cần điện trở pull-up (đã dùng internal pull-up)

---

## 📱 Test Với Android App

### Bước 1: Kết Nối Bluetooth
1. Mở **Android App** (PoseLandmarker)
2. Vào **Settings** → Bluetooth
3. Tìm thiết bị: **ESP32_Gimbal_Test**
4. Nhấn **Connect**

**Serial Monitor sẽ hiển thị:**
```
✓ BLE Client Connected!
```

### Bước 2: Test Nút Nhấn
1. **Nhấn nút GPIO16** trên ESP32
2. **Serial Monitor hiển thị:**
   ```
   >>> BUTTON PRESSED!
   >>> Sent: CAPTURE
   ```
3. **App sẽ:**
   - Tự động chụp ảnh
   - Hiển thị Toast: "Remote Capture!"
   - Lưu ảnh vào `Pictures/PoseLandmarker/`

---

## ✅ Checklist Test

### Trước Khi Upload
- [x] Code đã build thành công
- [ ] ESP32 đã cắm USB vào máy tính
- [ ] Đã xác định COM port

### Sau Khi Upload
- [ ] Serial Monitor hiển thị "BLE Service started"
- [ ] Nút nhấn đã kết nối (GPIO16 ↔ GND)
- [ ] App tìm thấy "ESP32_Gimbal_Test"
- [ ] Bluetooth kết nối thành công
- [ ] Nhấn nút → App chụp ảnh

---

## 🐛 Xử Lý Lỗi Upload

### Lỗi: "Failed to connect to ESP32"

**Nguyên nhân:**
- ESP32 chưa vào chế độ upload
- COM port sai
- Driver chưa cài

**Giải pháp:**
1. **Giữ nút BOOT** trên ESP32
2. **Nhấn nút EN** (reset)
3. **Thả nút EN**, giữ BOOT thêm 2 giây
4. **Thả nút BOOT**
5. **Upload lại**

### Lỗi: "Serial port not found"

**Giải pháp:**
1. Kiểm tra Device Manager
2. Cài driver:
   - CP210x: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
   - CH340: http://www.wch.cn/downloads/CH341SER_ZIP.html
3. Thử cáp USB khác

### Lỗi: "Permission denied" (Linux/Mac)

**Giải pháp:**
```bash
sudo chmod 666 /dev/ttyUSB0
# Hoặc thêm user vào group dialout
sudo usermod -a -G dialout $USER
```

---

## 📊 Thông Tin Kỹ Thuật

| Thông Số | Giá Trị |
|----------|---------|
| Platform | Espressif 32 (6.11.0) |
| Board | ESP32 Dev Module |
| Framework | Arduino |
| Upload Speed | 921600 baud |
| Monitor Speed | 115200 baud |
| Flash Size | 4MB |

---

## 🔄 Lệnh Hữu Ích

```bash
# Build (compile) code
C:\Users\USER\.platformio\penv\Scripts\platformio.exe run

# Upload lên ESP32
C:\Users\USER\.platformio\penv\Scripts\platformio.exe run --target upload

# Mở Serial Monitor
C:\Users\USER\.platformio\penv\Scripts\platformio.exe device monitor

# Build + Upload + Monitor (tất cả trong một)
C:\Users\USER\.platformio\penv\Scripts\platformio.exe run --target upload && C:\Users\USER\.platformio\penv\Scripts\platformio.exe device monitor

# Clean build files
C:\Users\USER\.platformio\penv\Scripts\platformio.exe run --target clean

# List all connected devices
C:\Users\USER\.platformio\penv\Scripts\platformio.exe device list
```

---

## 📁 File Đã Tạo

Sau khi build, các file binary được tạo tại:
```
.pio/build/esp32dev/
├── firmware.bin       ← File chính để flash
├── firmware.elf       ← File debug
└── bootloader.bin     ← Bootloader ESP32
```

---

## 🎯 Sẵn Sàng Upload!

**Các bước:**
1. ✅ Code đã build thành công
2. 🔌 Cắm ESP32 vào USB
3. 📤 Upload code
4. 📺 Mở Serial Monitor
5. 🔗 Kết nối phần cứng (GPIO16 ↔ Button ↔ GND)
6. 📱 Kết nối Bluetooth từ app
7. 🎮 Test nút nhấn

**Chúc bạn thành công! 🚀**

---

## 📞 Cần Trợ Giúp?

- **Serial Monitor trống?** → Kiểm tra baud rate = 115200
- **Không upload được?** → Thử giữ nút BOOT khi upload
- **App không tìm thấy ESP32?** → Reset ESP32 và kiểm tra Serial Monitor
- **Nhấn nút không có phản ứng?** → Kiểm tra kết nối GPIO16 ↔ GND

Xem thêm:
- `QUICK_START.md` - Hướng dẫn nhanh
- `TEST_INSTRUCTIONS.md` - Hướng dẫn chi tiết
- `WIRING_DIAGRAM.md` - Sơ đồ kết nối
