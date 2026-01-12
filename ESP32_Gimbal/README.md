# 🎯 ESP32 Gimbal - Bluetooth Remote Capture Test

Test chức năng điều khiển từ xa: **Nhấn nút GPIO16 → ESP32 gửi "CAPTURE" qua Bluetooth → App chụp ảnh**

---

## 📋 Tổng Quan

Project này test kết nối Bluetooth BLE giữa ESP32 và Android App với các chức năng:

- ✅ **Bluetooth BLE** - Kết nối không dây với app
- ✅ **Button Input** - Đọc trạng thái nút nhấn GPIO16
- ✅ **Remote Capture** - Gửi lệnh chụp ảnh từ xa
- ✅ **Debounce** - Chống nhiễu nút nhấn (200ms)

---

## 🚀 Bắt Đầu Nhanh

### 1️⃣ Build Code
```bash
# Build project
C:\Users\USER\.platformio\penv\Scripts\platformio.exe run
```
**Kết quả:** `[SUCCESS] Took 80.45 seconds` ✅

### 2️⃣ Upload Lên ESP32
```bash
# Upload code
C:\Users\USER\.platformio\penv\Scripts\platformio.exe run --target upload
```

### 3️⃣ Mở Serial Monitor
```bash
# Monitor serial output
C:\Users\USER\.platformio\penv\Scripts\platformio.exe device monitor
```

### 4️⃣ Kết Nối Phần Cứng
```
ESP32 GPIO16 ────[Button]──── GND
```

### 5️⃣ Kết Nối Bluetooth
1. Mở Android App
2. Settings → Bluetooth
3. Tìm "ESP32_Gimbal_Test"
4. Connect

### 6️⃣ Test!
- Nhấn nút GPIO16
- App tự động chụp ảnh
- Toast hiển thị "Remote Capture!"

---

## 📚 Tài Liệu Hướng Dẫn

| File | Mô Tả |
|------|-------|
| **[QUICK_START.md](QUICK_START.md)** | Hướng dẫn upload và test nhanh |
| **[UPLOAD_GUIDE.md](UPLOAD_GUIDE.md)** | Hướng dẫn upload chi tiết sau khi build |
| **[TEST_INSTRUCTIONS.md](TEST_INSTRUCTIONS.md)** | Hướng dẫn test từng bước |
| **[WIRING_DIAGRAM.md](WIRING_DIAGRAM.md)** | Sơ đồ kết nối phần cứng |
| **[SUMMARY.md](SUMMARY.md)** | Tóm tắt toàn bộ project |

---

## 🔧 Cấu Trúc Project

```
ESP32_Gimbal/
├── src/
│   ├── test_bluetooth_button.cpp  ← Code test chính (GPIO16)
│   └── esp32_capture_button.cpp   ← Code tham khảo (GPIO19)
├── platformio.ini                 ← Cấu hình PlatformIO
├── README.md                      ← File này
├── QUICK_START.md                 ← Hướng dẫn nhanh
├── UPLOAD_GUIDE.md                ← Hướng dẫn upload
├── TEST_INSTRUCTIONS.md           ← Hướng dẫn test chi tiết
├── WIRING_DIAGRAM.md              ← Sơ đồ kết nối
└── SUMMARY.md                     ← Tóm tắt
```

---

## ⚙️ Thông Số Kỹ Thuật

### ESP32
| Thông Số | Giá Trị |
|----------|---------|
| Board | ESP32 Dev Module |
| GPIO Pin | 16 (Capture Button) |
| Debounce | 200ms |
| Baud Rate | 115200 |
| Upload Speed | 921600 |

### Bluetooth BLE
| Thông Số | Giá Trị |
|----------|---------|
| Device Name | ESP32_Gimbal_Test |
| Service UUID | 4fafc201-1fb5-459e-8fcc-c5c9c331914b |
| Characteristic UUID | beb5483e-36e1-4688-b7f5-ea07361b26a8 |
| MTU Size | 512 bytes |

### Android App
| Thông Số | Giá Trị |
|----------|---------|
| Capture Command | "CAPTURE" |
| Min Capture Interval | 2000ms (2 giây) |
| Photo Directory | Pictures/PoseLandmarker/ |

---

## 🔌 Kết Nối Phần Cứng

### Sơ Đồ
```
┌─────────────────┐
│     ESP32       │
│                 │
│  GPIO16 ●───┐   │
│             │   │
│   GND   ●───┼───┤
│             │   │
└─────────────┼───┘
              │
         ┌────┴────┐
         │  Button │
         └─────────┘
```

### Linh Kiện Cần Thiết
- [x] ESP32 Dev Board
- [x] Tactile Button (6x6mm hoặc 12x12mm)
- [x] 2x Jumper Wires
- [x] USB Cable

### Nguyên Lý Hoạt Động
- **Không nhấn:** GPIO16 = HIGH (pull-up)
- **Nhấn nút:** GPIO16 = LOW → Trigger!

---

## 📱 Tích Hợp Android App

### Constants.kt
```kotlin
object Bluetooth {
    const val REMOTE_CAPTURE_COMMAND = "CAPTURE"
    const val MIN_CAPTURE_INTERVAL_MS = 2000L
}
```

### CameraFragment.kt
```kotlin
override fun onDataReceived(data: String) {
    if (data.contains(Constants.Bluetooth.REMOTE_CAPTURE_COMMAND) && 
        System.currentTimeMillis() - lastCaptureTime > Constants.Bluetooth.MIN_CAPTURE_INTERVAL_MS) {
        lastCaptureTime = System.currentTimeMillis()
        activity?.runOnUiThread {
            takePhoto()
            Toast.makeText(context, "Remote Capture!", Toast.LENGTH_SHORT).show()
        }
    }
}
```

---

## ✅ Kết Quả Mong Đợi

### Serial Monitor (ESP32)
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

✓ BLE Client Connected!

>>> BUTTON PRESSED!
>>> Sent: CAPTURE
```

### Android App
- ✅ Tự động chụp ảnh
- ✅ Toast: "Remote Capture!"
- ✅ Ảnh lưu tại: `Pictures/PoseLandmarker/PoseLandmarker_YYYYMMDD_HHMMSS.jpg`
- ✅ Hiệu ứng flash trắng

---

## 🐛 Xử Lý Lỗi

### ESP32 không hiển thị trong Bluetooth
- **Nguyên nhân:** BLE chưa khởi động
- **Giải pháp:** Reset ESP32, kiểm tra Serial Monitor

### Nhấn nút không có phản ứng
- **Nguyên nhân:** Kết nối sai hoặc Bluetooth chưa kết nối
- **Giải pháp:** Kiểm tra GPIO16 ↔ GND, kiểm tra BLE connected

### App không chụp ảnh
- **Nguyên nhân:** Không có quyền Camera/Storage
- **Giải pháp:** Cấp quyền trong Settings → Apps

---

## 🔄 Sau Khi Test

### Nếu Thành Công ✅
1. **Tích hợp vào code chính:**
   - Copy logic button từ `test_bluetooth_button.cpp`
   - Paste vào file gimbal chính
   - Chỉnh sửa `platformio.ini` để build file chính

2. **Tùy chỉnh:**
   - Thay đổi GPIO nếu cần
   - Điều chỉnh debounce time
   - Thêm các lệnh khác (RECORD, STOP, etc.)

### Nếu Thất Bại ❌
1. Kiểm tra Serial Monitor
2. Kiểm tra kết nối phần cứng bằng multimeter
3. Xem Android Logcat để tìm lỗi
4. Đọc [TEST_INSTRUCTIONS.md](TEST_INSTRUCTIONS.md)

---

## 📊 Build Status

| Platform | Status |
|----------|--------|
| ESP32 | ✅ Build Success (80.45s) |
| Android | ✅ Ready |

---

## 🎓 Học Thêm

### ESP32 BLE
- [ESP32 BLE Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gap_ble.html)
- [Arduino BLE Library](https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE)

### PlatformIO
- [PlatformIO Documentation](https://docs.platformio.org/)
- [ESP32 Platform](https://docs.platformio.org/en/latest/platforms/espressif32.html)

---

## 📞 Hỗ Trợ

Nếu gặp vấn đề:
1. Kiểm tra Serial Monitor của ESP32
2. Kiểm tra Logcat của Android App
3. Đọc các file hướng dẫn trong project
4. Test từng phần riêng biệt

---

## 📝 Changelog

### Version 1.0 (2025-12-26)
- ✅ Chuyển từ GPIO19 sang GPIO16
- ✅ Build thành công với PlatformIO
- ✅ Tạo đầy đủ tài liệu hướng dẫn
- ✅ Test sẵn sàng

---

## 📄 License

MIT License - Free to use and modify

---

## 👨‍💻 Author

Pham Phuc - Gimbal Tracking Project

---

**Sẵn sàng test! Chúc bạn thành công! 🎉🚀**

---

## 🔗 Quick Links

- [Bắt Đầu Nhanh](QUICK_START.md)
- [Hướng Dẫn Upload](UPLOAD_GUIDE.md)
- [Hướng Dẫn Test](TEST_INSTRUCTIONS.md)
- [Sơ Đồ Kết Nối](WIRING_DIAGRAM.md)
- [Tóm Tắt](SUMMARY.md)
