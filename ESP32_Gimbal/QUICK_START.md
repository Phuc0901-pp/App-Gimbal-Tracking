# 🚀 HƯỚNG DẪN UPLOAD CODE TEST LÊN ESP32

## 📋 Tóm Tắt
Code đã sẵn sàng để test chức năng:
- **GPIO16** + Button → Gửi lệnh **"CAPTURE"** qua Bluetooth → App chụp ảnh

---

## 🔧 Cách 1: Sử Dụng PlatformIO (VS Code)

### Bước 1: Mở Project
1. Mở **VS Code**
2. File → Open Folder → Chọn `ESP32_Gimbal`
3. PlatformIO sẽ tự động nhận diện project

### Bước 2: Upload Code
1. Cắm ESP32 vào máy tính qua USB
2. Nhấn nút **Upload** (mũi tên →) trên thanh PlatformIO
3. Hoặc nhấn `Ctrl+Alt+U`

### Bước 3: Mở Serial Monitor
1. Nhấn nút **Serial Monitor** (plug icon) trên thanh PlatformIO
2. Hoặc nhấn `Ctrl+Alt+S`
3. Baud rate: **115200**

---

## 🔧 Cách 2: Sử Dụng Arduino IDE

### Bước 1: Chuẩn Bị
1. Mở **Arduino IDE**
2. File → Open → Chọn file:
   ```
   c:\Pham_Phuc\Mobile_app\mediapipe-samples\ESP32_Gimbal\src\test_bluetooth_button.cpp
   ```

### Bước 2: Cài Đặt Board
1. Tools → Board → ESP32 Arduino → **ESP32 Dev Module**
2. Tools → Upload Speed → **921600**
3. Tools → Port → Chọn COM port của ESP32

### Bước 3: Cài Thư Viện
1. Sketch → Include Library → Manage Libraries
2. Tìm và cài:
   - **ESP32Servo**
   - **ArduinoBLE** (hoặc built-in BLE của ESP32)

### Bước 4: Upload
1. Nhấn nút **Upload** (→)
2. Đợi compile và upload xong

### Bước 5: Mở Serial Monitor
1. Tools → Serial Monitor
2. Baud rate: **115200**

---

## 📱 Test Với Android App

### Bước 1: Kiểm Tra ESP32
Sau khi upload, Serial Monitor sẽ hiển thị:
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

### Bước 2: Kết Nối Phần Cứng
```
ESP32 GPIO16 ----[Button]---- GND
```
- Một đầu nút → GPIO16
- Đầu kia → GND

### Bước 3: Kết Nối Bluetooth
1. Mở **Android App** (PoseLandmarker)
2. Vào **Settings** → Bluetooth
3. Tìm thiết bị: **ESP32_Gimbal_Test**
4. Nhấn **Connect**

Serial Monitor sẽ hiển thị:
```
✓ BLE Client Connected!
```

### Bước 4: Test Nút Nhấn
1. **Nhấn nút GPIO16**
2. Serial Monitor hiển thị:
   ```
   >>> BUTTON PRESSED!
   >>> Sent: CAPTURE
   ```
3. App sẽ:
   - Tự động chụp ảnh
   - Hiển thị Toast: "Remote Capture!"
   - Lưu ảnh vào `Pictures/PoseLandmarker/`

---

## ✅ Checklist

- [ ] ESP32 đã được cắm vào máy tính
- [ ] Code đã upload thành công
- [ ] Serial Monitor hiển thị "BLE Service started"
- [ ] Nút nhấn đã được kết nối (GPIO16 → GND)
- [ ] App tìm thấy "ESP32_Gimbal_Test"
- [ ] Bluetooth kết nối thành công
- [ ] Nhấn nút → App chụp ảnh

---

## 🐛 Xử Lý Lỗi

### ESP32 không hiển thị trong Bluetooth
- Reset ESP32 (nhấn nút EN)
- Kiểm tra Serial Monitor có "BLE Service started" không

### Nhấn nút không có phản ứng
- Kiểm tra kết nối: GPIO16 → Button → GND
- Kiểm tra Serial Monitor có "BUTTON PRESSED!" không

### App không chụp ảnh
- Kiểm tra quyền Camera và Storage
- Đảm bảo camera đang hoạt động (preview hiển thị)

---

## 📞 Thông Tin Kỹ Thuật

| Thông Số | Giá Trị |
|----------|---------|
| GPIO Pin | 16 |
| Baud Rate | 115200 |
| BLE Name | ESP32_Gimbal_Test |
| Command | CAPTURE |
| Debounce | 200ms |

---

**Chúc bạn test thành công! 🎉**

Nếu có vấn đề gì, hãy kiểm tra Serial Monitor để xem log chi tiết.
