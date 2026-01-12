# ✅ TÓM TẮT - CHUẨN BỊ TEST BLUETOOTH + BUTTON CAPTURE

## 🎯 Mục Tiêu
Test chức năng: **Nhấn nút GPIO16 → ESP32 gửi "CAPTURE" qua Bluetooth → App chụp ảnh**

---

## 📦 Những Gì Đã Chuẩn Bị

### 1. Code ESP32 ✅
- **File:** `src/test_bluetooth_button.cpp`
- **GPIO:** GPIO16 (đã thay đổi từ GPIO19)
- **Chức năng:**
  - Khởi động Bluetooth BLE với tên `ESP32_Gimbal_Test`
  - Đọc trạng thái nút GPIO16
  - Gửi lệnh "CAPTURE" khi nút được nhấn
  - Debounce 200ms để tránh nhiễu

### 2. Cấu Hình PlatformIO ✅
- **File:** `platformio.ini`
- **Board:** ESP32 Dev
- **Baud rate:** 115200
- **Build filter:** Chỉ build file test
- **Libraries:** ESP32Servo, ArduinoBLE

### 3. Android App ✅
- **File:** `Constants.kt`
- **Constant:** `REMOTE_CAPTURE_COMMAND = "CAPTURE"`
- **Debounce:** 2000ms (2 giây)
- **Handler:** Đã có sẵn trong `CameraFragment.kt`

### 4. Tài Liệu Hướng Dẫn ✅
- **QUICK_START.md** - Hướng dẫn upload và test nhanh
- **TEST_INSTRUCTIONS.md** - Hướng dẫn chi tiết từng bước
- **WIRING_DIAGRAM.md** - Sơ đồ kết nối phần cứng
- **SUMMARY.md** - File này (tóm tắt)

---

## 🔌 Kết Nối Phần Cứng

```
ESP32 GPIO16 ────[Button]──── GND
                    |
              (Internal Pull-up)
```

**Linh kiện cần:**
- ESP32 Dev Board
- Nút nhấn tactile (6x6mm hoặc 12x12mm)
- 2 dây jumper
- Cáp USB

---

## 🚀 Các Bước Thực Hiện

### Bước 1: Upload Code
**Cách 1 - PlatformIO (VS Code):**
```bash
# Mở project trong VS Code
# Nhấn nút Upload (→) trên thanh PlatformIO
# Hoặc Ctrl+Alt+U
```

**Cách 2 - Arduino IDE:**
1. Mở file `test_bluetooth_button.cpp`
2. Chọn Board: ESP32 Dev Module
3. Chọn Port: COM port của ESP32
4. Nhấn Upload

### Bước 2: Kết Nối Phần Cứng
1. GPIO16 → Chân 1 của nút
2. Chân 2 của nút → GND

### Bước 3: Kiểm Tra Serial Monitor
Sau khi upload, Serial Monitor sẽ hiển thị:
```
========================================
ESP32 BLUETOOTH + BUTTON TEST
========================================
✓ Button GPIO16 initialized (Pull-up)
✓ BLE Service started
✓ Advertising as: ESP32_Gimbal_Test
========================================
READY!
========================================
```

### Bước 4: Kết Nối Bluetooth
1. Mở Android App
2. Settings → Bluetooth
3. Tìm "ESP32_Gimbal_Test"
4. Connect

Serial Monitor sẽ hiển thị:
```
✓ BLE Client Connected!
```

### Bước 5: Test Nút Nhấn
1. Nhấn nút GPIO16
2. Serial Monitor hiển thị:
   ```
   >>> BUTTON PRESSED!
   >>> Sent: CAPTURE
   ```
3. App tự động chụp ảnh và hiển thị Toast

---

## ✅ Checklist Trước Khi Test

### Phần Cứng
- [ ] ESP32 đã cắm USB vào máy tính
- [ ] Nút nhấn đã kết nối đúng (GPIO16 ↔ GND)
- [ ] Kiểm tra kết nối bằng multimeter (tùy chọn)

### Phần Mềm ESP32
- [ ] Code đã upload thành công
- [ ] Serial Monitor mở và baud rate = 115200
- [ ] Serial hiển thị "BLE Service started"
- [ ] Serial hiển thị "READY!"

### Phần Mềm Android
- [ ] App đã build và cài đặt
- [ ] Quyền Camera và Storage đã được cấp
- [ ] Bluetooth đã bật
- [ ] App đang ở màn hình Camera

---

## 🎯 Kết Quả Mong Đợi

### Khi Nhấn Nút GPIO16:

**ESP32 (Serial Monitor):**
```
>>> BUTTON PRESSED!
>>> Sent: CAPTURE
```

**Android App:**
- ✅ Chụp ảnh ngay lập tức
- ✅ Hiển thị Toast: "Remote Capture!"
- ✅ Ảnh được lưu vào `Pictures/PoseLandmarker/`
- ✅ Có hiệu ứng flash trắng

---

## 🐛 Xử Lý Lỗi Nhanh

| Vấn Đề | Nguyên Nhân | Giải Pháp |
|--------|-------------|-----------|
| ESP32 không hiển thị trong Bluetooth | BLE chưa khởi động | Reset ESP32, kiểm tra Serial Monitor |
| Nhấn nút không có phản ứng | Kết nối sai | Kiểm tra GPIO16 → Button → GND |
| App không chụp ảnh | Không có quyền | Cấp quyền Camera/Storage |
| Serial Monitor trống | Baud rate sai | Đặt baud rate = 115200 |

---

## 📊 Thông Số Kỹ Thuật

| Thông Số | Giá Trị |
|----------|---------|
| **ESP32** | |
| GPIO Pin | 16 |
| Debounce Time | 200ms |
| Baud Rate | 115200 |
| BLE Device Name | ESP32_Gimbal_Test |
| **Bluetooth** | |
| Service UUID | 4fafc201-1fb5-459e-8fcc-c5c9c331914b |
| Characteristic UUID | beb5483e-36e1-4688-b7f5-ea07361b26a8 |
| **Android** | |
| Capture Command | "CAPTURE" |
| Min Capture Interval | 2000ms (2 giây) |

---

## 📁 Cấu Trúc File

```
ESP32_Gimbal/
├── src/
│   ├── test_bluetooth_button.cpp  ← Code test chính
│   └── esp32_capture_button.cpp   ← Code tham khảo
├── platformio.ini                 ← Cấu hình PlatformIO
├── QUICK_START.md                 ← Hướng dẫn nhanh
├── TEST_INSTRUCTIONS.md           ← Hướng dẫn chi tiết
├── WIRING_DIAGRAM.md              ← Sơ đồ kết nối
└── SUMMARY.md                     ← File này
```

---

## 🔄 Sau Khi Test

### Nếu Test Thành Công ✅
1. **Tích hợp vào code chính:**
   - Copy logic button từ `test_bluetooth_button.cpp`
   - Paste vào file gimbal chính
   - Chỉnh sửa `platformio.ini` để build file chính

2. **Tùy chỉnh:**
   - Thay đổi GPIO nếu cần
   - Điều chỉnh debounce time
   - Thêm các lệnh khác (RECORD, STOP, etc.)

### Nếu Test Thất Bại ❌
1. **Kiểm tra Serial Monitor:**
   - Có "BLE Service started" không?
   - Có "BLE Client Connected" không?
   - Có "BUTTON PRESSED!" khi nhấn nút không?

2. **Kiểm tra phần cứng:**
   - Dùng multimeter test kết nối
   - Thử nút khác nếu nghi ngờ nút bị hỏng

3. **Kiểm tra app:**
   - Xem logcat để tìm lỗi
   - Đảm bảo quyền đã được cấp

---

## 📞 Hỗ Trợ

### Đọc Tài Liệu
- **QUICK_START.md** - Bắt đầu nhanh
- **TEST_INSTRUCTIONS.md** - Hướng dẫn đầy đủ
- **WIRING_DIAGRAM.md** - Sơ đồ chi tiết

### Debug
1. Kiểm tra Serial Monitor
2. Kiểm tra Android Logcat
3. Test từng phần riêng biệt

---

## 🎉 Sẵn Sàng Test!

Bạn đã có đầy đủ:
- ✅ Code ESP32 (GPIO16)
- ✅ Code Android (handler CAPTURE)
- ✅ Hướng dẫn chi tiết
- ✅ Sơ đồ kết nối

**Bước tiếp theo:**
1. Đọc `QUICK_START.md`
2. Upload code lên ESP32
3. Kết nối phần cứng
4. Test!

**Chúc bạn thành công! 🚀**
