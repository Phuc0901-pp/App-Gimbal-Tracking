# 🧪 HƯỚNG DẪN TEST BLUETOOTH + BUTTON CAPTURE

## 📋 Mục Đích
Test kết nối Bluetooth BLE giữa ESP32 và Android App, với chức năng:
- Nhấn nút GPIO16 → ESP32 gửi lệnh "CAPTURE" → App chụp ảnh

---

## 🔌 Sơ Đồ Kết Nối Phần Cứng

```
ESP32 GPIO16 ----[Button]---- GND
                    |
              (Internal Pull-up)
```

**Chi tiết:**
- Một đầu nút nhấn nối với GPIO16
- Đầu kia nối với GND
- Khi nhấn nút: GPIO16 → LOW (trigger)
- Khi thả nút: GPIO16 → HIGH (pull-up)

---

## 📱 Chuẩn Bị

### 1. Phần Cứng
- [x] ESP32 Dev Board
- [x] Nút nhấn (tactile button)
- [x] Dây nối (jumper wires)
- [x] Cáp USB để upload code

### 2. Phần Mềm
- [x] PlatformIO đã cài đặt
- [x] Android App đã build và cài trên điện thoại
- [x] Code ESP32 đã được cập nhật (GPIO16)

---

## 🚀 Các Bước Thực Hiện

### Bước 1: Upload Code Lên ESP32

```bash
# Di chuyển vào thư mục project
cd "c:\Pham_Phuc\Mobile_app\mediapipe-samples\ESP32_Gimbal"

# Build và upload
pio run --target upload

# Mở Serial Monitor để xem log
pio device monitor
```

**Kết quả mong đợi trên Serial Monitor:**
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

### Bước 2: Kết Nối Bluetooth Từ App

1. **Mở Android App** (PoseLandmarker)
2. **Vào Settings** → Bluetooth
3. **Tìm thiết bị**: `ESP32_Gimbal_Test`
4. **Nhấn Connect**

**Kết quả mong đợi trên Serial Monitor:**
```
✓ BLE Client Connected!
```

---

### Bước 3: Test Nút Nhấn

1. **Nhấn nút GPIO16** trên ESP32
2. **Quan sát Serial Monitor:**
   ```
   >>> BUTTON PRESSED!
   >>> Sent: CAPTURE
   ```

3. **Quan sát trên App:**
   - App sẽ tự động chụp ảnh
   - Hiển thị Toast: "Remote Capture!"
   - Ảnh được lưu vào thư mục `Pictures/PoseLandmarker/`

---

## ✅ Checklist Kiểm Tra

### ESP32
- [ ] Serial Monitor hiển thị "BLE Service started"
- [ ] Serial Monitor hiển thị "BLE Client Connected" khi app kết nối
- [ ] Nhấn nút → Serial hiển thị "BUTTON PRESSED!" và "Sent: CAPTURE"
- [ ] Không có lỗi hoặc crash

### Android App
- [ ] App tìm thấy thiết bị "ESP32_Gimbal_Test"
- [ ] Kết nối Bluetooth thành công
- [ ] Nhấn nút → App chụp ảnh ngay lập tức
- [ ] Toast hiển thị "Remote Capture!"
- [ ] Ảnh được lưu thành công

---

## 🐛 Xử Lý Lỗi Thường Gặp

### Lỗi 1: ESP32 không hiển thị trong danh sách Bluetooth

**Nguyên nhân:**
- BLE chưa khởi động đúng
- App đang tìm Classic Bluetooth thay vì BLE

**Giải pháp:**
1. Reset ESP32 (nhấn nút EN)
2. Kiểm tra Serial Monitor xem có "BLE Service started" không
3. Đảm bảo app đang scan BLE devices

---

### Lỗi 2: Nhấn nút nhưng không gửi CAPTURE

**Nguyên nhân:**
- Nút chưa được kết nối đúng
- Bluetooth chưa kết nối

**Giải pháp:**
1. Kiểm tra kết nối phần cứng (GPIO16 → Button → GND)
2. Kiểm tra Serial Monitor:
   - Nếu hiển thị "BLE not connected" → Kết nối lại Bluetooth
   - Nếu không hiển thị gì → Kiểm tra lại dây nối

---

### Lỗi 3: App nhận lệnh nhưng không chụp ảnh

**Nguyên nhân:**
- Camera chưa được khởi động
- Không có quyền truy cập camera/storage

**Giải pháp:**
1. Kiểm tra quyền app trong Settings → Apps → PoseLandmarker
2. Đảm bảo camera đang hoạt động (preview hiển thị)
3. Kiểm tra logcat để xem lỗi chi tiết

---

## 📊 Thông Số Kỹ Thuật

| Thông Số | Giá Trị |
|----------|---------|
| GPIO Pin | 16 |
| Debounce Time | 200ms |
| BLE Device Name | ESP32_Gimbal_Test |
| Service UUID | 4fafc201-1fb5-459e-8fcc-c5c9c331914b |
| Characteristic UUID | beb5483e-36e1-4688-b7f5-ea07361b26a8 |
| Capture Command | "CAPTURE" |
| Min Capture Interval | 2000ms (2 giây) |

---

## 🔄 Sau Khi Test Xong

Nếu test thành công, bạn có thể:

1. **Tích hợp vào code chính:**
   - Copy logic button từ `test_bluetooth_button.cpp`
   - Paste vào file gimbal chính của bạn
   - Chỉnh sửa `platformio.ini` để build file chính

2. **Thay đổi GPIO nếu cần:**
   - Sửa `BUTTON_PIN` trong code
   - Kết nối lại phần cứng

3. **Tùy chỉnh debounce:**
   - Sửa `DEBOUNCE_DELAY` nếu nút bị nhấn nhiều lần

---

## 📝 Ghi Chú

- **Debounce 200ms**: Ngăn chặn việc gửi nhiều lệnh CAPTURE khi nhấn nút một lần
- **App debounce 2000ms**: App chỉ chụp ảnh tối đa 1 lần/2 giây
- **Pull-up resistor**: ESP32 có internal pull-up, không cần thêm điện trở ngoài

---

## 📞 Hỗ Trợ

Nếu gặp vấn đề, hãy kiểm tra:
1. Serial Monitor của ESP32
2. Logcat của Android App
3. Kết nối phần cứng (dùng multimeter để test)

**Chúc bạn test thành công! 🎉**
