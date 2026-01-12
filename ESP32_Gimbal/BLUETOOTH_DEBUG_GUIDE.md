# 🔍 HƯỚNG DẪN DEBUG BLUETOOTH CAPTURE

## 🎯 Mục Đích
Hướng dẫn này giúp bạn debug kết nối Bluetooth giữa ESP32 và Android App để đảm bảo lệnh CAPTURE được gửi và nhận đúng.

---

## 📊 Luồng Hoạt Động

```
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│   Nhấn Nút   │   BLE   │   ESP32 Gửi  │   BLE   │  App Nhận    │
│   GPIO16     │────────▶│   "CAPTURE"  │────────▶│  & Chụp Ảnh  │
└──────────────┘         └──────────────┘         └──────────────┘
```

---

## 🔧 Bước 1: Kiểm Tra ESP32

### Upload Code Mới (Đã Có LOG)

Code ESP32 đã được thêm nhiều LOG. Hãy upload lại:

```bash
# Build
C:\Users\USER\.platformio\penv\Scripts\platformio.exe run

# Upload
C:\Users\USER\.platformio\penv\Scripts\platformio.exe run --target upload

# Monitor
C:\Users\USER\.platformio\penv\Scripts\platformio.exe device monitor
```

### Kiểm Tra LOG ESP32

Sau khi upload, Serial Monitor sẽ hiển thị:

```
========================================
ESP32 BLUETOOTH + BUTTON TEST
========================================

[SETUP] Configuring GPIO16 as INPUT_PULLUP...
[SETUP] GPIO16 initial state: HIGH (not pressed)
✓ Button GPIO16 initialized (Pull-up)

[SETUP] Initializing BLE...
[SETUP] Device name: ESP32_Gimbal_Test
[SETUP] ✓ BLE Device initialized
[SETUP] Creating BLE Server...
[SETUP] ✓ BLE Server created
[SETUP] Creating BLE Service with UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
[SETUP] ✓ BLE Service created
[SETUP] Creating BLE Characteristic with UUID: beb5483e-36e1-4688-b7f5-ea07361b26a8
[SETUP] ✓ BLE Characteristic created
[SETUP] Adding CCCD descriptor...
[SETUP] ✓ Descriptor and callbacks added
[SETUP] Starting BLE Service...
[SETUP] ✓ BLE Service started
[SETUP] Configuring BLE Advertising...
[SETUP] Starting BLE Advertising...
[SETUP] ✓ BLE Advertising started

========================================
✓✓✓ SETUP COMPLETE ✓✓✓
========================================
BLE Status: ADVERTISING
Device Name: ESP32_Gimbal_Test
GPIO16 Status: READY
========================================
```

**✅ Checklist:**
- [ ] Tất cả bước có dấu `✓`
- [ ] `GPIO16 initial state: HIGH`
- [ ] `BLE Status: ADVERTISING`
- [ ] Không có lỗi hoặc WARNING

---

## 📱 Bước 2: Kiểm Tra Android App

### Build App Mới (Đã Có LOG)

App Android đã được thêm LOG vào `CameraFragment.kt`. Hãy build lại:

1. Mở Android Studio
2. Build → Rebuild Project
3. Run → Run 'app'
4. Cài đặt lên điện thoại

### Kết Nối Bluetooth

1. Mở app
2. Nhấn nút **Bluetooth** (icon Bluetooth)
3. Tìm thiết bị **ESP32_Gimbal_Test**
4. Nhấn để kết nối

**Kiểm tra ESP32 Serial Monitor:**
```
✓ BLE Client Connected!
```

**Kiểm tra App:**
- Màn hình hiển thị: `BT: ESP32_Gimbal_Test` (màu xanh)

---

## 🧪 Bước 3: Test Gửi Lệnh CAPTURE

### Nhấn Nút GPIO16

**ESP32 Serial Monitor sẽ hiển thị:**
```
[GPIO] State changed: HIGH → LOW

========================================
>>> BUTTON PRESSED!
>>> Timestamp: 12345
>>> BLE Connected: YES
>>> Preparing to send CAPTURE command...
>>> ✓ CAPTURE command sent via BLE
>>> Waiting for app response...
========================================

[GPIO] State changed: LOW → HIGH
```

### Kiểm Tra Android Logcat

Mở **Android Studio → Logcat** và filter theo tag `BLE_DEBUG`:

**Nếu nhận được lệnh, bạn sẽ thấy:**
```
D/BLE_DEBUG: ========== BLE Data Received ==========
D/BLE_DEBUG: Raw data: 'CAPTURE'
D/BLE_DEBUG: Data length: 7
D/BLE_DEBUG: Contains CAPTURE: true
D/BLE_DEBUG: Time since last capture: 5234ms
D/BLE_DEBUG: Min interval: 2000ms
D/BLE_DEBUG: ✓ CAPTURE command accepted! Taking photo...
D/BLE_DEBUG: =======================================
```

**App sẽ:**
- ✅ Chụp ảnh
- ✅ Hiển thị Toast: "Remote Capture!"
- ✅ Flash màn hình trắng
- ✅ Lưu ảnh vào `Pictures/PoseLandmarker/`

---

## 🐛 Xử Lý Lỗi

### Lỗi 1: ESP32 Gửi Nhưng App Không Nhận

**Triệu chứng:**
- ESP32 hiển thị: `✓ CAPTURE command sent via BLE`
- Logcat KHÔNG có `BLE Data Received`

**Nguyên nhân:**
- Bluetooth bị mất kết nối
- UUID không khớp
- App không đăng ký nhận notification

**Giải pháp:**
1. **Kiểm tra kết nối:**
   - ESP32 Serial: Phải có `✓ BLE Client Connected!`
   - App: Phải hiển thị `BT: ESP32_Gimbal_Test` màu xanh

2. **Kết nối lại:**
   - Disconnect và connect lại từ app
   - Reset ESP32 (nhấn nút EN)

3. **Kiểm tra UUID:**
   - ESP32: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
   - App: Phải giống nhau

---

### Lỗi 2: App Nhận Nhưng Không Chụp Ảnh

**Triệu chứng:**
- Logcat hiển thị: `BLE Data Received`
- Logcat hiển thị: `Contains CAPTURE: true`
- Nhưng KHÔNG có: `✓ CAPTURE command accepted!`

**Nguyên nhân:**
- Debounce: Nhấn nút quá nhanh (< 2 giây)

**Logcat sẽ hiển thị:**
```
D/BLE_DEBUG: ✗ CAPTURE command rejected - too soon (debounce)
```

**Giải pháp:**
- Đợi ít nhất 2 giây giữa các lần nhấn nút

---

### Lỗi 3: App Nhận Data Nhưng Không Phải "CAPTURE"

**Triệu chứng:**
- Logcat hiển thị: `BLE Data Received`
- Logcat hiển thị: `Contains CAPTURE: false`
- Logcat hiển thị: `Data is not a CAPTURE command`

**Nguyên nhân:**
- ESP32 gửi data khác (tracking data)
- Lệnh CAPTURE bị sai

**Logcat ví dụ:**
```
D/BLE_DEBUG: Raw data: '{{[x]:10;[y]:20;[pan]:5;[tilt]:-3}}'
D/BLE_DEBUG: Contains CAPTURE: false
D/BLE_DEBUG: Data is not a CAPTURE command
```

**Giải pháp:**
- Kiểm tra ESP32 có gửi đúng "CAPTURE" không
- Kiểm tra Constants.kt: `REMOTE_CAPTURE_COMMAND = "CAPTURE"`

---

### Lỗi 4: App Chụp Ảnh Nhưng Không Lưu

**Triệu chứng:**
- Logcat hiển thị: `✓ CAPTURE command accepted!`
- Toast hiển thị: "Remote Capture!"
- Flash màn hình
- Nhưng không tìm thấy ảnh

**Nguyên nhân:**
- Không có quyền WRITE_EXTERNAL_STORAGE
- Lỗi khi lưu file

**Giải pháp:**
1. **Kiểm tra quyền:**
   - Settings → Apps → PoseLandmarker → Permissions
   - Đảm bảo Camera và Storage được cấp

2. **Kiểm tra Logcat:**
   - Tìm tag `ImageCapture`
   - Xem có lỗi gì không

---

## 📊 Bảng So Sánh LOG

| Tình Huống | ESP32 Serial | Android Logcat | Kết Quả |
|------------|--------------|----------------|---------|
| **Thành công** | `✓ CAPTURE command sent` | `✓ CAPTURE command accepted!` | ✅ Chụp ảnh |
| **Chưa kết nối BLE** | `✗ BLE not connected` | (Không có) | ❌ Không gửi |
| **Debounce** | `✓ CAPTURE command sent` | `✗ CAPTURE command rejected` | ❌ Không chụp |
| **Mất kết nối** | `✓ CAPTURE command sent` | (Không có) | ❌ Không nhận |
| **Data khác** | (Gửi tracking data) | `Data is not a CAPTURE command` | ❌ Không chụp |

---

## 🔍 Cách Xem Logcat

### Trong Android Studio:

1. **Mở Logcat:**
   - View → Tool Windows → Logcat
   - Hoặc nhấn `Alt+6`

2. **Filter theo tag:**
   - Trong ô search, gõ: `tag:BLE_DEBUG`
   - Hoặc: `BLE_DEBUG`

3. **Filter theo level:**
   - Chọn "Debug" trong dropdown

### Qua ADB Command Line:

```bash
# Xem tất cả log BLE_DEBUG
adb logcat -s BLE_DEBUG

# Xem log realtime
adb logcat -s BLE_DEBUG:D

# Lưu log vào file
adb logcat -s BLE_DEBUG > ble_debug.log
```

---

## ✅ Checklist Debug Hoàn Chỉnh

### ESP32
- [ ] Code đã upload thành công
- [ ] Serial Monitor hiển thị "SETUP COMPLETE"
- [ ] GPIO16 initial state = HIGH
- [ ] BLE Advertising started
- [ ] Nhấn nút → `[GPIO] State changed: HIGH → LOW`
- [ ] `>>> BUTTON PRESSED!`
- [ ] `>>> BLE Connected: YES`
- [ ] `>>> ✓ CAPTURE command sent via BLE`

### Android App
- [ ] App đã build và cài đặt
- [ ] Kết nối Bluetooth thành công
- [ ] Màn hình hiển thị `BT: ESP32_Gimbal_Test` (xanh)
- [ ] Logcat filter `BLE_DEBUG` đã bật
- [ ] Nhấn nút → Logcat hiển thị `BLE Data Received`
- [ ] Logcat hiển thị `Contains CAPTURE: true`
- [ ] Logcat hiển thị `✓ CAPTURE command accepted!`
- [ ] App chụp ảnh + Toast + Flash
- [ ] Ảnh được lưu thành công

---

## 📝 Ví Dụ LOG Hoàn Chỉnh

### Khi Mọi Thứ Hoạt Động Tốt:

**ESP32 Serial Monitor:**
```
[STATUS] ==================== Periodic Status ====================
[STATUS] Uptime: 15 seconds
[STATUS] BLE Connected: YES
[STATUS] GPIO16 State: HIGH (not pressed)
[STATUS] =========================================================

[GPIO] State changed: HIGH → LOW

========================================
>>> BUTTON PRESSED!
>>> Timestamp: 15234
>>> BLE Connected: YES
>>> Preparing to send CAPTURE command...
>>> ✓ CAPTURE command sent via BLE
>>> Waiting for app response...
========================================

[GPIO] State changed: LOW → HIGH
```

**Android Logcat:**
```
D/BLE_DEBUG: ========== BLE Data Received ==========
D/BLE_DEBUG: Raw data: 'CAPTURE'
D/BLE_DEBUG: Data length: 7
D/BLE_DEBUG: Contains CAPTURE: true
D/BLE_DEBUG: Time since last capture: 15234ms
D/BLE_DEBUG: Min interval: 2000ms
D/BLE_DEBUG: ✓ CAPTURE command accepted! Taking photo...
D/BLE_DEBUG: =======================================
```

**App UI:**
- Toast: "Remote Capture!"
- Flash màn hình trắng
- Ảnh lưu vào `Pictures/PoseLandmarker/PoseLandmarker_1703592345678.jpg`

---

## 🎯 Tóm Tắt

| Bước | ESP32 | Android | Kết Quả |
|------|-------|---------|---------|
| 1 | Upload code + Monitor | Build app + Install | Chuẩn bị |
| 2 | Advertising | Connect Bluetooth | Kết nối |
| 3 | Nhấn nút GPIO16 | - | Trigger |
| 4 | Gửi "CAPTURE" | Nhận "CAPTURE" | Truyền data |
| 5 | - | Chụp ảnh | Hoàn thành |

---

**Với LOG chi tiết này, bạn có thể dễ dàng tìm ra vấn đề! 🎉**

Nếu vẫn gặp lỗi, hãy gửi cho tôi:
1. ESP32 Serial Monitor log
2. Android Logcat (filter BLE_DEBUG)
3. Mô tả chi tiết vấn đề
