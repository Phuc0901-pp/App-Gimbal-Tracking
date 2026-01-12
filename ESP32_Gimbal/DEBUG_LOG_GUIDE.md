# 📊 HƯỚNG DẪN ĐỌC LOG DEBUG

## 🎯 Mục Đích
Code đã được thêm nhiều LOG để giúp bạn debug dễ dàng hơn. File này giải thích ý nghĩa của từng loại LOG.

---

## 📝 Các Loại LOG

### 1. [SETUP] - Khởi Tạo Hệ Thống

Hiển thị khi ESP32 khởi động và cấu hình phần cứng.

```
[SETUP] Configuring GPIO16 as INPUT_PULLUP...
[SETUP] GPIO16 initial state: HIGH (not pressed)
✓ Button GPIO16 initialized (Pull-up)
```

**Ý nghĩa:**
- `GPIO16 initial state: HIGH` ✅ = Nút chưa nhấn, kết nối đúng
- `GPIO16 initial state: LOW` ⚠️ = Nút đang nhấn HOẶC bị short xuống GND

**Nếu thấy WARNING:**
```
[WARNING] GPIO16 is LOW! Check if button is pressed or shorted to GND
```
→ Kiểm tra lại kết nối phần cứng!

---

### 2. [SETUP] - Khởi Tạo Bluetooth

Hiển thị từng bước khởi động BLE:

```
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
```

**Kiểm tra:**
- ✅ Tất cả bước có dấu `✓` = Bluetooth đã sẵn sàng
- ❌ Nếu thiếu bất kỳ bước nào = Có lỗi khởi động BLE

---

### 3. Setup Complete - Tóm Tắt Trạng Thái

```
========================================
✓✓✓ SETUP COMPLETE ✓✓✓
========================================
BLE Status: ADVERTISING
Device Name: ESP32_Gimbal_Test
GPIO16 Status: READY
========================================
READY TO TEST!
1. Connect from Android app
2. Press button on GPIO16 to send CAPTURE
========================================
```

**Ý nghĩa:**
- Thấy message này = ESP32 đã sẵn sàng 100%
- Bây giờ có thể kết nối Bluetooth từ app

---

### 4. [INFO] - Thông Tin Chung

```
[INFO] Entering main loop...
[INFO] Monitoring GPIO16 state every 10ms
```

**Ý nghĩa:**
- ESP32 đã vào vòng lặp chính
- Đang kiểm tra GPIO16 mỗi 10ms

---

### 5. [GPIO] - Thay Đổi Trạng Thái GPIO

Hiển thị MỌI lần GPIO16 thay đổi trạng thái:

```
[GPIO] State changed: HIGH → LOW
[GPIO] State changed: LOW → HIGH
```

**Ý nghĩa:**
- `HIGH → LOW` = Bạn vừa NHẤN nút
- `LOW → HIGH` = Bạn vừa THẢ nút

**Nếu thấy nhiều lần liên tục:**
```
[GPIO] State changed: HIGH → LOW
[GPIO] State changed: LOW → HIGH
[GPIO] State changed: HIGH → LOW
[GPIO] State changed: LOW → HIGH
```
→ Nút bị nhiễu (bouncing) - Bình thường, debounce sẽ xử lý

---

### 6. >>> BUTTON PRESSED - Nút Được Nhấn

Hiển thị khi nút được nhấn SAU KHI debounce:

```
========================================
>>> BUTTON PRESSED!
>>> Timestamp: 12345
>>> BLE Connected: YES
>>> Preparing to send CAPTURE command...
>>> ✓ CAPTURE command sent via BLE
>>> Waiting for app response...
========================================
```

**Giải thích:**
- `Timestamp` = Thời gian (ms) kể từ khi ESP32 khởi động
- `BLE Connected: YES` = Đã kết nối với app, sẽ gửi lệnh
- `BLE Connected: NO` = Chưa kết nối, không gửi được

**Nếu chưa kết nối:**
```
========================================
>>> BUTTON PRESSED!
>>> Timestamp: 12345
>>> BLE Connected: NO
>>> ✗ BLE not connected, cannot send!
>>> Please connect from Android app first
========================================
```

---

### 7. [STATUS] - Trạng Thái Định Kỳ

Hiển thị MỖI 5 GIÂY để bạn biết ESP32 vẫn đang hoạt động:

```
[STATUS] ==================== Periodic Status ====================
[STATUS] Uptime: 25 seconds
[STATUS] BLE Connected: YES
[STATUS] GPIO16 State: HIGH (not pressed)
[STATUS] =========================================================
```

**Ý nghĩa:**
- `Uptime` = ESP32 đã chạy được bao lâu
- `BLE Connected` = Trạng thái kết nối Bluetooth
- `GPIO16 State` = Trạng thái nút nhấn hiện tại

**Sử dụng để:**
- Kiểm tra ESP32 không bị treo
- Theo dõi trạng thái BLE
- Kiểm tra GPIO có hoạt động không

---

## 🔍 Kịch Bản Debug Thường Gặp

### Kịch Bản 1: Bluetooth Không Khởi Động

**LOG bạn thấy:**
```
[SETUP] Initializing BLE...
[SETUP] Device name: ESP32_Gimbal_Test
[SETUP] ✓ BLE Device initialized
[SETUP] Creating BLE Server...
(Dừng ở đây, không có LOG tiếp theo)
```

**Nguyên nhân:**
- Lỗi khởi tạo BLE Server
- Có thể do lỗi phần mềm hoặc phần cứng

**Giải pháp:**
1. Reset ESP32 (nhấn nút EN)
2. Upload lại code
3. Kiểm tra nguồn cấp cho ESP32 (cáp USB tốt)

---

### Kịch Bản 2: GPIO16 Luôn Ở Trạng Thái LOW

**LOG bạn thấy:**
```
[SETUP] GPIO16 initial state: LOW (pressed or shorted)
[WARNING] GPIO16 is LOW! Check if button is pressed or shorted to GND
```

**Và trong periodic status:**
```
[STATUS] GPIO16 State: LOW (pressed)
```

**Nguyên nhân:**
- Nút đang bị nhấn
- GPIO16 bị short xuống GND
- Kết nối sai

**Giải pháp:**
1. Kiểm tra nút có đang bị nhấn không
2. Kiểm tra kết nối: GPIO16 → Button → GND
3. Dùng multimeter test continuity

---

### Kịch Bản 3: Nhấn Nút Nhưng Không Có LOG

**LOG bạn thấy:**
```
[STATUS] GPIO16 State: HIGH (not pressed)
(Nhấn nút nhưng không có [GPIO] State changed)
```

**Nguyên nhân:**
- Nút không được kết nối
- Nút bị hỏng
- Kết nối dây bị lỏng

**Giải pháp:**
1. Kiểm tra lại kết nối dây
2. Thử nút khác
3. Test bằng cách nối trực tiếp GPIO16 xuống GND bằng dây

---

### Kịch Bản 4: Nhấn Nút Có LOG Nhưng App Không Chụp

**LOG bạn thấy:**
```
>>> BUTTON PRESSED!
>>> BLE Connected: YES
>>> ✓ CAPTURE command sent via BLE
```

**Nhưng app không chụp ảnh**

**Nguyên nhân:**
- App không nhận được lệnh
- App chưa xử lý lệnh đúng
- Bluetooth bị mất kết nối giữa chừng

**Giải pháp:**
1. Kiểm tra Android Logcat
2. Đảm bảo app đang ở màn hình Camera
3. Thử kết nối lại Bluetooth

---

### Kịch Bản 5: BLE Không Kết Nối Được

**LOG bạn thấy:**
```
[STATUS] BLE Connected: NO
(Luôn là NO dù đã connect từ app)
```

**Nguyên nhân:**
- App kết nối sai UUID
- BLE callback không hoạt động
- App chưa kết nối đúng

**Giải pháp:**
1. Xem LOG khi connect, phải có:
   ```
   ✓ BLE Client Connected!
   ```
2. Nếu không có → Kiểm tra UUID trong app
3. Reset ESP32 và thử lại

---

## 📊 Ví Dụ LOG Hoàn Chỉnh

### Khi Mọi Thứ Hoạt Động Tốt:

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
READY TO TEST!
1. Connect from Android app
2. Press button on GPIO16 to send CAPTURE
========================================

[INFO] Entering main loop...
[INFO] Monitoring GPIO16 state every 10ms

[STATUS] ==================== Periodic Status ====================
[STATUS] Uptime: 5 seconds
[STATUS] BLE Connected: NO
[STATUS] GPIO16 State: HIGH (not pressed)
[STATUS] =========================================================

✓ BLE Client Connected!  ← App vừa kết nối

[STATUS] ==================== Periodic Status ====================
[STATUS] Uptime: 10 seconds
[STATUS] BLE Connected: YES  ← Đã kết nối
[STATUS] GPIO16 State: HIGH (not pressed)
[STATUS] =========================================================

[GPIO] State changed: HIGH → LOW  ← Nhấn nút

========================================
>>> BUTTON PRESSED!
>>> Timestamp: 12345
>>> BLE Connected: YES
>>> Preparing to send CAPTURE command...
>>> ✓ CAPTURE command sent via BLE
>>> Waiting for app response...
========================================

[GPIO] State changed: LOW → HIGH  ← Thả nút

[STATUS] ==================== Periodic Status ====================
[STATUS] Uptime: 15 seconds
[STATUS] BLE Connected: YES
[STATUS] GPIO16 State: HIGH (not pressed)
[STATUS] =========================================================
```

---

## ✅ Checklist Debug

Khi test, hãy kiểm tra các LOG sau:

### Khởi Động
- [ ] `[SETUP] GPIO16 initial state: HIGH` (không có WARNING)
- [ ] Tất cả BLE setup có dấu `✓`
- [ ] `✓✓✓ SETUP COMPLETE ✓✓✓`
- [ ] `[INFO] Entering main loop...`

### Kết Nối Bluetooth
- [ ] `✓ BLE Client Connected!` khi connect từ app
- [ ] `[STATUS] BLE Connected: YES` trong periodic status

### Test Nút Nhấn
- [ ] `[GPIO] State changed: HIGH → LOW` khi nhấn
- [ ] `>>> BUTTON PRESSED!`
- [ ] `>>> BLE Connected: YES`
- [ ] `>>> ✓ CAPTURE command sent via BLE`
- [ ] App chụp ảnh thành công

---

## 🎯 Tóm Tắt

| LOG Prefix | Ý Nghĩa | Khi Nào Hiển Thị |
|------------|---------|------------------|
| `[SETUP]` | Khởi tạo hệ thống | Lúc ESP32 boot |
| `[INFO]` | Thông tin chung | Khi vào main loop |
| `[GPIO]` | Thay đổi GPIO | Mỗi khi nút thay đổi trạng thái |
| `>>>` | Sự kiện nút nhấn | Sau debounce, khi nhấn nút |
| `[STATUS]` | Trạng thái định kỳ | Mỗi 5 giây |
| `[WARNING]` | Cảnh báo | Khi phát hiện vấn đề |
| `✓` | Thành công | Khi hoàn thành một bước |
| `✗` | Thất bại | Khi có lỗi |

---

**Với LOG chi tiết này, bạn có thể dễ dàng debug mọi vấn đề! 🎉**
