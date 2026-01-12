# 🔌 SƠ ĐỒ KẾT NỐI PHẦN CỨNG

## Sơ Đồ Kết Nối ESP32 + Button

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32 Dev Board                      │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                                                      │  │
│  │   GPIO16 ●────────────────────┐                     │  │
│  │                                │                     │  │
│  │                                │                     │  │
│  │                                │                     │  │
│  │                                │                     │  │
│  │                                │                     │  │
│  │                                │                     │  │
│  │                                │                     │  │
│  │    GND   ●────────────────┐   │                     │  │
│  │                           │   │                     │  │
│  └───────────────────────────┼───┼─────────────────────┘  │
│                              │   │                        │
└──────────────────────────────┼───┼────────────────────────┘
                               │   │
                               │   │
                               │   │    ┌──────────────┐
                               │   └────┤              │
                               │        │    BUTTON    │
                               └────────┤  (Tactile)   │
                                        │              │
                                        └──────────────┘

```

## Chi Tiết Kết Nối

### Kết Nối 1: GPIO16 → Button
- **Từ:** ESP32 GPIO16
- **Đến:** Chân 1 của nút nhấn
- **Chức năng:** Input signal (với internal pull-up)

### Kết Nối 2: Button → GND
- **Từ:** Chân 2 của nút nhấn
- **Đến:** ESP32 GND
- **Chức năng:** Ground reference

---

## Nguyên Lý Hoạt Động

### Trạng Thái Không Nhấn (Default)
```
GPIO16: HIGH (3.3V) ←── Internal Pull-up Resistor
  │
  │
  ├─── Button (OPEN) ───X
  │
GND: 0V
```
**Kết quả:** `digitalRead(GPIO16) = HIGH`

### Trạng Thái Nhấn Nút
```
GPIO16: HIGH (3.3V) ←── Internal Pull-up Resistor
  │
  │
  ├─── Button (CLOSED) ───┐
  │                        │
  └────────────────────────┘
  │
GND: 0V
```
**Kết quả:** `digitalRead(GPIO16) = LOW` → **TRIGGER!**

---

## Danh Sách Linh Kiện

| STT | Linh Kiện | Số Lượng | Ghi Chú |
|-----|-----------|----------|---------|
| 1 | ESP32 Dev Board | 1 | Bất kỳ board ESP32 nào |
| 2 | Tactile Button | 1 | 6x6mm hoặc 12x12mm |
| 3 | Jumper Wires | 2 | Male-to-Male hoặc Male-to-Female |
| 4 | Breadboard | 1 | (Tùy chọn, để dễ kết nối) |

---

## Hình Ảnh Thực Tế

### Nút Nhấn Tactile
```
     ┌───┐
  ───┤   ├───  Chân 1 & 2 (cùng bên)
     │   │
  ───┤   ├───  Chân 3 & 4 (cùng bên)
     └───┘

Lưu ý: Chân 1-2 luôn nối với nhau
       Chân 3-4 luôn nối với nhau
       Khi nhấn: 1-2 nối với 3-4
```

### Pinout ESP32 (Tham khảo)
```
                    ┌─────────┐
                    │   USB   │
                    └─────────┘
                         │
    ┌────────────────────┴────────────────────┐
    │                                         │
    │  3V3  ●                          ● GND  │
    │  EN   ●                          ● D23  │
    │  VP   ●                          ● D22  │
    │  VN   ●                          ● TX   │
    │  D34  ●                          ● RX   │
    │  D35  ●                          ● D21  │
    │  D32  ●                          ● D19  │
    │  D33  ●                          ● D18  │
    │  D25  ●                          ● D5   │
    │  D26  ●                          ● D17  │
    │  D27  ●                          ● D16  │ ← GPIO16 (Capture Button)
    │  D14  ●                          ● D4   │
    │  D12  ●                          ● D0   │
    │  D13  ●                          ● D2   │
    │  GND  ●                          ● D15  │ ← GND (Button Ground)
    │  VIN  ●                          ● D8   │
    │                                         │
    └─────────────────────────────────────────┘
```

---

## Các Bước Kết Nối

### Bước 1: Chuẩn Bị
1. Tắt nguồn ESP32 (rút USB)
2. Chuẩn bị 2 dây jumper
3. Chuẩn bị nút nhấn tactile

### Bước 2: Kết Nối
1. **Dây 1:** GPIO16 → Chân 1 của nút
2. **Dây 2:** Chân 2 của nút → GND

### Bước 3: Kiểm Tra
1. Kiểm tra lại kết nối
2. Đảm bảo không có chập mạch
3. Cắm USB vào ESP32

### Bước 4: Test
1. Upload code test
2. Mở Serial Monitor
3. Nhấn nút → Xem kết quả

---

## Lưu Ý Quan Trọng

### ✅ Nên Làm
- Sử dụng internal pull-up (đã có trong code)
- Kiểm tra kết nối trước khi cấp nguồn
- Sử dụng debounce trong code (đã có)

### ❌ Không Nên
- Không cần thêm điện trở pull-up ngoài
- Không nối GPIO16 trực tiếp với 3.3V
- Không sử dụng điện áp > 3.3V cho ESP32

---

## Kiểm Tra Kết Nối

### Test Bằng Multimeter
1. **Chế độ:** Continuity (beep)
2. **Test 1:** GPIO16 ↔ Chân 1 nút (phải beep)
3. **Test 2:** Chân 2 nút ↔ GND (phải beep)
4. **Test 3:** Nhấn nút → GPIO16 ↔ GND (phải beep khi nhấn)

### Test Bằng Code
```cpp
void setup() {
  Serial.begin(115200);
  pinMode(16, INPUT_PULLUP);
}

void loop() {
  int state = digitalRead(16);
  Serial.println(state);  // HIGH = 1, LOW = 0
  delay(100);
}
```
- Không nhấn: In ra `1` (HIGH)
- Nhấn nút: In ra `0` (LOW)

---

## Sơ Đồ Luồng Hoạt Động

```
┌─────────────────┐
│  Nhấn Nút GPIO16│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ GPIO16 → LOW    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Debounce 200ms  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Kiểm tra BLE    │
│ Connected?      │
└────┬───────┬────┘
     │       │
    YES     NO
     │       │
     ▼       ▼
┌─────────┐ ┌──────────────┐
│ Gửi     │ │ In "BLE not  │
│ CAPTURE │ │ connected"   │
└────┬────┘ └──────────────┘
     │
     ▼
┌─────────────────┐
│ BLE Notify      │
│ "CAPTURE"       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ App Nhận Lệnh   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ App Chụp Ảnh    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Lưu Ảnh        │
│ + Toast        │
└─────────────────┘
```

---

**Hoàn thành! Bây giờ bạn có thể bắt đầu kết nối phần cứng! 🎉**
