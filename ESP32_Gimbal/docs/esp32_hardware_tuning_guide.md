# ESP32 Hardware Auto-Tuning Guide
## Tự Động Tìm PID Trực Tiếp Trên Gimbal

## 🎯 Ưu Điểm

✅ **Test trên hardware thật** - Không cần simulation  
✅ **Tự động hoàn toàn** - Không cần manual tuning  
✅ **Chính xác** - Dựa trên oscillation thực tế  
✅ **Nhanh** - Chỉ mất ~30 giây mỗi motor  

## 🚀 Cách Sử Dụng

### Bước 1: Upload Code

```bash
# Mở file: esp32_auto_tuning_hardware.ino
# Upload lên ESP32 (Arduino IDE hoặc PlatformIO)
```

### Bước 2: Mở Serial Monitor

```
Baud rate: 115200
```

Bạn sẽ thấy:
```
========================================
ESP32 GIMBAL AUTO-TUNING
========================================
Commands:
  T - Auto-tune Tilt motor
  P - Auto-tune Pan motor
  H - Hold current position (test)
========================================

Motors initialized. Ready for tuning!
Send 'T' or 'P' to start...
```

### Bước 3: Tune Tilt Motor

**Gửi:** `T` (hoặc `t`)

**Quá trình:**
1. Motor sẽ oscillate (dao động) ±10°
2. System đo period và amplitude
3. Tính toán Ku (ultimate gain)
4. Apply Ziegler-Nichols rules
5. Hiển thị PID tối ưu

**Output mẫu:**
```
========================================
AUTO-TUNING TILT MOTOR
========================================
Starting relay test...
Peak 1 detected at 100.23°
Peak 2 detected at 99.87°
Peak 3 detected at 100.15°
Peak 4 detected at 99.92°
Peak 5 detected at 100.08°

=== OSCILLATION ANALYSIS ===
Period (Tu): 0.850 seconds
Amplitude: 10.15 degrees
Ultimate Gain (Ku): 12.5432

=== OPTIMAL PID (Ziegler-Nichols) ===
Kp = 7.5259
Ki = 0.1062
Kd = 0.7978

=== COPY TO YOUR CODE ===
PID_Controller pidTilt(
    7.5259f,  // Kp
    0.1062f,  // Ki
    0.7978f,  // Kd
    90.00f * DEG_TO_RAD
);

Auto-tuning complete!
========================================
```

### Bước 4: Tune Pan Motor

**Gửi:** `P` (hoặc `p`)

Tương tự như Tilt, nhưng cho Pan motor.

### Bước 5: Copy PID Values

Copy code snippet từ Serial Monitor vào main code của bạn:

```cpp
// Main code
PID_Controller pidTilt(
    7.5259f,  // Kp - từ auto-tuning
    0.1062f,  // Ki
    0.7978f,  // Kd
    90.0f * DEG_TO_RAD
);

PID_Controller pidPan(
    6.2341f,  // Kp - từ auto-tuning
    0.0892f,  // Ki
    0.6543f,  // Kd
    0.0f
);
```

## 🔬 Cách Hoạt Động (Ziegler-Nichols Method)

### Step 1: Relay Test
```
Motor position oscillates:
  90° → 100° → 80° → 100° → 80° → ...
```

### Step 2: Measure Oscillation
- **Period (Tu):** Thời gian 1 chu kỳ dao động
- **Amplitude:** Độ lớn dao động

### Step 3: Calculate Ultimate Gain
```
Ku = (4 × relay_amplitude) / (π × oscillation_amplitude)
```

### Step 4: Ziegler-Nichols Rules
```
Kp = 0.6 × Ku
Ki = 1.2 × Ku / Tu  (reduced 10% for gimbal)
Kd = 0.075 × Ku × Tu
```

## ⚙️ Tuning Parameters

Nếu kết quả không tốt, điều chỉnh trong code:

```cpp
// Line ~30
float test_amplitude = 10.0f * DEG_TO_RAD;  // ±10° test

// Nếu oscillation quá nhỏ → Tăng lên 15°
// Nếu oscillation quá lớn → Giảm xuống 5°
```

## 🐛 Troubleshooting

### Vấn Đề 1: "Not enough oscillations detected"

**Nguyên nhân:** Motor không dao động đủ

**Giải pháp:**
```cpp
// Tăng test amplitude
float test_amplitude = 15.0f * DEG_TO_RAD;  // Từ 10° → 15°

// Hoặc giảm friction (kiểm tra bearing)
```

### Vấn Đề 2: PID values quá cao/thấp

**Kp > 20:**
```cpp
// Giảm test_amplitude
float test_amplitude = 5.0f * DEG_TO_RAD;
```

**Kp < 2:**
```cpp
// Tăng test_amplitude
float test_amplitude = 15.0f * DEG_TO_RAD;
```

### Vấn Đề 3: Motor rung lắc sau khi apply PID

**Giải pháp:**
```cpp
// Giảm Kp 30%
Kp = 7.5259f * 0.7f;  // = 5.268f

// Tăng Kd 50%
Kd = 0.7978f * 1.5f;  // = 1.197f
```

## 📊 So Sánh Methods

| Method | Ưu điểm | Nhược điểm |
|--------|---------|------------|
| **Auto-tuning (Hardware)** | Chính xác, test thật | Cần upload code riêng |
| **MATLAB Simulation** | Nhanh, không cần hardware | Model không chính xác |
| **Manual Tuning** | Flexible | Mất thời gian, cần kinh nghiệm |
| **Conservative Values** | An toàn, dễ bắt đầu | Chưa tối ưu |

## ✅ Workflow Khuyến Nghị

1. **Start:** Dùng conservative values (Kp=8, Kd=0.3)
2. **Test:** Xem gimbal hoạt động như thế nào
3. **Auto-tune:** Chạy hardware auto-tuning
4. **Fine-tune:** Điều chỉnh nhỏ nếu cần
5. **Done:** Apply vào production code

## 🎯 Expected Results

**Good Auto-Tuning:**
- Kp: 5-15
- Ki: 0.05-0.2
- Kd: 0.3-1.0

**Nếu ngoài range này:**
- Check test_amplitude
- Check mechanical setup
- Check motor connections

## 💡 Tips

1. **Tune từng motor riêng** - Tilt và Pan khác nhau
2. **Test nhiều lần** - Chạy 2-3 lần, lấy average
3. **Check mechanical** - Bearing phải smooth
4. **Start conservative** - Nếu auto-tune fail, dùng Kp=8, Kd=0.3

## 📚 Tham Khảo

- [Ziegler-Nichols Method](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
- [PID Relay Auto-Tuning](https://www.sciencedirect.com/topics/engineering/relay-feedback)

Good luck! 🎛️
