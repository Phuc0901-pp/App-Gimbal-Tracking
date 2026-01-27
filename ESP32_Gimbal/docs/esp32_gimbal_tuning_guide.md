# ESP32 Gimbal PID Tuning Guide

## 🎯 Mục Đích

Tìm PID gains tối ưu cho gimbal hardware với:
- **BLDC motors** (SimpleFOC OpenLoop control)
- **Physical constraints** (Tilt: 45-135°, Pan: ±180°)
- **Update rate**: 100 Hz (10ms loop time)

## 🚀 Cách Sử Dụng

### Bước 1: Chạy Simulation

```matlab
% Mở file trong MATLAB:
esp32_gimbal_pid_tuning.m

% Chạy: F5
```

### Bước 2: Xem Kết Quả

Script sẽ hiển thị:
- **9 plots**: Position, Error, Control signal cho cả Tilt và Pan
- **Performance metrics**: MAE, RMSE, Overshoot
- **Recommendations**: Gợi ý điều chỉnh PID

### Bước 3: Điều Chỉnh PID

Sửa values trong code (line ~40-50):

```matlab
% Tilt Motor
Kp_tilt_init = 5.0;   % Điều chỉnh ở đây
Ki_tilt_init = 0.0;
Kd_tilt_init = 0.15;

% Pan Motor  
Kp_pan_init = 3.5;    % Điều chỉnh ở đây
Ki_pan_init = 0.0;
Kd_pan_init = 0.15;
```

## 📊 Hiểu Kết Quả

### Performance Targets (Good)

**Tilt Motor:**
- MAE < 2° ✅
- Overshoot < 5° ✅
- No oscillation ✅

**Pan Motor:**
- MAE < 3° ✅
- Smooth tracking ✅

### Reading Plots

**Plot 1 & 4 (Position):**
- Red line phải follow blue line sát
- Không vọt lố (overshoot) quá 5°
- Không rung lắc (oscillation)

**Plot 2 & 5 (Error):**
- Error phải giảm nhanh về gần 0
- Không dao động liên tục

**Plot 3 & 6 (Control Signal):**
- Smooth, không nhảy cóc
- Không quá lớn (saturation)

## 🔧 Tuning Guide

### Vấn Đề 1: Overshoot (Vọt Lố)

**Triệu chứng:** Gimbal vượt quá target rồi mới quay lại

**Giải pháp:**
```cpp
// Tăng Kd (damping)
Kd = 0.25;  // Từ 0.15

// Hoặc giảm Kp
Kp = 4.0;   // Từ 5.0
```

### Vấn Đề 2: Oscillation (Rung Lắc)

**Triệu chứng:** Gimbal dao động qua lại quanh target

**Giải pháp:**
```cpp
// Giảm Kp mạnh
Kp = 3.0;   // Từ 5.0

// Tăng Kd
Kd = 0.3;   // Từ 0.15
```

### Vấn Đề 3: Sluggish (Phản Hồi Chậm)

**Triệu chứng:** Gimbal di chuyển quá chậm, không bắt kịp target

**Giải pháp:**
```cpp
// Tăng Kp
Kp = 7.0;   // Từ 5.0

// Kiểm tra motor velocity_limit
motor.velocity_limit = 2.0f;  // Tăng từ 1.5
```

### Vấn Đề 4: Steady-State Error

**Triệu chứng:** Gimbal dừng lại nhưng không đúng vị trí target

**Giải pháp:**
```cpp
// Thêm Ki (integral term)
Ki = 0.01;  // Từ 0.0

// Lưu ý: Ki quá lớn → overshoot!
```

## 🎛️ Tuning Workflow

### Step 1: Start with Kp Only

```cpp
Kp = 3.0;
Ki = 0.0;
Kd = 0.0;
```

Tăng Kp dần cho đến khi thấy oscillation → Giảm 20%

### Step 2: Add Kd

```cpp
Kp = [value from step 1];
Ki = 0.0;
Kd = 0.1;  // Bắt đầu nhỏ
```

Tăng Kd cho đến khi overshoot < 5°

### Step 3: Add Ki (If Needed)

```cpp
Ki = 0.01;  // Rất nhỏ
```

Chỉ thêm nếu có steady-state error

## 🔬 Advanced: Auto-Optimization

Uncomment code ở cuối file (line ~200):

```matlab
% Bỏ comment các dòng optimization
% Chạy lại script
% Đợi 5-10 phút
```

MATLAB sẽ tự động tìm PID tối ưu.

## 📱 Apply Vào ESP32

Sau khi tìm được PID tốt, update vào code:

```cpp
// Tilt Motor (Motor 2)
PID_Controller pidTilt(
    5.0f,   // Kp ← Update
    0.0f,   // Ki ← Update  
    0.15f,  // Kd ← Update
    90.0f * DEG_TO_RAD
);

// Pan Motor (Motor 1)
PID_Controller pidPan(
    3.5f,   // Kp ← Update
    0.0f,   // Ki ← Update
    0.15f,  // Kd ← Update
    0.0f
);
```

## ⚠️ Important Notes

### Motor Velocity Limit

```cpp
motor.velocity_limit = 1.5f;  // [rad/s]
```

- Quá thấp → PID không thể điều khiển nhanh
- Khuyến nghị: **2.0 - 3.0** cho gimbal tracking

### Integral Windup

Code của bạn đã có anti-windup:
```cpp
integral = constrain(integral, -5.0f, 5.0f);
```

Giữ nguyên hoặc giảm xuống `-2.0, 2.0` nếu Ki > 0.

### Loop Frequency

```cpp
// ESP32 loop phải chạy >= 100 Hz
void loop() {
    motor1.loopFOC();  // Nhanh nhất có thể
    motor2.loopFOC();
    motor3.loopFOC();
    
    // PID calculation
    float smooth_pan = pidPan.compute(target_pan);
    // ...
}
```

Không thêm `delay()` trong loop!

## 🐛 Troubleshooting

### Q: Gimbal rung lắc ngay cả với Kp thấp

**A:** Kiểm tra:
1. Motor wiring (đúng phase UVW?)
2. Pole pairs (đúng số cực?)
3. Voltage limit (quá cao?)

### Q: Gimbal không di chuyển

**A:** Kiểm tra:
1. `motor.voltage_limit` (đủ lớn?)
2. `motor.velocity_limit` (đủ lớn?)
3. Target có đến motor không? (Serial.print debug)

### Q: Gimbal nhảy cóc (jerky)

**A:** 
1. Tăng Kd để smooth hơn
2. Giảm Kp
3. Kiểm tra loop frequency (phải >= 100 Hz)

## 📚 Tham Khảo

- [SimpleFOC Documentation](https://docs.simplefoc.com/)
- [PID Tuning Guide](https://en.wikipedia.org/wiki/PID_controller#Loop_tuning)
- App's `pid_optimization_summary.md` - So sánh với software PID

## ✅ Checklist

- [ ] Chạy simulation với PID hiện tại
- [ ] MAE < 2° cho Tilt
- [ ] MAE < 3° cho Pan
- [ ] Không overshoot > 5°
- [ ] Không oscillation
- [ ] Update vào ESP32 code
- [ ] Test trên hardware thật
- [ ] Fine-tune dựa trên physical behavior

Good luck! 🎯
