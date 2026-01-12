# Thông Số PID - TRACKING_MODE

## 📊 PID Parameters Hiện Tại

### YAW (Motor1 - Pan Left/Right)
```cpp
PIDParams pidParamYaw = { 
    Kp: 4.0,    // Proportional gain
    Ki: 0.0,    // Integral gain  
    Kd: 0.05    // Derivative gain
};
```

**Motor:** GM5602 (7 pole pairs)  
**Góc điều khiển:** -45° đến +45° (Pan)  
**Vận tốc max:** 1000 deg/s

---

### PITCH (Motor2 - Tilt Up/Down)
```cpp
PIDParams pidParamPitch = { 
    Kp: 10.0,   // Proportional gain
    Ki: 0.0,    // Integral gain
    Kd: 0.01    // Derivative gain
};
```

**Motor:** GM3502 (11 pole pairs)  
**Góc điều khiển:** -30° đến +30° (Tilt)  
**Vận tốc max:** 1200 deg/s

---

### ROLL (Motor3 - Không dùng trong tracking)
```cpp
PIDParams pidParamRoll = { 
    Kp: 15.0,
    Ki: 0.0,
    Kd: 0.01
};
```
**Note:** Motor3 (Roll) không được sử dụng trong TRACKING_MODE

---

## 🎯 Ý Nghĩa Từng Thông Số

### Kp (Proportional Gain) - Độ nhạy
- **YAW (4.0):** Nhạy vừa phải cho chuyển động ngang
- **PITCH (10.0):** Nhạy cao hơn cho chuyển động dọc (nhanh hơn)

**Công thức:** `P_output = Kp × error`

**Ví dụ:**
- Error = 20° → YAW output = 4.0 × 20 = 80 deg/s
- Error = 20° → PITCH output = 10.0 × 20 = 200 deg/s

### Ki (Integral Gain) - Loại bỏ sai số tĩnh
- **Cả 3 = 0.0:** Tắt hoàn toàn

**Tại sao tắt Ki?**
- Gimbal không cần giữ chính xác tuyệt đối
- Tránh integral windup (tích lũy lỗi)
- Giảm overshoot

### Kd (Derivative Gain) - Chống rung
- **YAW (0.05):** Khá mạnh để giảm dao động ngang
- **PITCH (0.01):** Nhẹ hơn

**Công thức:** `D_output = Kd × (rate of change of error)`

**Tác dụng:** Giảm overshoot, làm mượt chuyển động

---

## 🔧 Cách Tune PID

### File: `main.cpp` lines 124-126

```cpp
// Tìm dòng này và chỉnh:
PIDParams pidParamPitch = { 10.0f, 0.0f, 0.01f }; 
PIDParams pidParamYaw   = { 4.0f, 0.0f, 0.05f };
```

### Nếu Tracking CHẬM:
```cpp
// Tăng Kp
PIDParams pidParamYaw = { 6.0f, 0.0f, 0.05f };  // 4.0→6.0
```

### Nếu Tracking RUNG/DAO ĐỘNG:
```cpp
// Tăng Kd
PIDParams pidParamYaw = { 4.0f, 0.0f, 0.1f };   // 0.05→0.1
// HOẶC giảm Kp
PIDParams pidParamYaw = { 3.0f, 0.0f, 0.05f };  // 4.0→3.0
```

### Nếu CÓ SAI SỐ TĨNH (không về đúng 0°):
```cpp
// Bật Ki nhẹ
PIDParams pidParamYaw = { 4.0f, 0.1f, 0.05f };  // Ki=0→0.1
```

---

## 📈 So Sánh Với Chế Độ Khác

| Mode | YAW Kp | PITCH Kp | Note |
|------|--------|----------|------|
| **TRACKING** | 4.0 | 10.0 | Theo app Android |
| **STABLE** | 4.0 | 10.0 | Tự cân bằng IMU |
| **MANUAL** | N/A | N/A | Velocity control trực tiếp |

---

## ⚙️ Thông Số Khác

### Voltage Limits (TRACKING_MODE)
```cpp
motor1.setPhaseVoltage(7.0f * 0.8f, 0, elec_angleYaw);   // 5.6V
motor2.setPhaseVoltage(7.0f * 0.8f, 0, elec_anglePitch); // 5.6V
```

**Tại sao × 0.8?**
- Giảm 20% voltage để tracking mượt hơn
- Tránh jerky motion
- Tiết kiệm năng lượng

### Velocity Limits
```cpp
computeElectricalAngle("Yaw",  ..., 1000.0f, ...);  // Max 1000 deg/s
computeElectricalAngle("Pitch", ..., 1200.0f, ...); // Max 1200 deg/s
```

### Low-pass Filter (Derivative)
```cpp
float tau = 0.05f;  // Time constant
float alpha = tau / (tau + Ts);
state.derivative = alpha * state.derivative + (1.0f - alpha) * derivative_raw;
```

**Tác dụng:** Lọc nhiễu khâu D, tránh khuếch đại noise

---

## 🎓 Khuyến Nghị

### Thông Số Hiện Tại:
✅ **ĐÃ TỐI ƯU** cho tracking người ở khoảng cách 2-5m

### Nếu Muốn Tracking Nhanh Hơn:
```cpp
PIDParams pidParamYaw   = { 6.0f, 0.0f, 0.08f };   // ↑ Kp, ↑ Kd
PIDParams pidParamPitch = { 15.0f, 0.0f, 0.02f };  // ↑ Kp, ↑ Kd
```

### Nếu Muốn Tracking Mượt Hơn:
```cpp
PIDParams pidParamYaw   = { 3.0f, 0.0f, 0.03f };   // ↓ Kp
PIDParams pidParamPitch = { 8.0f, 0.0f, 0.01f };   // ↓ Kp
```

### Nếu Target Di Chuyển Nhanh:
```cpp
// Tăng velocity limit
computeElectricalAngle("Yaw",  ..., 1500.0f, ...);  // 1000→1500
computeElectricalAngle("Pitch", ..., 1800.0f, ...); // 1200→1800
```

---

## 🔍 Debug PID

### Thêm Log để Xem PID Output:

```cpp
// Trong computeElectricalAngle(), sau dòng "float velocity = ..."
Serial.printf("[PID %s] Error:%.1f P:%.1f I:%.1f D:%.1f → Vel:%.1f\n",
    name, error, 
    params.Kp * error, 
    params.Ki * state.integral,
    params.Kd * state.derivative,
    velocity
);
```

**Output mẫu:**
```
[PID Yaw] Error:25.0 P:100.0 I:0.0 D:5.0 → Vel:105.0
[PID Pitch] Error:10.0 P:100.0 I:0.0 D:2.0 → Vel:102.0
```

---

**Thông số hiện tại là balanced cho tracking ổn định!** ⚖️
