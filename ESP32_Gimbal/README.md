# ESP32 Gimbal - 3-Axis Tracking System
----------------------------------------------------------------------------------------------------
Hệ thống gimbal 3 trục sử dụng ESP32 để tracking đối tượng thông qua Android App với Bluetooth BLE.

## Tổng Quan

### Chức năng chính
- **MANUAL Mode**: Điều khiển bằng joystick qua Bluetooth
- **STABLE Mode**: Tự cân bằng dựa trên IMU (BNO055/MPU6050)
- **TRACKING Mode**: Visual servoing - theo dõi đối tượng từ Android App

### Hardware
| Thành phần | Mô tả |
|------------|-------|
| MCU | ESP32 Dev Module |
| Motor 1 (Yaw) | GM5602 - 7 pole pairs |
| Motor 2 (Pitch) | GM3502 - 11 pole pairs |
| Motor 3 (Roll) | GM3502 - 11 pole pairs |
| IMU | Adafruit BNO055 / MPU6050 |
| Nguồn | 12V DC |

### Software Stack
- **Framework**: Arduino + PlatformIO
- **Motor Control**: SimpleFOC Library
- **Communication**: Bluetooth Low Energy (BLE)
- **Control Algorithm**: PID Controller

## Sơ Đồ Kết Nối

### Motor Pin Assignments

```
Motor 1 (Yaw - GM5602):     Motor 2 (Pitch - GM3502):    Motor 3 (Roll - GM3502):
  UH: GPIO 2                  UH: GPIO 12                  UH: GPIO 25
  VH: GPIO 4                  VH: GPIO 14                  VH: GPIO 33
  WH: GPIO 5                  WH: GPIO 27                  WH: GPIO 32
  EN: GPIO 15                 EN: GPIO 13                  EN: GPIO 26
```

### Button & Sensors

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32 Dev Board                      │
│                                                             │
│  GPIO16 (Capture Button) ●────[Button]─── GND               │
│  GPIO21/22 (I2C) ●─────────── IMU Sensor                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Thông Số PID

### Giá trị hiện tại (TRACKING Mode)

```cpp
// YAW (Motor1 - Pan Left/Right)
PIDParams pidParamYaw = { 
    Kp: 4.0,    // Proportional gain
    Ki: 0.0,    // Integral gain  
    Kd: 0.05    // Derivative gain
};

// PITCH (Motor2 - Tilt Up/Down)
PIDParams pidParamPitch = { 
    Kp: 10.0,   // Proportional gain
    Ki: 0.0,    // Integral gain
    Kd: 0.01    // Derivative gain
};

// ROLL (Motor3 - không dùng trong tracking)
PIDParams pidParamRoll = { 
    Kp: 15.0,
    Ki: 0.0,
    Kd: 0.01
};
```

### Hướng dẫn Tune PID

| Tình huống | Giải pháp |
|------------|-----------|
| Tracking **chậm** | Tăng Kp (4.0 → 6.0) |
| Tracking **rung/dao động** | Tăng Kd hoặc giảm Kp |
| Có sai số tĩnh | Bật Ki (0.0 → 0.1) |

> Xem thêm: `docs/esp32_conservative_pid_guide.md` để có hướng dẫn chi tiết

## Bluetooth Protocol

### Service & Characteristics

| Tham số | UUID |
|---------|------|
| Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |
| TX (ESP32 → App) | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` |
| RX (App → ESP32) | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` |

### Data Format (App → ESP32)

```
{{[x]:offset_x;[y]:offset_y;[pan]:target_pan;[tilt]:target_tilt}}
```

| Field | Mô tả | Đơn vị |
|-------|-------|--------|
| `x` | Offset X từ center | pixels |
| `y` | Offset Y từ center | pixels |
| `pan` | Target pan angle | độ |
| `tilt` | Target tilt angle | độ |

### Commands

| Command | Mô tả |
|---------|-------|
| `CAPTURE` | Chụp ảnh từ xa |
| `M:0` | MANUAL mode |
| `M:1` | STABLE mode |
| `M:2` | TRACKING mode |


### Sử dụng VS Code
- **Build**: `Ctrl+Alt+B`
- **Upload**: `Ctrl+Alt+U`
- **Serial Monitor**: `Ctrl+Alt+S`

## Troubleshooting

### ESP32 không upload được
1. Giữ nút **BOOT** trên ESP32
2. Nhấn nút **EN** (reset)
3. Thả **EN**, giữ **BOOT** thêm 2 giây
4. Thả **BOOT** → Upload

### Bluetooth không kết nối
- Kiểm tra Serial Monitor có `BLE Service started`
- Reset ESP32 (nhấn nút EN)
- Xóa cache Bluetooth trên điện thoại

### Gimbal rung/oscillation
- Giảm `Kp` hoặc tăng `Kd`
- Kiểm tra power supply ổn định 12V
- Kiểm tra kết nối motor

### Tracking không chính xác
- Calibrate IMU sensor
- Kiểm tra polo pairs của motor
- Tune lại PID parameters

---

## Cấu Trúc Project

```
ESP32_Gimbal/
├── src/
│   ├── main.cpp              # Code chính (gimbal control)
│   └── Bluetooth.cpp         # BLE implementation
├── include/
│   ├── Bluetooth.h           # BLE class header
│   ├── BleProtocol.h         # Protocol utilities
│   └── PID_Controller.h      # PID controller class
├── lib/
│   └── SimpleFOC/            # Motor control library
├── docs/
│   ├── esp32_conservative_pid_guide.md
│   ├── esp32_hardware_tuning_guide.md
│   ├── esp32_gimbal_tuning_guide.md
│   ├── esp32_gimbal_pid_tuning.m      # MATLAB simulation
│   └── esp32_gimbal_auto_optimization.m
├── matlab_simulation/
│   └── visual_servoing_simulation.m
├── platformio.ini            # PlatformIO configuration
└── README.md                 # File này
```

---

## Control Modes Chi Tiết

### Mode 0: MANUAL
- Điều khiển joystick từ app
- Tốc độ quay tỉ lệ với input
- Không dùng IMU

### Mode 1: STABLE
- Tự cân bằng dựa trên IMU
- Giữ góc horizon cố định
- Dùng để chống rung

### Mode 2: TRACKING
- Visual servoing từ Android app
- App gửi target angle qua BLE
- Gimbal theo dõi đối tượng realtime
- PID controller điều khiển smooth movement

---

## Thông Số Kỹ Thuật

| Parameter | Value |
|-----------|-------|
| Supply Voltage | 12V DC |
| Motor Voltage Limit | 7.0V |
| Tracking Update Rate | 50 Hz (20ms) |
| BLE Baud Rate | 115200 |
| Velocity Limit (Yaw) | 1000 deg/s |
| Velocity Limit (Pitch) | 1200 deg/s |
| Deadzone | 2.0° |

---

## Changelog

### v1.1 (2025-12-26)
- Bluetooth BLE integration
- Remote capture button (GPIO16)
- Visual servoing tracking
- 3 control modes (MANUAL/STABLE/TRACKING)

### v1.0 (Initial)
- SimpleFOC motor control
- IMU integration (BNO055/MPU6050)
- Basic PID control

---

## License

MIT License - Free to use and modify

---

## Author

**TEAM DATN** - Gimbal Tracking Project

---

## Tài Liệu Tham Khảo

- [SimpleFOC Documentation](https://docs.simplefoc.com/)
- [ESP32 BLE Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/)
- [PlatformIO ESP32](https://docs.platformio.org/en/latest/platforms/espressif32.html)
