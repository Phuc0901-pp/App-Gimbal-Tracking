#include <Arduino.h>
#include "Bluetooth.h" 
#include <SimpleFOC.h>
#include <Wire.h>

// ======================================================================================
// 1. CẤU HÌNH PIN VÀ ĐỘNG CƠ
// ======================================================================================
constexpr uint8_t PIN_UH_1 = 0, PIN_VH_1 = 2,  PIN_WH_1 = 15,  PIN_EN_1 = 4;
constexpr uint8_t PIN_UH_2 = 12, PIN_VH_2 = 14, PIN_WH_2 = 27, PIN_EN_2 = 13;
constexpr uint8_t PIN_UH_3 = 25, PIN_VH_3 = 33, PIN_WH_3 = 32, PIN_EN_3 = 26;

// BUTTON PINS
constexpr uint8_t PIN_BUTTON_CAPTURE = 16;
constexpr uint8_t PIN_BUTTON_MODE = 19;

// Số cặp cực (Pole Pairs)
constexpr uint8_t POLE_PAIRS = 11;
constexpr uint8_t POLE_PAIRS_1 = 7;
constexpr float SUPPLY_V = 12.0;

BLDCMotor       motor1 (POLE_PAIRS_1); // PAN
BLDCDriver3PWM  driver1(PIN_UH_1, PIN_VH_1, PIN_WH_1, PIN_EN_1);

BLDCMotor       motor2 (POLE_PAIRS);   // TILT
BLDCDriver3PWM  driver2(PIN_UH_2, PIN_VH_2, PIN_WH_2, PIN_EN_2);

BLDCMotor       motor3 (POLE_PAIRS);   // ROLL
BLDCDriver3PWM  driver3(PIN_UH_3, PIN_VH_3, PIN_WH_3, PIN_EN_3);

// ======================================================================================
// 2. CÁC STRUCT VÀ HÀM HỖ TRỢ
// ======================================================================================

const float Ts_min = 1e-3;   // 1 ms
const float Ts_max = 0.005;  // 5 ms

struct PIDState {
  float integral = 0.0f;
  float prevErr = 0.0f;
  float derivative = 0.0f;
  float shaft_angle = 0.0f;  // deg (Góc trục động cơ)
  unsigned long prev_time = 0.0f;
};

struct PIDParams {
  float Kp;
  float Ki;
  float Kd;
};

// Khai báo các biến PID toàn cục
PIDState pidPitch;
PIDState pidYaw;
PIDState pidRoll;

// Tinh chỉnh PID
PIDParams pidParamPitch = { 5.5f, 0.0f, 0.2f }; // Kp, Ki, Kd
PIDParams pidParamYaw   = { 4.5f, 0.0f, 0.15f };
PIDParams pidParamRoll  = { 3.0f, 0.0f, 0.1f };

// Biến quản lý trạng thái Mode
bool isTrackingMode = false; // False = Home Mode, True = Tracking Mode
unsigned long lastDebounceTime = 0; // Giữ lại để code khác không lỗi nếu có dùng
unsigned long homeModeStartTime = 0; // Timer cho quy trình khởi động từng motor

// Hàm tính chênh lệch góc tối ưu (-180 đến 180)
float angleDiff(float target, float current) {
  float diff = fmod(target - current + 180.0f, 360.0f) - 180.0f;
  return diff < -180.0f ? diff + 360.0f : diff;
}

// HÀM QUAN TRỌNG: Tính toán góc điện dựa trên PID
float computeElectricalAngle(const char* name, float target_angle, float set_point,  
                             float velocity_limit_deg, int pole_pairs,
                             PIDState& state, const PIDParams& params) {

  unsigned long now_us = micros();
  // Khởi tạo prev_time nếu là lần đầu chạy
  if (state.prev_time == 0) state.prev_time = now_us;
  
  float Ts = (now_us - state.prev_time) * 1e-6f;
  state.prev_time = now_us;
  
  // Giới hạn Ts để tránh lỗi khi loop chạy quá chậm hoặc quá nhanh
  if(Ts < Ts_min) Ts = Ts_min;
  if(Ts > Ts_max) Ts = Ts_max;

  // Tính sai số
  float error = angleDiff(target_angle, set_point);

  // Tích phân
  state.integral += error * Ts;
  state.integral = constrain(state.integral, -10.0f, 10.0f); // Anti-windup

  // Đạo hàm
  float derivative_raw = (error - state.prevErr) / Ts;
  state.prevErr = error;
  
  // Lọc thông thấp cho D (Low Pass Filter)
  float tau = 0.05f; 
  float alpha = tau / (tau + Ts);
  state.derivative = alpha * state.derivative + (1.0f - alpha) * derivative_raw;

  // Tính vận tốc mong muốn (PID Output)
  float velocity = params.Kp * error + params.Ki * state.integral + params.Kd * state.derivative;

  // Giới hạn vận tốc
  velocity = constrain(velocity, -velocity_limit_deg, velocity_limit_deg);

  // Cập nhật góc trục (Tích phân vận tốc ra vị trí)
  state.shaft_angle += velocity * Ts;  

  // Đổi sang góc điện (Electrical Angle) để điều khiển cuộn dây
  float shaft_angle_rad = state.shaft_angle * DEG_TO_RAD;
  float electrical_angle = fmod(shaft_angle_rad * (float)pole_pairs, _2PI);
  
  return electrical_angle;
}

// ======================================================================================
// 3. XỬ LÝ BLUETOOTH
// ======================================================================================

// Biến toàn cục lưu lỗi nhận từ App
float app_err_yaw = 0.0f;
float app_err_pitch = 0.0f;

String getValue(String data, String key) {
  String keyPattern = "[" + key + "]:";
  int start = data.indexOf(keyPattern);
  if (start == -1) return "";
  start += keyPattern.length();
  int end = data.indexOf(";", start);
  if (end == -1) end = data.indexOf("}}", start);
  if (end == -1) return "";
  return data.substring(start, end);
}

void onDataReceived(String data) {
  data.trim();
  if (data.indexOf("LOST") != -1) {
     // Khi mất dấu, set lỗi về 0
     app_err_yaw = 0.0f;
     app_err_pitch = 0.0f;
     return;
  }
  
  // Parse dữ liệu {{[ax]:...;[ay]:...}}
  if (data.startsWith("{{") && data.endsWith("}}")) {
      String s_ax = getValue(data, "ax"); // Pan Error
      String s_ay = getValue(data, "ay"); // Tilt Error
      if(s_ax != "" && s_ay != "") {
          app_err_yaw = s_ax.toFloat();
          app_err_pitch = s_ay.toFloat();
      }
  }
}

// ======================================================================================
// 4. SETUP
// ======================================================================================
void setup() {
  Serial.begin(115200);
  Bluetooth::getInstance()->init("ESP32_Tracker", onDataReceived);
  Wire.begin(22, 21);
  
  // Setup Buttons
  pinMode(PIN_BUTTON_CAPTURE, INPUT_PULLUP);
  pinMode(PIN_BUTTON_MODE, INPUT_PULLUP);

  // --- CẤU HÌNH MOTOR ---
  
  // PAN
  driver1.voltage_power_supply = SUPPLY_V;
  driver1.pwm_frequency = 25000;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.controller = MotionControlType::angle_openloop; 
  motor1.voltage_limit = 9.0f; 
  motor1.init();

  // TILT
  driver2.voltage_power_supply = SUPPLY_V;
  driver2.pwm_frequency = 20000;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.controller = MotionControlType::angle_openloop;
  motor2.voltage_limit = 9.0f; 
  motor2.init();

  // ROLL
  driver3.voltage_power_supply = SUPPLY_V;
  driver3.pwm_frequency = 20000;
  driver3.init();
  motor3.linkDriver(&driver3);
  motor3.controller = MotionControlType::angle_openloop;
  motor3.voltage_limit = 5.0f; // Tăng lên 5V 
  motor3.init();

  // --- THIẾT LẬP ĐIỂM KHỞI ĐỘNG (0, -90, 0) ---
  pidYaw.shaft_angle   = 0.0f;   
  pidPitch.shaft_angle = 0.0f;  
  pidRoll.shaft_angle  = 0.0f;  // Bắt đầu từ 0 để chạy lên 90
  
  unsigned long now = micros();
  pidYaw.prev_time = now;
  pidPitch.prev_time = now;
  pidRoll.prev_time = now;
  
  homeModeStartTime = millis(); // Bắt đầu đếm giờ cho quy trình khởi động

  Serial.println("System Ready. Mode 1: HOME.");
  delay(1000);
}

// ======================================================================================
// 5. LOOP
// ======================================================================================
void loop() {
  motor1.loopFOC();
  motor2.loopFOC();
  motor3.loopFOC();

  // --- XỬ LÝ NÚT NHẤN (CHỐNG RUNG & NHIỄU) ---
  
  // 1. Xử lý nút CAPTURE (GPIO16)
  static int lastBtnCaptureState = HIGH;      // Trạng thái vật lý trước đó
  static int stableBtnCaptureState = HIGH;    // Trạng thái ổn định (đã debounce)
  static unsigned long lastDebounceCapture = 0;
  
  int readingCapture = digitalRead(PIN_BUTTON_CAPTURE);
  if (readingCapture != lastBtnCaptureState) {
      lastDebounceCapture = millis(); // Reset timer khi có tín hiệu thay đổi (do nhiễu hoặc nhấn thật)
  }
  
  if ((millis() - lastDebounceCapture) > 50) { // Đợi 50ms ổn định
      if (readingCapture != stableBtnCaptureState) {
          stableBtnCaptureState = readingCapture;
          // Chỉ kích hoạt sự kiện khi chuyển từ HIGH -> LOW (Nhấn xuống)
          if (stableBtnCaptureState == LOW) {
              Serial.println(">>> BUTTON CAPTURE PRESSED (Verified)");
              Bluetooth::getInstance()->send("CAPTURE");
          }
      }
  }
  lastBtnCaptureState = readingCapture;


  // 2. Xử lý nút MODE (GPIO19)
  static int lastBtnModeState = HIGH;
  static int stableBtnModeState = HIGH;
  static unsigned long lastDebounceMode = 0;
  
  int readingMode = digitalRead(PIN_BUTTON_MODE);
  if (readingMode != lastBtnModeState) {
      lastDebounceMode = millis();
  }
  
  if ((millis() - lastDebounceMode) > 50) {
      if (readingMode != stableBtnModeState) {
          stableBtnModeState = readingMode;
          // Chỉ kích hoạt khi nhấn xuống
          if (stableBtnModeState == LOW) {
              isTrackingMode = !isTrackingMode;
              
              // Nếu chuyển về HOME mode, reset timer để chạy lại quy trình khởi động từng motor
              if (!isTrackingMode) {
                  homeModeStartTime = millis();
              }
              
              Serial.print(">>> BUTTON MODE PRESSED. New Mode: ");
              Serial.println(isTrackingMode ? "TRACKING" : "HOME");
          }
      }
  }
  lastBtnModeState = readingMode;

  // --- LOGIC ĐIỀU KHIỂN ---
  
  float elec_angleYaw, elec_anglePitch, elec_angleRoll;
  float volYaw = 0, volPitch = 0, volRoll = 0; // Biến lưu điện áp sẽ cấp
  float velLimitYaw = 1000.0f, velLimitPitch = 1000.0f, velLimitRoll = 1000.0f;

  if (isTrackingMode) {
      // --- MODE 2: TRACKING (Kích hoạt toàn bộ) ---
      
      // 1. Dữ liệu từ App
      float effective_yaw_err = (abs(app_err_yaw) < 0.5) ? 0 : app_err_yaw;
      float effective_pitch_err = (abs(app_err_pitch) < 0.5) ? 0 : app_err_pitch;

      // Tính toán góc
      elec_angleYaw = computeElectricalAngle("Yaw", 0.0f, effective_yaw_err, 1000.0f, POLE_PAIRS_1, pidYaw, pidParamYaw);
      elec_anglePitch = computeElectricalAngle("Pitch", 0.0f, effective_pitch_err, 1000.0f, POLE_PAIRS, pidPitch, pidParamPitch);
      elec_angleRoll = computeElectricalAngle("Roll", 90.0f, pidRoll.shaft_angle, 1000.0f, POLE_PAIRS, pidRoll, pidParamRoll);
      
      // Cấp điện áp (Full)
      volYaw = 7.0f;
      volPitch = 6.0f;
      volRoll = 4.0f;
      
  } else {
      // --- MODE 1: HOME (Khởi động tuần tự) ---
      
      unsigned long elapsed = millis() - homeModeStartTime;
      
      // QUY TRÌNH KHỞI ĐỘNG TUẦN TỰ:
      // Điều khiển cả Voltage và Velocity Limit để tạo hiệu ứng "quét" từ 0 đến đích
      
      if (elapsed < 2000) {
          // Giai đoạn 1: Chỉ Yaw
          volYaw = 7.0f;   velLimitYaw = 500.0f;
          volPitch = 0.0f; velLimitPitch = 0.0f; // Giữ nguyên vị trí PID
          volRoll = 0.0f;  velLimitRoll = 0.0f;
      } else if (elapsed < 4000) {
          // Giai đoạn 2: Yaw + Pitch
          volYaw = 7.0f;   velLimitYaw = 500.0f;
          volPitch = 6.0f; velLimitPitch = 500.0f; // Bắt đầu chạy Pitch
          volRoll = 0.0f;  velLimitRoll = 0.0f;
      } else {
          // Giai đoạn 3: Full
          volYaw = 7.0f;   velLimitYaw = 500.0f;
          volPitch = 6.0f; velLimitPitch = 500.0f;
          volRoll = 4.0f;  velLimitRoll = 500.0f; // Bắt đầu chạy Roll
      }

      // Tính toán góc (Với limit động)
      elec_angleYaw = computeElectricalAngle("Yaw", 0.0f, pidYaw.shaft_angle, velLimitYaw, POLE_PAIRS_1, pidYaw, pidParamYaw);
      elec_anglePitch = computeElectricalAngle("Pitch", 270.0f, pidPitch.shaft_angle, velLimitPitch, POLE_PAIRS, pidPitch, pidParamPitch);
      elec_angleRoll = computeElectricalAngle("Roll", 90.0f, pidRoll.shaft_angle, velLimitRoll, POLE_PAIRS, pidRoll, pidParamRoll);
  }

  // Xuất ra động cơ
  motor1.setPhaseVoltage(volYaw, 0, elec_angleYaw);
  motor2.setPhaseVoltage(volPitch, 0, elec_anglePitch);
  motor3.setPhaseVoltage(volRoll, 0, elec_angleRoll);

  delayMicroseconds(800); 
}
