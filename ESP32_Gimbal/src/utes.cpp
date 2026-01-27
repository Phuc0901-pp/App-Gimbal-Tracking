// Library includes
  #include <SimpleFOC.h>
  #include <Wire.h>
  #include <Adafruit_BNO055.h>
  #include <utility/imumaths.h>
  #include <Adafruit_MPU6050.h>
  #include "../include/Bluetooth.h"
  #include "../include/BleProtocol.h"
// Define pin numbers
  #define BTN1 16
  #define BTN2 19
  #define JOY_X 34
  #define JOY_Y 35
// Motor and driver definitions
  constexpr uint8_t PIN_UH_1 = 0, PIN_VH_1 = 2,  PIN_WH_1 = 15,  PIN_EN_1 = 4;
  constexpr uint8_t PIN_UH_2 = 12, PIN_VH_2 = 14, PIN_WH_2 = 27, PIN_EN_2 = 13;
  constexpr uint8_t PIN_UH_3 = 25, PIN_VH_3 = 33, PIN_WH_3 = 32, PIN_EN_3 = 26;
  constexpr uint8_t POLE_PAIRS = 11;
  constexpr uint8_t POLE_PAIRS_1 = 7;
  BLDCMotor       motor1 (POLE_PAIRS_1);
  BLDCDriver3PWM  driver1(PIN_UH_1, PIN_VH_1, PIN_WH_1, PIN_EN_1);
  BLDCMotor       motor2 (POLE_PAIRS);
  BLDCDriver3PWM  driver2(PIN_UH_2, PIN_VH_2, PIN_WH_2, PIN_EN_2);
  BLDCMotor       motor3 (POLE_PAIRS);
  BLDCDriver3PWM  driver3(PIN_UH_3, PIN_VH_3, PIN_WH_3, PIN_EN_3);
  const unsigned long SEND_INTERVAL_MS = 20; 
  constexpr float SUPPLY_V = 12.0;
  //=========================================================================================================================================================//
  String rxBuffer = "";
  int lastReceivedSeq = -1;
  float targetPanAngle = 0.0f;   // Góc Pan từ app
  float targetTiltAngle = 0.0f;  // Góc Tilt từ app
  float saved_roll_target = 0.0f; // Biến lưu góc Roll mục tiêu (so với mặt đất)
  bool trackingDataReceived = false;
  unsigned long lastTrackingDataTime = 0;
  const unsigned long TRACKING_TIMEOUT_MS = 500; // 0.5s
// ========================================================================================================================================================//
  const float Ts_min = 1e-3;   // 1 ms
  const float Ts_max = 0.005;  // 5 ms, dựa trên quan sát thực tế
  uint8_t last_mode = 255;
  unsigned long mode_enter_time = 0;
  imu::Quaternion q_offset;
  enum Mode : uint8_t {CAPTURE_ZERO, BALANCE};
  Mode Stable_Mode = CAPTURE_ZERO;
  Mode Tracking_Mode = CAPTURE_ZERO;
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
  Adafruit_MPU6050 mpu;
  unsigned long btn2_press_time = 0;
  bool roll_selected = false;   // false = yaw, true = roll
  bool btn1_last = LOW;
  bool btn2_pressed = false;
  const unsigned long RESET_HOLD_TIME = 3000; // 3 giây
  const unsigned long SHORT_PRESS_MIN = 50;   // chống dội
  uint8_t last_button_mode = 0;
  // ===============================================================================================================================================================//
  float yaw_vel_target = 0.0f;
  float yaw_vel_cmd    = 0.0f;
  float pitch_vel_target = 0.0f;
  float pitch_vel_cmd    = 0.0f;
  float roll_vel_target = 0.0f;
  float roll_vel_cmd    = 0.0f;
  const float VEL_RAMP = 2.0f;   // rad/s² (tune: 0.5 – 3)
// ==========================================================================================================================================================//
  // ===== TARGET ANGLES =====
  float targetYaw   = 0.0f;
  float targetPitch = 0.0f;
  float targetRoll  = 0.0f;
  // ===== ACCUMULATED POSITION (for smooth mode transition) =====
  float accumulated_yaw_pos = 0.0f;   // Tích lũy vị trí Yaw (rad)
  float accumulated_pitch_pos = 0.0f; // Tích lũy vị trí Pitch (rad)
  // ===== SOFT START VOLTAGE =====
  float v1_cmd = 0.0f, v1_target = 0.0f;
  float v2_cmd = 0.0f, v2_target = 0.0f;
  float v3_cmd = 0.0f, v3_target = 0.0f;
  const float VOLT_RAMP = 5.0f; 
  // V/s (tune: 2–10)
  #define MA_SIZE 2
  float pid_buffer[MA_SIZE] = {0};
  int pid_idx = 0;
  float pitch_error = 0.0f;
  float roll_error = 0.0f;
  float yaw_error = 0.0f;
  //========================================================================================================================================================//
  float filterPID(float input) {
    pid_buffer[pid_idx] = input;
    pid_idx = (pid_idx + 1) % MA_SIZE;
    float sum = 0;
    for (int i = 0; i < MA_SIZE; i++) sum += pid_buffer[i];
    return sum / MA_SIZE;
  }

  // ===== Đổi angleDiff sang ĐỘ =====
  /*
  float angleDiff(float target, float current) {
    float diff = fmod(target - current + 180.0f, 360.0f) - 180.0f;
    return diff < -180.0f ? diff + 360.0f : diff;
  }
  */
  float angleDiff(float target, float current) {
  float diff = fmod(target - current + M_PI, 2.0f * M_PI) - M_PI;
  return (diff < -M_PI) ? diff + 2.0f * M_PI : diff;
  }
  

  struct PIDState {
    float integral = 0.0f;
    float prevErr = 0.0f;
    float derivative = 0.0f;
    float shaft_angle = 0.0f;  // deg
    unsigned long prev_time = 0.0f;
  };

  struct PIDParams {
    float Kp;
    float Ki;
    float Kd;
  };
  float normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

  enum Button1_Mode : uint8_t {
    BT1_NONE_MODE = 0,
  };

  enum Button2_Mode : uint8_t {
    BT2_NONE_MODE = 0,
    MANUAL_MODE = 1,
    STABLE_MODE = 2,
    TRACKING_MODE = 3,
  };
  Button2_Mode Button2_State =  BT2_NONE_MODE;
  uint8_t Bt1_Mode_Flag = 0; // Khac trong kia
  uint8_t Bt2_Mode_Flag = 0;

  PIDState pidPitch;
  PIDState pidYaw;
  PIDState pidRoll;

//PID không tải
 /*
 PIDParams pidParamPitch = { 100.0f, 1.0f, 0.001f }; 
 PIDParams pidParamYaw   = { 100.0f, 0.0f, 0.001f };
 PIDParams pidParamRoll  = { 300.0f, 1.0f, 0.001f };
 */
/*
PIDParams pidParamPitch = { 4.0f, 0.1f, 0.01f }; 
PIDParams pidParamYaw   = { 3.0f, 0.0f, 0.01f };
PIDParams pidParamRoll  = { 2.0f, 0.2f, 0.001f };
*/
//PID có tải
 PIDParams pidParamPitch = { 80.0f, 1.0f, 0.001f }; 
 PIDParams pidParamYaw   = { 50.0f, 0.0f, 0.001f };
 PIDParams pidParamRoll  = { 70.0f, 1.0f, 0.001f };


  float computeElectricalAngle(const char* name, float target_angle, float set_point,  
                              float velocity_limit_deg, int pole_pairs,
                              PIDState& state, const PIDParams& params) {

    unsigned long now_us = micros();
    float Ts = (now_us - state.prev_time) * 1e-6f;
    state.prev_time = now_us;
    // Giới hạn Ts
    if(Ts < Ts_min) Ts = Ts_min;
    if(Ts > Ts_max) Ts = Ts_max;

    float error = angleDiff(target_angle, set_point);
    float abs_err = fabs(error);
    // ===== Integral =====
    state.integral += error * Ts;

  // ===== Derivative =====
  float derivative_raw = (error - state.prevErr) / Ts;
  state.prevErr = error;

  float tau = 0.05f;
  float alpha = tau / (tau + Ts);
  state.derivative = alpha * state.derivative
                   + (1.0f - alpha) * derivative_raw;

  // ===== PID giảm dần khi gần đích =====
  float slow_zone = 5.0f;    // deg
  float stop_zone = 0.1f;    // deg

  float scale = 1.0f;
  /*
  if (abs_err < slow_zone) {
    scale = abs_err / slow_zone;
    scale = constrain(scale, 0.0f, 1.0f);
  }
  */

  float Kp_eff = params.Kp * scale;
  float Ki_eff = params.Ki * scale * scale;
  float Kd_eff = params.Kd * scale;

  if (abs_err < stop_zone) {
    state.integral *= 0.9f;
  }

  float velocity = Kp_eff * error
                 + Ki_eff * state.integral
                 + Kd_eff * state.derivative;

  float vel_limit_eff = velocity_limit_deg * (0.2f + 0.8f * scale);
  velocity = constrain(velocity, -vel_limit_eff, vel_limit_eff);
  //velocity = filterPID(velocity);
  // ===== Update shaft angle =====
  state.shaft_angle += velocity * Ts;
  float shaft_angle_rad = state.shaft_angle;
  //float shaft_angle_rad = state.shaft_angle * DEG_TO_RAD;
  float electrical_angle = fmod(shaft_angle_rad * pole_pairs, _2PI);
  return electrical_angle;

  }


  imu::Quaternion normalizeQuat(const imu::Quaternion& q) {
    float norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
    if (norm < 1e-6) return imu::Quaternion(1,0,0,0);
    return imu::Quaternion(q.w()/norm, q.x()/norm, q.y()/norm, q.z()/norm);
  }

  enum ControlMode {
    MODE_ANGLE,
    MODE_VELOCITY
  };

  ControlMode controlMode = MODE_VELOCITY;   

  void initDrivers() {
    // MOTOR 1
    driver1.voltage_power_supply = SUPPLY_V;
    driver1.pwm_frequency = 25000;
    driver1.init();
    motor1.linkDriver(&driver1);

    // MOTOR 2
    driver2.voltage_power_supply = SUPPLY_V;
    driver2.pwm_frequency = 25000;
    driver2.init();
    motor2.linkDriver(&driver2);

    // MOTOR 3
    driver3.voltage_power_supply = SUPPLY_V;
    driver3.pwm_frequency = 25000;
    driver3.init();
    motor3.linkDriver(&driver3);
  }

  void setupAngleOpenLoop() {
    motor1.controller = MotionControlType::angle_openloop;
    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor1.voltage_limit = 7.0f;
    motor1.velocity_limit = 0.5f;
    motor1.init();

    motor2.controller = MotionControlType::angle_openloop;
    motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor2.voltage_limit = 9.0f;
    motor2.velocity_limit = 1.0f;
    motor2.init();

    motor3.controller = MotionControlType::angle_openloop;
    motor3.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor3.voltage_limit = 7.0f;
    motor3.velocity_limit = 0.5f;
    motor3.init();
    Serial.println("MODE: ANGLE_OPENLOOP");

  }

  void setupVelocityOpenLoop() {
    motor1.controller = MotionControlType::velocity_openloop;
    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor1.voltage_limit = 7.0f;
    motor1.init();

    motor2.controller = MotionControlType::velocity_openloop;
    motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor2.voltage_limit = 9.0f;
    motor2.init();

    motor3.controller = MotionControlType::velocity_openloop;
    motor3.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor3.voltage_limit = 7.0f;
    motor3.init();
    Serial.println("MODE: VELOCITY_OPENLOOP");
  }
/*

  void setMotorMode(ControlMode mode) {
    if (mode == MODE_ANGLE) {
      motor1.controller = MotionControlType::angle_openloop;
      motor2.controller = MotionControlType::angle_openloop;
      motor3.controller = MotionControlType::angle_openloop;

      Serial.println("MODE: ANGLE_OPENLOOP");
    }
    else if (mode == MODE_VELOCITY) {
      motor1.controller = MotionControlType::velocity_openloop;
      motor2.controller = MotionControlType::velocity_openloop;
      motor3.controller = MotionControlType::velocity_openloop;

      Serial.println("MODE: VELOCITY_OPENLOOP");
    }
  }

  void controlAngleMode(float yaw_deg, float pitch_deg, float roll_deg) {
    motor1.loopFOC();
    motor2.loopFOC();
    motor3.loopFOC();

    motor1.move(yaw_deg   * DEG_TO_RAD);
    motor2.move(pitch_deg * DEG_TO_RAD);
    motor3.move(roll_deg  * DEG_TO_RAD);
  }

  void controlVelocityMode(float yaw_rate, float pitch_rate, float roll_rate) {
    motor1.loopFOC();
    motor2.loopFOC();
    motor3.loopFOC();

    motor1.move(yaw_rate);    // rad/s
    motor2.move(pitch_rate);
    motor3.move(roll_rate);
  }
*/
  float rampVelocity(float cmd, float target, float Ts) {
    float diff = target - cmd;
    float step = VEL_RAMP * Ts;

    if (fabs(diff) <= step)
      return target;
    else
      return cmd + (diff > 0 ? step : -step);
  }

  float rampVoltage(float cmd, float target, float Ts) {
    float diff = target - cmd;
    float step = VOLT_RAMP * Ts;

    if (fabs(diff) <= step)
      return target;
    else
      return cmd + (diff > 0 ? step : -step);
  }
/*
  float roundTo(float x, int decimals) {
      float scale = powf(10.0f, decimals);
      return roundf(x * scale) / scale;
  }

  float complementaryPitch(float acc_pitch, float gyro_y, float Ts) {
      static float pitch = 0;
      const float alpha = 0.98f;  // 0.95–0.99
      pitch = alpha * (pitch + gyro_y * Ts) + (1.0f - alpha) * acc_pitch;
      return pitch;
  }
  */
  //
  // ========== THÊM: BLE CALLBACK ==========
  void onBluetoothDataReceived(String data) {
      int seqNum = -1;
      String payload = "";
      
      bool isValidPacket = BleProtocol::parsePacket(data, seqNum, payload);
      
      if (seqNum != -1) {
          if (!isValidPacket) {
              Serial.println("[BLE] CHECKSUM MISMATCH!");
              return;
          }
          
          if (BleProtocol::hasSequenceGap(lastReceivedSeq, seqNum)) {
              int gap = (lastReceivedSeq == -1) ? 0 : seqNum - lastReceivedSeq - 1;
              Serial.printf("[BLE] PACKET LOSS! Gap: %d\n", gap);
          }
          
          lastReceivedSeq = seqNum;
          rxBuffer += payload;
      } else {
          rxBuffer += data;
      }
  }

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

  void processBluetooth() {
    if (rxBuffer.length() == 0) return;
    
    while (true) {
      int startIdx = rxBuffer.indexOf("{{");
      int endIdx = rxBuffer.indexOf("}}");
      
      if (startIdx == -1 || endIdx == -1) break;
      
      String command = rxBuffer.substring(startIdx, endIdx + 2);
      rxBuffer.remove(0, endIdx + 2);
      
      String ax_str = getValue(command, "ax");
      String ay_str = getValue(command, "ay");
      
      if (ax_str.length() > 0 && ay_str.length() > 0) {
        targetPanAngle = ax_str.toFloat();
        targetTiltAngle = ay_str.toFloat();
        trackingDataReceived = true;
        lastTrackingDataTime = millis();
        
        Serial.printf("[TRACK] Pan: %.1f° | Tilt: %.1f°\n", targetPanAngle, targetTiltAngle);
      }
      
      String status = getValue(command, "status");
      if (status == "LOST") {
        Serial.println("[TRACK] Target LOST");
        trackingDataReceived = false;
      }
    }
  }

  //===================================================================================================================================================//
  void setup() {
    Serial.begin(115200);
    Wire.begin(22, 21);

    // --- THÊM: BLUETOOTH INIT ---
    Bluetooth::getInstance()->init("ESP32_Tracker", onBluetoothDataReceived);
    Serial.println("[BLE] Bluetooth Initialized!");

    if (!bno.begin()) {
      Serial.println("Không tìm thấy BNO055!");
      while (1);
    }
    bno.setExtCrystalUse(true);
    bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
    Serial.println("BNO055 OK!");
    // Initial Button
    pinMode(16, INPUT);
    pinMode(19, INPUT);
    // Joystick (ADC) – ESP32 tự nhận
    pinMode(JOY_X, INPUT);
    pinMode(JOY_Y, INPUT);
    initDrivers();
    if (controlMode == MODE_ANGLE) {
      setupAngleOpenLoop();
    } else {
      setupVelocityOpenLoop();
    }
    // setMotorMode(controlMode);
  }
  void readSerialAngleCommand() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("SET")) {
      int idx1 = cmd.indexOf(',');
      int idx2 = cmd.indexOf(',', idx1 + 1);
      int idx3 = cmd.indexOf(',', idx2 + 1);

      if (idx1 > 0 && idx2 > 0 && idx3 > 0) {
        targetYaw   = cmd.substring(idx1 + 1, idx2).toFloat();
        targetPitch = cmd.substring(idx2 + 1, idx3).toFloat();
        targetRoll  = cmd.substring(idx3 + 1).toFloat();
      }
    }
  }
}
  /*
  void disableMotors() {
    motor1.disable();
    motor2.disable();
    motor3.disable();
  }

  void enableMotors() {
    motor1.enable();
    motor2.enable();
    motor3.enable();
  }*/
  //======================================================================================================================================================//
  void loop() {
    //motor1.loopFOC();
    //motor2.loopFOC();
    //motor3.loopFOC();
    // --- THÊM: XỬ LÝ BLE ---
    processBluetooth();
    // ===== ĐỌC NÚT NHẤN =====
    bool btn1_state = digitalRead(BTN1);
    bool btn2_state = digitalRead(BTN2);
    // --- BTN1: GỬI CAPTURE (chỉ trong TRACKING_MODE) + Toggle Yaw/Roll ---
    if (btn1_state == HIGH && btn1_last == LOW) {
      // CAPTURE chỉ gửi khi ở TRACKING_MODE
      if (Bt2_Mode_Flag == TRACKING_MODE) {
        if (Bluetooth::getInstance()->isConnected()) {
          Bluetooth::getInstance()->send("CAPTURE");
          Serial.println("[BLE] → SENT: CAPTURE");
          delay(300);
        }
      }
      // Toggle Yaw/Roll (hoạt động ở mọi mode)
      roll_selected = !roll_selected;
      Serial.println(roll_selected ? "Joystick X → ROLL" : "Joystick X → YAW");
    }
    btn1_last = btn1_state;
    // imu::Quaternion q_prev_stable(1.0f, 0.0f, 0.0f, 0.0f);
    // bool q_prev_valid = false;
    // ===== ĐỌC JOYSTICK =====
    int joyX = analogRead(JOY_X);   // 0 – 4095
    int joyY = analogRead(JOY_Y);   // 0 – 4095

    static unsigned long last_us = micros();
    unsigned long now_us = micros();
    float Ts = (now_us - last_us) * 1e-6f;
    last_us = now_us;
    Ts = constrain(Ts, 0.0005f, 0.002f); // 0.5 ms – 2 ms
  
    if (btn2_state == HIGH) {
    if (!btn2_pressed) {
      btn2_pressed = true;
      btn2_press_time = millis();   // bắt đầu đếm
    }

    // LONG PRESS → RESET
    if (millis() - btn2_press_time >= RESET_HOLD_TIME) {
      Serial.println("ESP RESTART (LONG PRESS)");
      delay(100);
      ESP.restart();
    }
  }
  else {
    // THẢ NÚT
    if (btn2_pressed) {
      unsigned long press_time = millis() - btn2_press_time;
      btn2_pressed = false;
      btn2_press_time = 0;

      // SHORT PRESS → NEXT MODE
      if (press_time >= SHORT_PRESS_MIN && press_time < RESET_HOLD_TIME) {
        Bt2_Mode_Flag += 1;
        Serial.print("MODE → ");
        Serial.println(Bt2_Mode_Flag);
      }
      if(Bt2_Mode_Flag == 5){  
        Bt2_Mode_Flag = 0;
        last_button_mode = 5;
      } 
    }
  }
    // ===== CHỐNG GIỰT KHI ĐỔI MODE =====
    if (Bt2_Mode_Flag != last_mode) {
      last_mode = Bt2_Mode_Flag;
      mode_enter_time = millis();
      //
      yaw_vel_cmd = yaw_vel_target = 0;
      pitch_vel_cmd = pitch_vel_target = 0;
      roll_vel_cmd = roll_vel_target = 0;
      //
      //v1_cmd = v2_cmd = v3_cmd = 0;

      v1_target = motor1.voltage_limit;
      v2_target = motor2.voltage_limit;
      v3_target = motor3.voltage_limit;
      //
      if (Bt2_Mode_Flag == TRACKING_MODE) {
        Tracking_Mode = CAPTURE_ZERO; 
        imu::Quaternion q = bno.getQuat();
        q = normalizeQuat(q);
        float gy = 2 * (q.w()*q.x() + q.y()*q.z());
        float gz = q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();
        saved_roll_target = atan2(gy, gz) * RAD_TO_DEG;
          //
        Serial.printf("[MODE] TRACKING ACTIVATED -> Locked Roll at: %.2f deg\n", saved_roll_target);}
      /*
        if (Bt2_Mode_Flag == STABLE_MODE) 
      {
        Stable_Mode = CAPTURE_ZERO;
      }
      */
    }

      // ===== SOFT START VOLTAGE UPDATE =====
    v1_cmd = rampVoltage(v1_cmd, v1_target, Ts);
    v2_cmd = rampVoltage(v2_cmd, v2_target, Ts);
    v3_cmd = rampVoltage(v3_cmd, v3_target, Ts);

    motor1.voltage_limit = v1_cmd;
    motor2.voltage_limit = v2_cmd;
    motor3.voltage_limit = v3_cmd;

    switch (Bt2_Mode_Flag) {

      case (MANUAL_MODE): {
        // ===== JOYSTICK X =====
        if (!roll_selected) {
          // → YAW (motor1)
          if (joyX > 3200)
            yaw_vel_target = 0.5f;
          else if (joyX < 700)
            yaw_vel_target = -0.5f;
          else
            yaw_vel_target = 0.0f;

          roll_vel_target = 0.0f;
        }
        else {
          // → ROLL (motor3)
          yaw_vel_target = 0.0f;

          if (joyX > 3200)
            roll_vel_target = 0.5f;
          else if (joyX < 700)
            roll_vel_target = -0.5f;
          else
            roll_vel_target = 0.0f;
        }
        if (joyY > 2900)
          pitch_vel_target = -0.5f;
        else if (joyY < 700)
          pitch_vel_target = 0.5f;
        else
          pitch_vel_target = 0.0f;
        // ===== RAMP MƯỢT =====
        roll_vel_cmd  = rampVelocity(roll_vel_cmd, roll_vel_target, Ts * 0.5f);
        pitch_vel_cmd = rampVelocity(pitch_vel_cmd, pitch_vel_target, Ts * 0.5f);
        yaw_vel_cmd   = rampVelocity(yaw_vel_cmd, yaw_vel_target, Ts * 0.5f);
        // Tích lũy vị trí để dùng khi chuyển sang TRACKING_MODE
        accumulated_yaw_pos   += yaw_vel_cmd * Ts;
        accumulated_pitch_pos += pitch_vel_cmd * Ts;
        // ===== GỬI LỆNH ĐIỀU KHIỂN =====
        motor1.move(yaw_vel_cmd);
        motor2.move(pitch_vel_cmd);
        motor3.move(roll_vel_cmd);
        break;
      }     

      case(STABLE_MODE):{

      static imu::Quaternion q_now;
      static unsigned long last_bno_time = 0;       // thời gian lần đọc trước
      const unsigned long BNO_INTERVAL_MS = 50;     // đọc mỗi 10ms (~100Hz)
      unsigned long now_ms = millis();

      if ( last_button_mode == 5) {
        last_button_mode = 0;
        last_bno_time = 0;
        q_now = imu::Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
      }

      if (now_ms - last_bno_time >= BNO_INTERVAL_MS) {
          q_now = normalizeQuat(bno.getQuat());
          last_bno_time = now_ms;
      }

      imu::Quaternion q_offset_inv(q_offset.w(), -q_offset.x(), -q_offset.y(), -q_offset.z());
      q_offset_inv = normalizeQuat(q_offset_inv);
      imu::Quaternion q_error = q_now * q_offset_inv;
      q_error = normalizeQuat(q_error);
      
      

      switch (Stable_Mode) {
          case CAPTURE_ZERO: {
              q_offset = normalizeQuat(bno.getQuat());
              pidRoll.shaft_angle = 0;
              pidPitch.shaft_angle = 0;
              pidYaw.shaft_angle = 0;
              Stable_Mode = BALANCE;
              break;
          }

          case BALANCE: {
              readSerialAngleCommand();
              float qw = constrain(q_error.w(), -1.0f, 1.0f);
              float angle = 2.0f * acosf(qw);   // rad
              float sin_half = sqrtf(1.0f - qw * qw);
              float ex = 0, ey = 0, ez = 0;
              if (sin_half > 1e-6f && !isnan(angle)) {
                  ex = q_error.x() / sin_half * angle;
                  ey = q_error.y() / sin_half * angle;
                  ez = q_error.z() / sin_half * angle;
              }
              // float elec_angleYaw = computeElectricalAngle("Yaw", -ez, targetYaw * DEG_TO_RAD, 10.0f, POLE_PAIRS_1, pidYaw, pidParamYaw);         
              // float elec_anglePitch = computeElectricalAngle("Pitch", -ey, targetPitch * DEG_TO_RAD, 10.0f,POLE_PAIRS,pidPitch,pidParamPitch);
              // float elec_angleRoll = computeElectricalAngle("Roll", ex, targetRoll * DEG_TO_RAD, 10.0f, POLE_PAIRS, pidRoll, pidParamRoll);

              float elec_angleYaw   = computeElectricalAngle("Yaw",   -ez, 0.0f, 10.0f, POLE_PAIRS_1, pidYaw,  pidParamYaw);
              float elec_anglePitch = computeElectricalAngle("Pitch", -ey, 0.0f, 10.0f, POLE_PAIRS, pidPitch, pidParamPitch);
              float elec_angleRoll  = computeElectricalAngle("Roll",   ex, 0.0f, 10.0f, POLE_PAIRS, pidRoll,  pidParamRoll);
              motor3.setPhaseVoltage(7.0f, 0, elec_angleRoll);
              motor2.setPhaseVoltage(9.0f, 0, elec_anglePitch);
              motor1.setPhaseVoltage(7.0f, 0, elec_angleYaw);
              static unsigned long last_send = 0;
              if (now_ms - last_send >= SEND_INTERVAL_MS) {
                  Serial.printf("%lu,%.3f,%.3f,%.3f\n", now_ms, ex*RAD_TO_DEG, ey*RAD_TO_DEG, ez*RAD_TO_DEG);
                  last_send = now_ms;
              }
              break;
                }
            }
      break;
    }

      
      // ========== TRACKING_MODE (VELOCITY-BASED VISUAL SERVOING) ==========
      //
     case(TRACKING_MODE): {
      static unsigned long last_sensor_time = 0;
      const unsigned long SENSOR_INTERVAL_MS = 5; // Đọc nhanh hơn (200Hz)
      unsigned long now_ms = millis();
      
      // Góc Roll thực tế hiện tại
      static float current_abs_roll = 0.0f;

      // 1. ĐỌC CẢM BIẾN
      // Quaternion Based Roll (Fused & Stable)
      if (now_ms - last_sensor_time >= SENSOR_INTERVAL_MS) {
          imu::Quaternion q = bno.getQuat();
          q = normalizeQuat(q);
          
          // Convert to Roll (Gravity projection)
          float gy = 2 * (q.w()*q.x() + q.y()*q.z());
          float gz = q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();
          
          current_abs_roll = atan2(gy, gz) * RAD_TO_DEG;
          last_sensor_time = now_ms;
      }
      float roll_error_to_correct = normalizeAngle(current_abs_roll - saved_roll_target);

      static float prev_error_pan = 0.0f;
      static float prev_error_tilt = 0.0f;
      static float prev_roll_error_d = 0.0f; 
      static unsigned long prev_time_track = 0;

      switch (Tracking_Mode) {
          case CAPTURE_ZERO: {
              prev_error_pan = 0; prev_error_tilt = 0; prev_roll_error_d = 0;
              prev_time_track = millis();
              Tracking_Mode = BALANCE;
              break;
          }

          case BALANCE: {
              float dt = (now_ms - prev_time_track) / 1000.0f;
              prev_time_track = now_ms;
              if (dt <= 0 || dt > 0.1f) dt = 0.02f;
              
              if (trackingDataReceived && (now_ms - lastTrackingDataTime > TRACKING_TIMEOUT_MS)) {
                  trackingDataReceived = false;
              }

              // --- A. PAN & TILT (AUTO TRACKING) ---
              float vel_pan_rad = 0; 
              float vel_tilt_rad = 0;
              if (trackingDataReceived) {
                  
                  const float Kp_Pan = 0.9f; const float Kd_Pan = 0.3f; // Đáp ứng chậm, nhưng ổn định
                  const float Kp_Tilt = 2.5f; const float Kd_Tilt = 0.8f; // Đáp ứng nhanh và ổn định
                  const float MAX_VEL = 25.0f; const float DEADBAND = 1.0f;
                  
                  static float f_d_pan=0, f_d_tilt=0;
                  const float tau=0.05f; float alpha=tau/(tau+dt);
                  
                  float err_p=targetPanAngle; float err_t=targetTiltAngle;
                  if(fabs(err_p)<1.0f) err_p=0; if(fabs(err_t)<DEADBAND) err_t=0;

                  float d_p=(err_p-prev_error_pan)/dt; float d_t=(err_t-prev_error_tilt)/dt;
                  prev_error_pan=err_p; prev_error_tilt=err_t;
                  
                  f_d_pan=alpha*f_d_pan+(1-alpha)*d_p; f_d_tilt=alpha*f_d_tilt+(1-alpha)*d_t;

                  float vp=(Kp_Pan*err_p + Kd_Pan*f_d_pan);
                  float vt=-(Kp_Tilt*err_t + Kd_Tilt*f_d_tilt);

                  vel_pan_rad = constrain(vp, -MAX_VEL, MAX_VEL) * DEG_TO_RAD;
                  vel_tilt_rad = constrain(vt, -MAX_VEL, MAX_VEL) * DEG_TO_RAD;
                  
                  accumulated_yaw_pos += vel_pan_rad * dt;
                  accumulated_pitch_pos += vel_tilt_rad * dt;
              } 

             
              // Joystick Remote Control (TRACKING_MODE)
              // Navigation: UP/DOWN (List), LEFT (Open List), RIGHT (Select/Switch Cam)
              static unsigned long last_remote_cmd_time = 0;
              const unsigned long REMOTE_CMD_DEBOUNCE = 300; // 300ms cho navigation nhanh hơn

              if (now_ms - last_remote_cmd_time > REMOTE_CMD_DEBOUNCE) {
                if (Bluetooth::getInstance()->isConnected()) {
                   // UP (Joy Y < 1000) - Joy dẩy lên
                   if (joyY < 1000) {
                      Bluetooth::getInstance()->send("UP");
                      Serial.println("[BLE] → SENT: UP");
                      last_remote_cmd_time = now_ms;
                   }
                   // DOWN (Joy Y > 3000) - Joy dẩy xuống
                   else if (joyY > 3000) {
                      Bluetooth::getInstance()->send("DOWN");
                      Serial.println("[BLE] → SENT: DOWN");
                      last_remote_cmd_time = now_ms;
                   }
                   // LEFT (Joy X < 1000) -> Open Menu
                   else if (joyX < 1000) {
                      Bluetooth::getInstance()->send("LEFT");
                      Serial.println("[BLE] → SENT: LEFT");
                       last_remote_cmd_time = now_ms + 200; 
                   }
                   // RIGHT (Joy X > 3200) -> Select / Switch Cam
                   else if (joyX > 3200) {
                      Bluetooth::getInstance()->send("RIGHT");
                      Serial.println("[BLE] → SENT: RIGHT");
                      last_remote_cmd_time = now_ms + 500; 
                   }
                }
              }
              //Roll
              const float Kp_R = 0.2f; // Stronger Kp
              const float Kd_R = 0.005f; // Good damping
              
              float d_roll_raw = (roll_error_to_correct - prev_roll_error_d) / dt;
              prev_roll_error_d = roll_error_to_correct;
              
              const float tau_r = 0.006f; 
              static float d_roll_f = 0;
              float alpha_r = tau_r / (tau_r + dt);
              d_roll_f = alpha_r * d_roll_f + (1 - alpha_r) * d_roll_raw;
              
              float vel_roll = (Kp_R * roll_error_to_correct + Kd_R * d_roll_f);
              vel_roll = constrain(vel_roll, -10.0f, 10.0f);

              float vel_roll_rad = vel_roll * DEG_TO_RAD;
  // ############################################################################################################
              // --- C. XUẤT RA MOTOR ---
              motor1.move(vel_pan_rad);   
              motor2.move(vel_tilt_rad);  
              motor3.move(vel_roll_rad);  

              // Debug log + BLE Feedback
              static unsigned long last_send = 0;
              if (now_ms - last_send >= SEND_INTERVAL_MS) {
                  Serial.printf("[TRACK] [ERR ANGLE (ĐỘ)] Pan Err: %.1f | Tilt Err: %.1f | Roll Err: %.1f\n",
                              targetPanAngle, targetTiltAngle, roll_error_to_correct);
                  Serial.printf("[TRACK] [VEL CMD (ĐỘ/S)] Pan: %.2f | Tilt: %.2f | Roll: %.2f\n", 
                              vel_pan_rad * RAD_TO_DEG, vel_tilt_rad * RAD_TO_DEG, vel_roll_rad * RAD_TO_DEG);
                  
                  // Gửi Roll error về Android App để hiển thị
                  if (Bluetooth::getInstance()->isConnected()) {
                      char rollMsg[32];
                      sprintf(rollMsg, "{{[roll]:%.1f}}", roll_error_to_correct);
                      Bluetooth::getInstance()->send(rollMsg);
                  }
                  
                  last_send = now_ms;
              }
              break;
          }
    }
        delayMicroseconds(800);
        break;
      }
    }
  }
    
    