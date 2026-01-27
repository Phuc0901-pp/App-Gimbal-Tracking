/*********************************************************************
 * Gimbal 2 trục Yaw-Pitch sử dụng quaternion (tính toán bằng độ)
 *********************************************************************/
#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_MPU6050.h>
#define BTN1 16
#define BTN2 19
#define JOY_X 34
#define JOY_Y 35

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
const unsigned long SEND_INTERVAL_MS = 20; // 50 Hz
constexpr float SUPPLY_V = 12.0;

//=========================================================================================================================================================//

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

float yaw_vel_target = 0.0f;
float yaw_vel_cmd    = 0.0f;
float pitch_vel_target = 0.0f;
float pitch_vel_cmd    = 0.0f;
float roll_vel_target = 0.0f;
float roll_vel_cmd    = 0.0f;
const float VEL_RAMP = 2.0f;   // rad/s² (tune: 0.5 – 3)

float targetYaw   = 0.0f;
float targetPitch = 0.0f;
float targetRoll  = 0.0f;

// ===== SOFT START VOLTAGE =====
float v1_cmd = 0.0f, v1_target = 0.0f;
float v2_cmd = 0.0f, v2_target = 0.0f;
float v3_cmd = 0.0f, v3_target = 0.0f;

const float VOLT_RAMP = 5.0f; // V/s (tune: 2–10)

#define MA_SIZE 2
float pid_buffer[MA_SIZE] = {0};
int pid_idx = 0;
float pitch_error = 0.0f;
float roll_error = 0.0f;
float yaw_error = 0.0f;
float filterPID(float input) {
  pid_buffer[pid_idx] = input;
  pid_idx = (pid_idx + 1) % MA_SIZE;
  float sum = 0;
  for (int i = 0; i < MA_SIZE; i++) sum += pid_buffer[i];
  return sum / MA_SIZE;
}

//========================================================================================================================================================//
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
uint8_t Bt2_Mode_Flag = 0;
PIDState pidPitch;
PIDState pidYaw;
PIDState pidRoll;

//PID không tải
PIDParams pidParamPitch = { 4.0f, 0.1f, 0.01f }; 
PIDParams pidParamYaw   = { 3.0f, 0.0f, 0.01f };
PIDParams pidParamRoll  = { 2.0f, 0.2f, 0.001f };
//PID có tải
// PIDParams pidParamPitch = { 80.0f, 1.0f, 0.001f }; 
// PIDParams pidParamYaw   = { 50.0f, 0.0f, 0.001f };
// PIDParams pidParamRoll  = { 70.0f, 1.0f, 0.001f };

float computeElectricalAngle(const char* name, float target_angle, float set_point,
                             float velocity_limit_deg, int pole_pairs,
                             PIDState& state, const PIDParams& params) {

  unsigned long now_us = micros();
  float Ts = (now_us - state.prev_time) * 1e-6f;
  state.prev_time = now_us;

  if (Ts < Ts_min) Ts = Ts_min;
  if (Ts > Ts_max) Ts = Ts_max;

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
  // if (strcmp(name, "Roll") == 0) {
  //   if (abs_err < slow_zone) {
  //     scale = abs_err / slow_zone;
  //     scale = constrain(scale, 0.0f, 1.0f);
  //   }
  // }

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

//===================================================================================================================================================//
void setup() {
  Serial.begin(115200);
  Wire.begin(22, 21);


  if (!bno.begin()) {
    Serial.println("Không tìm thấy BNO055!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
   Serial.println("BNO055 OK!");
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

//======================================================================================================================================================//
void loop() {

  // ===== ĐỌC NÚT NHẤN =====
  bool btn1_state = digitalRead(BTN1);
  bool btn2_state = digitalRead(BTN2);
  if (btn1_state == HIGH && btn1_last == LOW) {
    roll_selected = !roll_selected;   // TOGGLE
    Serial.println(roll_selected ? "Joystick X → ROLL" : "Joystick X → YAW");
  }
  btn1_last = btn1_state;
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

    // SHORT PRESS → MANUAL MODE
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

    // reset velocity
    yaw_vel_cmd = yaw_vel_target = 0;
    pitch_vel_cmd = pitch_vel_target = 0;
    roll_vel_cmd = roll_vel_target = 0;
    // reset voltage ramp
    v1_cmd = v2_cmd = v3_cmd = 0;

    // set target voltage theo mode
    v1_target = motor1.voltage_limit;
    v2_target = motor2.voltage_limit;
    v3_target = motor3.voltage_limit;
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
          yaw_vel_target = 1.0f;
        else if (joyX < 700)
          yaw_vel_target = -1.0f;
        else
          yaw_vel_target = 0.0f;

        roll_vel_target = 0.0f;
      }
      else {
        // → ROLL (motor3)
        yaw_vel_target = 0.0f;

        if (joyX > 3200)
          roll_vel_target = 1.0f;
        else if (joyX < 700)
          roll_vel_target = -1.0f;
        else
          roll_vel_target = 0.0f;
      }
      if (joyY > 2900)
        pitch_vel_target = -1.0f;
      else if (joyY < 700)
        pitch_vel_target = 1.0f;
      else
        pitch_vel_target = 0.0f;
      // ===== RAMP MƯỢT =====
      roll_vel_cmd = rampVelocity(roll_vel_cmd, roll_vel_target, Ts * 0.5f);
      pitch_vel_cmd = rampVelocity(pitch_vel_cmd, pitch_vel_target, Ts * 0.5f);
      yaw_vel_cmd = rampVelocity(yaw_vel_cmd, yaw_vel_target, Ts * 0.5f);
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
  }
}
