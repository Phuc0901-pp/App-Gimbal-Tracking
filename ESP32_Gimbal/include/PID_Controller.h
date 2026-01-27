
// --- CLASS PID CONTROLLER (LÀM MƯỢT CHUYỂN ĐỘNG) ---
// User provided class for smooth position control
class PID_Controller {
  public:
    float Kp; // Độ nhạy
    float Ki; // Tích phân
    float Kd; // Giảm chấn
    
    float current_val = 0.0f; 
    float prev_error = 0.0f;
    float integral = 0.0f;
    unsigned long last_calc_time = 0;

    PID_Controller(float p, float i, float d, float start_val) {
      Kp = p; Ki = i; Kd = d;
      current_val = start_val;
      last_calc_time = millis();
    }

    // Hàm tính toán vị trí tiếp theo
    float compute(float target) {
      unsigned long now = millis();
      float dt = (now - last_calc_time) / 1000.0f; // dt tính bằng giây
      
      if (dt <= 0 || dt > 0.5) { 
         last_calc_time = now;
         return current_val;
      }
      last_calc_time = now;

      // Tính sai số
      float error = target - current_val;

      // P - Tỷ lệ
      float P = Kp * error;

      // I - Tích phân
      integral += error * dt;
      integral = constrain(integral, -5.0f, 5.0f);
      float I = Ki * integral;

      // D - Đạo hàm
      float D = Kd * (error - prev_error) / dt;
      prev_error = error;

      // Output là vận tốc thay đổi vị trí
      float output = P + I + D;

      // Cập nhật vị trí mượt mà
      current_val += output * dt;

      return current_val;
    }
    
    void reset(float start_val) {
        current_val = start_val;
        prev_error = 0;
        integral = 0;
        last_calc_time = millis();
    }
};

// End of class definition
 
