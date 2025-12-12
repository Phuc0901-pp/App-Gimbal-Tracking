#include <Arduino.h>
#include "Bluetooth.h" 

// --- BỎ HẾT SIMPLEFOC VÀ PID ĐỂ TEST ---

// Hàm tách giá trị từ chuỗi (Giữ nguyên logic của bạn)
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

// Callback xử lý dữ liệu
void onDataReceived(String data) {
  // 1. Dọn dẹp chuỗi (quan trọng)
  data.trim(); 

  // 2. In dữ liệu thô (Raw) nhận được để xem có bị cắt hay lỗi không
  Serial.print("RAW REC: ");
  Serial.println(data);

  // 3. Kiểm tra trường hợp Mất Dấu
  if (data.indexOf("LOST") != -1) {
     Serial.println(">>> TRANG THAI: MAT DAU (LOST TARGET)");
     return;
  }

  // 4. Giải mã dữ liệu Tracking
  // Lưu ý: Dùng indexOf thay vì endsWith để an toàn hơn nếu có ký tự lạ cuối chuỗi
  if (data.indexOf("{{") != -1 && data.indexOf("}}") != -1) {
      
      String s_x = getValue(data, "x");
      String s_y = getValue(data, "y");
      String s_ax = getValue(data, "ax"); 
      String s_ay = getValue(data, "ay");

      // Kiểm tra xem có đủ dữ liệu không
      if(s_x != "" && s_y != "" && s_ax != "" && s_ay != "") {
          int x = s_x.toInt();
          int y = s_y.toInt();
          int ax = s_ax.toInt(); // Góc pan
          int ay = s_ay.toInt(); // Góc tilt

          // In kết quả đã tách được
          Serial.printf("=> PARSED: X=%d | Y=%d | AngleX=%d | AngleY=%d\n", x, y, ax, ay);
          Serial.println("------------------------------------------------");
      } else {
          Serial.println("=> LOI: Nhan duoc goi tin nhung thieu du lieu!");
      }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- BAT DAU CHE DO TEST BLUETOOTH ---");
  Serial.println("Hay ket noi App va bat Tracking...");

  // Khởi tạo Bluetooth
  Bluetooth::getInstance()->init("ESP32_Tracker", onDataReceived);
}

void loop() {
  // Không làm gì cả, chỉ chờ dữ liệu đến
  delay(10); 
}