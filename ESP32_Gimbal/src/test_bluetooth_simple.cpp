#include <Arduino.h>
#include "Bluetooth.h"

// ======================================================================================
// CONFIGURATION
// ======================================================================================
constexpr uint8_t PIN_BUTTON_CAPTURE = 16;
constexpr unsigned long DEBOUNCE_DELAY = 200; // ms

unsigned long lastDebounceTime = 0;

// ======================================================================================
// CALLBACKS
// ======================================================================================

// Hàm xử lý dữ liệu nhận được từ App
void onDataReceived(String data) {
  Serial.print(">>> [RX] Received from App: ");
  Serial.println(data);
  
  // Kiểm tra nếu là lệnh tracking
  if (data.startsWith("{{") && data.endsWith("}}")) {
      Serial.println("    -> Tracking Data Pattern Detected!");
  } else if (data.indexOf("LOST") != -1) {
      Serial.println("    -> LOST TARGET Message Detected!");
  }
}

// ======================================================================================
// SETUP
// ======================================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n================================================");
  Serial.println("ESP32 BLUETOOTH WRAPPER TEST (NO MOTORS)");
  Serial.println("================================================");

  // 1. Setup Button
  pinMode(PIN_BUTTON_CAPTURE, INPUT_PULLUP);
  Serial.println("[SETUP] GPIO16 initialized as INPUT_PULLUP");

  // 2. Setup Bluetooth
  Serial.println("[SETUP] Initializing Bluetooth...");
  Bluetooth::getInstance()->init("ESP32_Tracker_Test", onDataReceived);
  Serial.println("[SETUP] Bluetooth Initialized. Waiting for App connection...");
  Serial.println("================================================\n");
}

// ======================================================================================
// LOOP
// ======================================================================================
void loop() {
  // Check Bluetooth connection status
  static bool wasConnected = false;
  bool isConnected = Bluetooth::getInstance()->isConnected();
  
  if (isConnected && !wasConnected) {
      Serial.println("\n>>> [STATUS] BLE CONNECTED! Ready to receive data.\n");
      wasConnected = true;
  } else if (!isConnected && wasConnected) {
      Serial.println("\n>>> [STATUS] BLE DISCONNECTED!\n");
      wasConnected = false;
  }

  // Handle Button Press
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
      if (digitalRead(PIN_BUTTON_CAPTURE) == LOW) {
          lastDebounceTime = millis();
          
          Serial.println("\n>>> [BUTTON] GPIO16 Pressed!");
          
          if (isConnected) {
              Serial.println("    -> Sending 'CAPTURE' command...");
              Bluetooth::getInstance()->send("CAPTURE");
              Serial.println("    -> Sent!");
          } else {
              Serial.println("    -> [WARNING] Bluetooth not connected. Cannot send.");
          }
      }
  }

  delay(10); // Small delay
}
