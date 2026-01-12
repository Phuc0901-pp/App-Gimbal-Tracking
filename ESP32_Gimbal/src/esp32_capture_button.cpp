/*
 * ESP32 REMOTE CAPTURE BUTTON
 * GPIO19 Button → Send "CAPTURE" to Android App via BLE
 * 
 * ADD TO YOUR EXISTING ESP32 CODE
 */

// ============================================================
// 1. DEFINE GPIO PIN & DEBOUNCE
// ============================================================
constexpr uint8_t CAPTURE_BUTTON_PIN = 19;
constexpr unsigned long DEBOUNCE_DELAY = 200;  // 200ms debounce

// Button state
bool lastButtonState = HIGH;  // Pull-up → HIGH when not pressed
unsigned long lastDebounceTime = 0;

// ============================================================
// 2. SETUP - Add to your setup() function
// ============================================================
void setup() {
  // ... your existing setup code ...
  
  // Configure GPIO19 as input with internal pull-up
  pinMode(CAPTURE_BUTTON_PIN, INPUT_PULLUP);
  
  Serial.println("Capture button initialized on GPIO19");
}

// ============================================================
// 3. LOOP - Add to your loop() function
// ============================================================
void loop() {
  // ... your existing loop code (motors, PID, etc.) ...
  
  // Read button state
  bool currentButtonState = digitalRead(CAPTURE_BUTTON_PIN);
  
  // Check if button state changed (with debounce)
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // If enough time has passed since last change
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // Button pressed (LOW because of pull-up)
    if (currentButtonState == LOW && lastButtonState == HIGH) {
      // Send CAPTURE command via Bluetooth
      if (::bleManager.isInitialized && bleManager.isConnected) {
        bleManager.send("CAPTURE");
        Serial.println(">>> CAPTURE command sent to app!");
      } else {
        Serial.println(">>> BLE not connected, cannot send CAPTURE");
      }
    }
  }
  
  lastButtonState = currentButtonState;
  
  // ... rest of your loop code ...
}

// ============================================================
// 4. ALTERNATIVE: Interrupt-Based (More Responsive)
// ============================================================
// If you want instant response without polling in loop()

volatile bool captureRequested = false;

void IRAM_ATTR buttonISR() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  
  // Debounce in ISR
  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY) {
    captureRequested = true;
  }
  last_interrupt_time = interrupt_time;
}

void setup() {
  // ... existing setup ...
  
  pinMode(CAPTURE_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAPTURE_BUTTON_PIN), buttonISR, FALLING);
  
  Serial.println("Capture button with interrupt initialized");
}

void loop() {
  // ... existing loop ...
  
  // Check if capture was requested by button
  if (captureRequested) {
    captureRequested = false;  // Reset flag
    
    if (::bleManager.isInitialized && bleManager.isConnected) {
      bleManager.send("CAPTURE");
      Serial.println(">>> CAPTURE command sent!");
    }
  }
  
  // ... rest of loop ...
}

// ============================================================
// 5. WIRING
// ============================================================
/*
 * ESP32 GPIO19 ----[Button]---- GND
 *                      |
 *                   (Optional: 10kΩ pull-up if not using INPUT_PULLUP)
 * 
 * - Button connects GPIO19 to GND when pressed
 * - Internal pull-up keeps GPIO19 HIGH when not pressed
 * - Pressing button pulls GPIO19 LOW → triggers capture
 */

// ============================================================
// 6. ANDROID APP - Already Has Handler!
// ============================================================
/*
 * Your CameraFragment.kt already has this code (line 119-128):
 * 
 * override fun onDataReceived(data: String) {
 *     if (data.contains(Constants.Bluetooth.REMOTE_CAPTURE_COMMAND) && 
 *         System.currentTimeMillis() - lastCaptureTime > Constants.Bluetooth.MIN_CAPTURE_INTERVAL_MS) {
 *         lastCaptureTime = System.currentTimeMillis()
 *         activity?.runOnUiThread {
 *             takePhoto()
 *             Toast.makeText(context, "Remote Capture!", Toast.LENGTH_SHORT).show()
 *         }
 *     }
 * }
 * 
 * Just need to define REMOTE_CAPTURE_COMMAND in Constants.kt
 */

// ============================================================
// 7. CONSTANTS.KT - Add This
// ============================================================
/*
 * In your Android Constants.kt, add:
 * 
 * object Bluetooth {
 *     const val REMOTE_CAPTURE_COMMAND = "CAPTURE"
 *     const val MIN_CAPTURE_INTERVAL_MS = 1000L  // 1 second debounce
 *     // ... other constants ...
 * }
 */
