/*
 * ESP32 BLUETOOTH + BUTTON TEST
 * Test kết nối Bluetooth BLE và nút nhấn GPIO16
 * 
 * CHỨC NĂNG:
 * 1. Kết nối Bluetooth BLE với tên "ESP32_Gimbal_Test"
 * 2. Nhấn nút GPIO16 → Gửi "CAPTURE" qua BLE
 * 3. Nhận data từ app → In ra Serial Monitor
 * 
 * WIRING:
 * - GPIO16 ----[Button]---- GND
 * - Nút nhấn nối GPIO16 xuống GND khi nhấn
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================
// BLE CONFIGURATION
// ============================================================
// ============================================================
// BLE CONFIGURATION
// ============================================================
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART Service
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // RX (Write)
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // TX (Notify)

BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
BLECharacteristic* pRxCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ============================================================
// BUTTON CONFIGURATION
// ============================================================
constexpr uint8_t BUTTON_PIN = 16;  // GPIO16 for capture button
constexpr unsigned long DEBOUNCE_DELAY = 200;  // 200ms

bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// Debug: Periodic status logging
unsigned long lastStatusPrint = 0;
constexpr unsigned long STATUS_PRINT_INTERVAL = 5000;  // Print status every 5 seconds


// ============================================================
// BLE SERVER CALLBACKS
// ============================================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("✓ BLE Client Connected!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("✗ BLE Client Disconnected!");
    }
};

// ============================================================
// BLE CHARACTERISTIC CALLBACKS (Nhận data từ app)
// ============================================================
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.print(">>> Received from app: ");
        for (int i = 0; i < value.length(); i++) {
          Serial.print(value[i]);
        }
        Serial.println();
      }
    }
};

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("ESP32 BLUETOOTH + BUTTON TEST");
  Serial.println("========================================");
  
  // --- BUTTON SETUP ---
  Serial.println("\n[SETUP] Configuring GPIO16 as INPUT_PULLUP...");
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Test initial GPIO state
  int initialState = digitalRead(BUTTON_PIN);
  Serial.print("[SETUP] GPIO16 initial state: ");
  Serial.println(initialState == HIGH ? "HIGH (not pressed)" : "LOW (pressed or shorted)");
  
  if (initialState == LOW) {
    Serial.println("[WARNING] GPIO16 is LOW! Check if button is pressed or shorted to GND");
  }
  
  Serial.println("✓ Button GPIO16 initialized (Pull-up)");
  
  // --- BLE SETUP ---
  Serial.println("\n[SETUP] Initializing BLE...");
  Serial.print("[SETUP] Device name: ");
  Serial.println("ESP32_Gimbal_Test");
  
  BLEDevice::init("ESP32_Gimbal_Test");
  Serial.println("[SETUP] ✓ BLE Device initialized");
  
  // Create BLE Server
  Serial.println("[SETUP] Creating BLE Server...");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  Serial.println("[SETUP] ✓ BLE Server created");
  
  // Create BLE Service
  Serial.print("[SETUP] Creating BLE Service with UUID: ");
  Serial.println(SERVICE_UUID);
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.println("[SETUP] ✓ BLE Service created");
  
  // Create BLE Characteristics
  // 1. TX Characteristic (Notify) - Sends data to App
  Serial.print("[SETUP] Creating TX Characteristic: ");
  Serial.println(CHARACTERISTIC_UUID_TX);
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  // 2. RX Characteristic (Write) - Receives data from App
  Serial.print("[SETUP] Creating RX Characteristic: ");
  Serial.println(CHARACTERISTIC_UUID_RX);
  pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                       BLECharacteristic::PROPERTY_WRITE
                     );
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  
  Serial.println("[SETUP] ✓ BLE Characteristics created");
  
  // Start service
  Serial.println("[SETUP] Starting BLE Service...");
  pService->start();
  Serial.println("[SETUP] ✓ BLE Service started");
  
  // Start advertising
  Serial.println("[SETUP] Configuring BLE Advertising...");
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  
  Serial.println("[SETUP] Starting BLE Advertising...");
  BLEDevice::startAdvertising();
  Serial.println("[SETUP] ✓ BLE Advertising started");
  
  Serial.println("\n========================================");
  Serial.println("✓✓✓ SETUP COMPLETE ✓✓✓");
  Serial.println("========================================");
  Serial.println("BLE Status: ADVERTISING");
  Serial.println("Device Name: ESP32_Gimbal_Test");
  Serial.println("GPIO16 Status: READY");
  Serial.println("========================================");
  Serial.println("READY TO TEST!");
  Serial.println("1. Connect from Android app");
  Serial.println("2. Press button on GPIO16 to send CAPTURE");
  Serial.println("========================================\n");
  
  Serial.println("[INFO] Entering main loop...");
  Serial.println("[INFO] Monitoring GPIO16 state every 10ms\n");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // --- HANDLE BLE CONNECTION ---
  // Reconnect if disconnected
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Restarting advertising...");
    oldDeviceConnected = deviceConnected;
  }
  
  // New connection
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  // --- HANDLE BUTTON ---
  int reading = digitalRead(BUTTON_PIN);

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    
    // Log raw state change for debugging (optional, can be noisy)
    // Serial.print("[GPIO] Raw change: ");
    // Serial.println(reading == HIGH ? "HIGH" : "LOW");
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    static int buttonState = HIGH; // Status ổn định hiện tại
    if (reading != buttonState) {
      buttonState = reading;

      // Log verified state change
      Serial.print("[GPIO] Verified State: ");
      Serial.println(buttonState == HIGH ? "HIGH" : "LOW (Pressed)");

      // only toggle the LED if the new button state is LOW
      if (buttonState == LOW) {
        Serial.println("\n========================================");
        Serial.println(">>> BUTTON PRESSED (Verified)!");
        Serial.print(">>> Timestamp: ");
        Serial.println(millis());
        Serial.print(">>> BLE Connected: ");
        Serial.println(deviceConnected ? "YES" : "NO");
        
        if (deviceConnected) {
          // Send CAPTURE command via BLE
          Serial.println(">>> Preparing to send CAPTURE command...");
          if (pTxCharacteristic != nullptr) {
             pTxCharacteristic->setValue("CAPTURE");
             pTxCharacteristic->notify();
             Serial.println(">>> ✓ CAPTURE command sent via BLE");
             Serial.println(">>> Waiting for app response...");
          } else {
             Serial.println(">>> ✗ Error: pTxCharacteristic is NULL!");
          }
        } else {
          Serial.println(">>> ✗ BLE not connected, cannot send!");
        }
        Serial.println("========================================\n");
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
  
  // --- PERIODIC STATUS LOG ---
  // Print status every 5 seconds for debugging
  if (millis() - lastStatusPrint > STATUS_PRINT_INTERVAL) {
    lastStatusPrint = millis();
    
    Serial.println("\n[STATUS] ==================== Periodic Status ====================");
    Serial.print("[STATUS] Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");
    Serial.print("[STATUS] BLE Connected: ");
    Serial.println(deviceConnected ? "YES" : "NO");
    Serial.print("[STATUS] GPIO16 State: ");
    Serial.println(reading == HIGH ? "HIGH (not pressed)" : "LOW (pressed)");
    Serial.println("[STATUS] =========================================================\n");
  }
  
  delay(10);  // Small delay for stability
}
