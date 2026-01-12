#include "Bluetooth.h"

Bluetooth* Bluetooth::instance = NULL;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Bluetooth::getInstance()->deviceConnected = true;
        Serial.println("[BLE] ✓ Thiết bị đã kết nối");
    };

    void onDisconnect(BLEServer* pServer) {
        Bluetooth::getInstance()->deviceConnected = false;
        Serial.println("[BLE] ✗ Thiết bị đã ngắt kết nối");
        // Reinitalize advertising on disconnect
        pServer->startAdvertising(); 
        Serial.println("[BLE] → Đang quảng bá lại...");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue().c_str(); // Use c_str() to convert std::string to Arduino String if needed, or direct assignment
        
        if (rxValue.length() > 0) {
             Serial.print("[BLE] ← Nhận: ");
             Serial.println(rxValue);
             if(Bluetooth::getInstance()->onDataReceived != NULL) {
                 Bluetooth::getInstance()->onDataReceived(rxValue);
             }
        }
    }
};

Bluetooth* Bluetooth::getInstance() {
    if (instance == NULL) {
        instance = new Bluetooth();
    }
    return instance;
}

void Bluetooth::init(String deviceName, void (*callback)(String)) {
    this->onDataReceived = callback;

    // Create the BLE Device
    BLEDevice::init(deviceName.c_str());

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic for TX (Sending data to App)
    pTxCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_TX,
                                        BLECharacteristic::PROPERTY_NOTIFY
                                    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    // Create a BLE Characteristic for RX (Receiving data from App)
    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                             CHARACTERISTIC_UUID_RX,
                                             BLECharacteristic::PROPERTY_WRITE
                                         );
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("[BLE] ✓ Khởi tạo thành công");
    Serial.print("[BLE] → Tên thiết bị: ");
    Serial.println(deviceName);
    Serial.println("[BLE] ⏳ Đang chờ kết nối từ Android...");
}

void Bluetooth::send(String message) {
    if (deviceConnected) {
        // Note: setValue takes std::string or (uint8_t*, size_t). 
        // Arduino String can be converted using .c_str()
        pTxCharacteristic->setValue((uint8_t*)message.c_str(), message.length());
        pTxCharacteristic->notify();
        Serial.print("[BLE] → Gửi: ");
        Serial.println(message);
    } else {
        Serial.println("[BLE] ✗ Không thể gửi (chưa kết nối)");
    }
}

bool Bluetooth::isConnected() {
    return deviceConnected;
}
