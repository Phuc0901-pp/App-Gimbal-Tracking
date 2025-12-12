#include "Bluetooth.h"

Bluetooth* Bluetooth::instance = nullptr;

// Callback khi thiết bị kết nối/ngắt kết nối
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Bluetooth::getInstance()->setConnected(true);
        Serial.println(">> DA KET NOI BLUETOOTH");
        
        // Gợi ý: Có thể cập nhật tham số kết nối tại đây nếu cần tốc độ cao hơn
    };

    void onDisconnect(BLEServer* pServer) {
        Bluetooth::getInstance()->setConnected(false);
        Serial.println(">> MAT KET NOI. DANG QUANG BA LAI...");
        // Khởi động lại quảng bá để điện thoại tìm thấy lại
        delay(500);
        pServer->getAdvertising()->start();
    }
};

// Callback khi nhận được dữ liệu (RX)
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        // Lấy dữ liệu raw
        std::string rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
            // Chuyển std::string sang Arduino String
            String data = String(rxValue.c_str());
            
            // Gọi callback về main để xử lý
            Bluetooth::getInstance()->triggerCallback(data);
        }
    }
};

Bluetooth::Bluetooth() {}

Bluetooth* Bluetooth::getInstance() {
    if (instance == nullptr) {
        instance = new Bluetooth();
    }
    return instance;
}

void Bluetooth::init(std::string deviceName, void (*callback)(String)) {
    this->dataCallback = callback;

    BLEDevice::init(deviceName);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Tạo Service UART
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Tạo đặc tính TX (Gửi đi)
    pTxCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_TX,
                                        BLECharacteristic::PROPERTY_NOTIFY
                                    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    // Tạo đặc tính RX (Nhận về)
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                             CHARACTERISTIC_UUID_RX,
                                             BLECharacteristic::PROPERTY_WRITE
                                         );
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();
    
    // Cấu hình quảng bá
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); 
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

void Bluetooth::send(String data) {
    if (deviceConnected) {
        pTxCharacteristic->setValue(data.c_str());
        pTxCharacteristic->notify();
    }
}

void Bluetooth::setConnected(bool connected) {
    this->deviceConnected = connected;
}

bool Bluetooth::isConnected() {
    return this->deviceConnected;
}

void Bluetooth::triggerCallback(String data) {
    if (dataCallback != nullptr) {
        dataCallback(data);
    }
}