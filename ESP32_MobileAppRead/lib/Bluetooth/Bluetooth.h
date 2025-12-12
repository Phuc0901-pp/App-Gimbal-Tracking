#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUID chuẩn Nordic UART (Khớp với file BleUartClient.kt trên Android)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class Bluetooth {
private:
    static Bluetooth* instance;
    BLEServer* pServer;
    BLECharacteristic* pTxCharacteristic;
    bool deviceConnected = false;
    
    // Con trỏ hàm callback
    void (*dataCallback)(String) = nullptr;

    Bluetooth();

public:
    static Bluetooth* getInstance();
    void init(std::string deviceName, void (*callback)(String));
    void send(String data);
    
    void setConnected(bool connected);
    void triggerCallback(String data);
    bool isConnected();
};

#endif