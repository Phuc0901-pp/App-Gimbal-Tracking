#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Nordic UART Service UUIDs
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

class Bluetooth {
public:
    static Bluetooth* getInstance();
    void init(String deviceName, void (*callback)(String));
    void send(String message);
    bool isConnected();

private:
    Bluetooth() {} // Private constructor
    static Bluetooth* instance;
    
    BLEServer* pServer = NULL;
    BLECharacteristic* pTxCharacteristic = NULL;
    bool deviceConnected = false;
    bool oldDeviceConnected = false;
    
    // Callback function pointer
    void (*onDataReceived)(String) = NULL;

    // Friends to access private members
    friend class MyServerCallbacks;
    friend class MyCallbacks;
};

#endif
