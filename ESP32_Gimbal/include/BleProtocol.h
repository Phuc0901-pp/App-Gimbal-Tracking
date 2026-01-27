#ifndef BLE_PROTOCOL_H
#define BLE_PROTOCOL_H

#include <Arduino.h>

/**
 * BLE Protocol utilities for ESP32
 * 
 * Packet format: {seq:XXXXX,chk:YY,data:...}
 */
class BleProtocol {
public:
    /**
     * Parse packet and validate
     * Returns: true if valid, false otherwise
     * Outputs: seqNum, payload
     */
    static bool parsePacket(String packet, int& seqNum, String& payload) {
        seqNum = -1;
        payload = "";
        
        // Expected format: {seq:XXXXX,chk:YY,data:...}
        if (!packet.startsWith("{seq:")) {
            return false;
        }
        
        int seqStart = 5; // after "{seq:"
        int seqEnd = packet.indexOf(",chk:", seqStart);
        if (seqEnd == -1) return false;
        
        seqNum = packet.substring(seqStart, seqEnd).toInt();
        
        int chkStart = seqEnd + 5; // after ",chk:"
        int chkEnd = packet.indexOf(",data:", chkStart);
        if (chkEnd == -1) return false;
        
        String checksumStr = packet.substring(chkStart, chkEnd);
        int receivedChecksum = strtol(checksumStr.c_str(), NULL, 16);
        
        int dataStart = chkEnd + 6; // after ",data:"
        int dataEnd = packet.lastIndexOf("}");
        if (dataEnd == -1) return false;
        
        payload = packet.substring(dataStart, dataEnd);
        int calculatedChecksum = calculateChecksum(payload);
        
        return receivedChecksum == calculatedChecksum;
    }
    
    /**
     * Wrap payload with sequence number and checksum
     */
    static String wrapPacket(String payload, int sequenceNumber) {
        int checksum = calculateChecksum(payload);
        char chkHex[3];
        sprintf(chkHex, "%02X", checksum);
        
        return "{seq:" + String(sequenceNumber) + ",chk:" + String(chkHex) + ",data:" + payload + "}";
    }
    
    /**
     * Calculate CRC8-XOR checksum
     */
    static int calculateChecksum(String data) {
        int crc = 0;
        for (int i = 0; i < data.length(); i++) {
            crc ^= data[i];
        }
        return crc & 0xFF;
    }
    
    /**
     * Check if there's a sequence gap
     */
    static bool hasSequenceGap(int lastSeq, int currentSeq) {
        if (lastSeq == -1) return false; // First packet
        
        int expected = (lastSeq + 1) % 65536;
        return currentSeq != expected;
    }
};

#endif
