package com.google.mediapipe.examples.poselandmarker

/**
 * BLE Protocol utilities for reliable packet transmission.
 * 
 * Packet format: {seq:XXXXX,chk:YY,data:...}
 * - seq: Sequence number (0-65535, wraps around)
 * - chk: Checksum (2-digit hex)
 * - data: Payload (tracking data, commands, etc.)
 */
object BleProtocol {
    
    /**
     * Wrap payload with sequence number and checksum
     */
    fun wrapPacket(payload: String, sequenceNumber: Int): String {
        val checksum = calculateChecksum(payload)
        return "{seq:$sequenceNumber,chk:${checksum.toString(16).uppercase().padStart(2, '0')},data:$payload}"
    }
    
    /**
     * Parse packet and validate
     * Returns Triple<sequenceNumber, payload, isValid>
     */
    fun parsePacket(packet: String): Triple<Int, String, Boolean> {
        return try {
            // Expected format: {seq:XXXXX,chk:YY,data:...}
            if (!packet.startsWith("{seq:")) {
                return Triple(-1, "", false)
            }
            
            val seqStart = 5 // after "{seq:"
            val seqEnd = packet.indexOf(",chk:", seqStart)
            if (seqEnd == -1) return Triple(-1, "", false)
            
            val seqNum = packet.substring(seqStart, seqEnd).toIntOrNull() ?: return Triple(-1, "", false)
            
            val chkStart = seqEnd + 5 // after ",chk:"
            val chkEnd = packet.indexOf(",data:", chkStart)
            if (chkEnd == -1) return Triple(-1, "", false)
            
            val checksumStr = packet.substring(chkStart, chkEnd)
            val receivedChecksum = checksumStr.toIntOrNull(16) ?: return Triple(-1, "", false)
            
            val dataStart = chkEnd + 6 // after ",data:"
            val dataEnd = packet.lastIndexOf("}")
            if (dataEnd == -1) return Triple(-1, "", false)
            
            val payload = packet.substring(dataStart, dataEnd)
            val calculatedChecksum = calculateChecksum(payload)
            
            val isValid = receivedChecksum == calculatedChecksum
            
            Triple(seqNum, payload, isValid)
        } catch (e: Exception) {
            Triple(-1, "", false)
        }
    }
    
    /**
     * Calculate CRC8-XOR checksum
     */
    fun calculateChecksum(data: String): Int {
        var crc = 0
        for (char in data) {
            crc = crc xor char.code
        }
        return crc and 0xFF
    }
    
    /**
     * Check if sequence number indicates a gap (packet loss)
     * Handles wraparound at 65535
     */
    fun hasSequenceGap(lastSeq: Int, currentSeq: Int): Boolean {
        if (lastSeq == -1) return false // First packet
        
        val expected = (lastSeq + 1) % 65536
        return currentSeq != expected
    }
    
    /**
     * Calculate sequence gap size
     */
    fun getSequenceGap(lastSeq: Int, currentSeq: Int): Int {
        if (lastSeq == -1) return 0
        
        return if (currentSeq >= lastSeq) {
            currentSeq - lastSeq - 1
        } else {
            // Wraparound case
            (65536 - lastSeq) + currentSeq - 1
        }
    }
}
