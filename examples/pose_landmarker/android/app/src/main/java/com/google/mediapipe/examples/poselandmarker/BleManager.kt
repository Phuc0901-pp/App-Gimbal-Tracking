package com.google.mediapipe.examples.poselandmarker

import android.annotation.SuppressLint
import android.bluetooth.*
import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import java.util.*

class BleManager(
    private val context: Context,
    private val bluetoothAdapter: BluetoothAdapter?,
    private val listener: Listener
) {
    interface Listener {
        fun onConnected(device: BluetoothDevice)
        fun onConnectFailed(device: BluetoothDevice)
        fun onDisconnected()
        fun onDataReceived(data: String)
        fun onMessage(msg: String) // Để log UI
    }

    companion object {
        val UART_SERVICE_UUID: UUID = UUID.fromString(Constants.Bluetooth.UART_SERVICE_UUID)
        val RX_CHAR_UUID: UUID      = UUID.fromString(Constants.Bluetooth.RX_CHAR_UUID)
        val TX_CHAR_UUID: UUID      = UUID.fromString(Constants.Bluetooth.TX_CHAR_UUID)
    }

    private var bluetoothGatt: BluetoothGatt? = null
    private var rxCharacteristic: BluetoothGattCharacteristic? = null
    private var txCharacteristic: BluetoothGattCharacteristic? = null
    private val mainHandler = Handler(Looper.getMainLooper())
    
    // Auto-reconnect support
    private var lastConnectedAddress: String? = null
    private var reconnectAttempts = 0
    private val MAX_RECONNECT_ATTEMPTS = 3
    private val RECONNECT_DELAY_MS = 2000L
    
    // Reliability tracking
    private var sequenceNumber = 0
    private var lastReceivedSequence = -1
    private var sendTimestamp = 0L

    @SuppressLint("MissingPermission")
    fun connect(deviceAddress: String) {
        if (bluetoothAdapter == null) {
            listener.onMessage("Bluetooth Adapter Null")
            return
        }
        disconnect()
        try {
            val device = bluetoothAdapter.getRemoteDevice(deviceAddress)
            bluetoothGatt = device.connectGatt(context, false, gattCallback)
            lastConnectedAddress = deviceAddress // Save for auto-reconnect
            listener.onMessage("Connecting to ${device.name}...")
        } catch (e: Exception) {
            listener.onMessage("Connect Error: ${e.message}")
        }
    }

    @SuppressLint("MissingPermission")
    fun disconnect() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
        rxCharacteristic = null
        txCharacteristic = null
        listener.onDisconnected()
    }

    @SuppressLint("MissingPermission")
    fun send(message: String) {
        val gatt = bluetoothGatt ?: return
        val rx = rxCharacteristic ?: return

        rx.value = message.toByteArray(Charsets.UTF_8)
        rx.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
        gatt.writeCharacteristic(rx)
    }
    
    /**
     * Send with sequence number and checksum validation
     */
    @SuppressLint("MissingPermission")
    fun sendReliable(message: String): Boolean {
        val gatt = bluetoothGatt ?: run {
            MetricsCollector.recordError(ErrorCodes.ERR_BLE_DISCONNECTED)
            return false
        }
        val rx = rxCharacteristic ?: run {
            MetricsCollector.recordError(ErrorCodes.ERR_BLE_DISCONNECTED)
            return false
        }
        
        return try {
            val packet = BleProtocol.wrapPacket(message, sequenceNumber)
            
            rx.value = packet.toByteArray(Charsets.UTF_8)
            rx.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
            
            val success = gatt.writeCharacteristic(rx)
            
            if (success) {
                sequenceNumber = (sequenceNumber + 1) % 65536
                sendTimestamp = System.currentTimeMillis()
                MetricsCollector.blePacketsSent++
            } else {
                MetricsCollector.recordError(ErrorCodes.ERR_BLE_SEND_FAILED)
            }
            
            success
        } catch (e: Exception) {
            MetricsCollector.recordError(ErrorCodes.ERR_BLE_SEND_FAILED)
            false
        }
    }

    val isConnected: Boolean
        get() = bluetoothGatt != null && rxCharacteristic != null

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            val device = gatt.device
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                reconnectAttempts = 0 // Reset on successful connection
                gatt.requestMtu(Constants.Bluetooth.MTU_SIZE)
                gatt.discoverServices()
                mainHandler.post { listener.onConnected(device) }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                disconnect() // Ensure cleanup
                
                // Auto-reconnect logic
                if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS && lastConnectedAddress != null) {
                    reconnectAttempts++
                    mainHandler.post { 
                        listener.onMessage("Reconnecting... (Attempt $reconnectAttempts/$MAX_RECONNECT_ATTEMPTS)") 
                    }
                    mainHandler.postDelayed({
                        connect(lastConnectedAddress!!)
                    }, RECONNECT_DELAY_MS)
                } else if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
                    mainHandler.post { 
                        listener.onMessage("Auto-reconnect failed after $MAX_RECONNECT_ATTEMPTS attempts") 
                    }
                }
                // mainHandler.post { listener.onDisconnected() } // Đã gọi trong disconnect()
            } else {
                mainHandler.post { listener.onConnectFailed(device) }
            }
        }

        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val service = gatt.getService(UART_SERVICE_UUID)
                if (service != null) {
                    rxCharacteristic = service.getCharacteristic(RX_CHAR_UUID)
                    txCharacteristic = service.getCharacteristic(TX_CHAR_UUID)

                    if (txCharacteristic != null) {
                        gatt.setCharacteristicNotification(txCharacteristic, true)
                        val descriptor = txCharacteristic!!.getDescriptor(
                            UUID.fromString(Constants.Bluetooth.CCCD_UUID)
                        )
                        if (descriptor != null) {
                            descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                            gatt.writeDescriptor(descriptor)
                        }
                    }
                    gatt.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH)
                }
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            if (characteristic.uuid == TX_CHAR_UUID) {
                val data = characteristic.getStringValue(0)
                
                // Try to parse and validate packet
                val (seqNum, payload, isValid) = BleProtocol.parsePacket(data)
                
                if (seqNum != -1) {
                    // It's a formatted packet
                    if (!isValid) {
                        // Checksum mismatch
                        MetricsCollector.bleChecksumErrors++
                        MetricsCollector.recordError(ErrorCodes.ERR_BLE_CHECKSUM_MISMATCH)
                        return
                    }
                    
                    // Check for sequence gap
                    if (BleProtocol.hasSequenceGap(lastReceivedSequence, seqNum)) {
                        val gap = BleProtocol.getSequenceGap(lastReceivedSequence, seqNum)
                        MetricsCollector.blePacketsLost += gap
                        MetricsCollector.bleSequenceErrors++
                        MetricsCollector.recordError(ErrorCodes.ERR_BLE_SEQUENCE_GAP)
                    }
                    
                    lastReceivedSequence = seqNum
                    mainHandler.post { listener.onDataReceived(payload) }
                } else {
                    // Legacy format (no sequence/checksum)
                    mainHandler.post { listener.onDataReceived(data) }
                }
            }
        }
    }
}
