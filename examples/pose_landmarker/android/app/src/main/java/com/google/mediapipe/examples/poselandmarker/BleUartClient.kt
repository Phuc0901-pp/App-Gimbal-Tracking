package com.google.mediapipe.examples.poselandmarker

import android.annotation.SuppressLint
import android.bluetooth.*
import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import java.util.*

class BleUartClient(
    private val context: Context,
    private val bluetoothAdapter: BluetoothAdapter?,
    private val listener: Listener
) {
    interface Listener {
        fun onConnected(device: BluetoothDevice)
        fun onConnectFailed(device: BluetoothDevice)
        fun onDisconnected()
        fun onDataReceived(data: String)
    }

    companion object {
        val UART_SERVICE_UUID: UUID = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
        val RX_CHAR_UUID: UUID      = UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e") // Ghi (Write)
        val TX_CHAR_UUID: UUID      = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e") // Đọc (Notify)
    }

    private var bluetoothGatt: BluetoothGatt? = null
    private var rxCharacteristic: BluetoothGattCharacteristic? = null
    private var txCharacteristic: BluetoothGattCharacteristic? = null
    private val mainHandler = Handler(Looper.getMainLooper())

    @SuppressLint("MissingPermission")
    fun connect(deviceAddress: String) {
        if (bluetoothAdapter == null) return
        disconnect()
        try {
            val device = bluetoothAdapter.getRemoteDevice(deviceAddress)
            bluetoothGatt = device.connectGatt(context, false, gattCallback)
        } catch (e: IllegalArgumentException) { }
    }

    @SuppressLint("MissingPermission")
    fun disconnect() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
        rxCharacteristic = null
        txCharacteristic = null
    }

    // [TỐI ƯU] Gửi dữ liệu siêu tốc
    @SuppressLint("MissingPermission")
    fun send(message: String) {
        val gatt = bluetoothGatt ?: return
        val rx = rxCharacteristic ?: return

        // Gửi chuỗi thô, không tự động thêm \n để tiết kiệm bytes
        // ESP32 sẽ dùng ký tự }} để nhận biết kết thúc
        rx.value = message.toByteArray(Charsets.UTF_8)

        // QUAN TRỌNG: Gửi không cần phản hồi (nhanh hơn gấp 5 lần)
        rx.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE

        gatt.writeCharacteristic(rx)
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            val device = gatt.device
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                // Yêu cầu tăng kích thước gói tin (MTU) lên 512 byte
                gatt.requestMtu(512)
                // Lưu ý: discoverServices sẽ được gọi sau khi MTU thay đổi xong (onMtuChanged)
                // Nhưng để đơn giản, ta gọi luôn ở đây (đôi khi cần delay nhỏ)
                gatt.discoverServices()
                mainHandler.post { listener.onConnected(device) }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                disconnect()
                mainHandler.post { listener.onDisconnected() }
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

                    // Bật Notify để nhận dữ liệu từ ESP32
                    if (txCharacteristic != null) {
                        gatt.setCharacteristicNotification(txCharacteristic, true)
                        val descriptor = txCharacteristic!!.getDescriptor(
                            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
                        )
                        if (descriptor != null) {
                            descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                            gatt.writeDescriptor(descriptor)
                        }
                    }

                    // [TỐI ƯU] Yêu cầu độ trễ thấp nhất (High Priority)
                    // Giúp giảm ping từ ~100ms xuống ~11ms
                    gatt.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH)
                }
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            if (characteristic.uuid == TX_CHAR_UUID) {
                val data = characteristic.getStringValue(0)
                mainHandler.post { listener.onDataReceived(data) }
            }
        }
    }
}