package com.google.mediapipe.examples.poselandmarker

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.graphics.Color
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView

class DeviceAdapter(
    private val devices: ArrayList<BluetoothDevice>,
    private val onClick: (BluetoothDevice) -> Unit
) : RecyclerView.Adapter<DeviceAdapter.ViewHolder>() {

    class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
        val tvName: TextView = view.findViewById(android.R.id.text1)
        val tvAddress: TextView = view.findViewById(android.R.id.text2)
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        // Dùng layout 2 dòng có sẵn của Android
        val view = LayoutInflater.from(parent.context)
            .inflate(android.R.layout.simple_list_item_2, parent, false)
        return ViewHolder(view)
    }

    @SuppressLint("MissingPermission")
    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        val device = devices[position]

        // Tên thiết bị (Màu trắng, đậm)
        holder.tvName.text = device.name ?: "Unknown Device"
        holder.tvName.setTextColor(Color.WHITE)
        holder.tvName.typeface = android.graphics.Typeface.DEFAULT_BOLD
        holder.tvName.textSize = 16f

        // Địa chỉ MAC (Màu xám, nhỏ)
        holder.tvAddress.text = device.address
        holder.tvAddress.setTextColor(Color.GRAY)
        holder.tvAddress.textSize = 12f

        holder.itemView.setOnClickListener { onClick(device) }
    }

    override fun getItemCount() = devices.size
}