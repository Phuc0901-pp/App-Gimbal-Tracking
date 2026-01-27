package com.google.mediapipe.examples.poselandmarker

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView

class LabelAdapter(
    private val labels: List<String>,
    private val onClick: (String) -> Unit
) : RecyclerView.Adapter<LabelAdapter.ViewHolder>() {

    class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
        val tvName: TextView = view.findViewById(android.R.id.text1)
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        // Dùng layout có sẵn của Android cho nhanh
        val view = LayoutInflater.from(parent.context)
            .inflate(android.R.layout.simple_list_item_1, parent, false)
        return ViewHolder(view)
    }

    private var selectedIndex = 0

    fun setSelection(index: Int) {
        selectedIndex = index
        notifyDataSetChanged()
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        val label = labels[position]
        holder.tvName.text = label.uppercase()
        
        // Highlight logic
        if (position == selectedIndex) {
            holder.itemView.setBackgroundColor(android.graphics.Color.DKGRAY)
            holder.tvName.setTextColor(android.graphics.Color.WHITE)
        } else {
            holder.itemView.setBackgroundColor(android.graphics.Color.WHITE)
            holder.tvName.setTextColor(android.graphics.Color.BLACK)
        }

        holder.itemView.setOnClickListener { onClick(label) }
    }

    override fun getItemCount() = labels.size
}