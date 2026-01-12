package com.google.mediapipe.examples.poselandmarker

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import com.google.android.material.bottomsheet.BottomSheetDialogFragment
import com.google.android.material.slider.Slider

/**
 * Settings dialog for PID tuning.
 * 
 * Provides user-friendly sliders with recommended ranges and real-time value display.
 */
class SettingsFragment : BottomSheetDialogFragment() {

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        val view = inflater.inflate(R.layout.fragment_settings, container, false)

        // Get initial values from arguments
        val initialKp = arguments?.getFloat(ARG_KP) ?: Constants.PID.DEFAULT_KP
        val initialKi = arguments?.getFloat(ARG_KI) ?: Constants.PID.DEFAULT_KI
        val initialKd = arguments?.getFloat(ARG_KD) ?: Constants.PID.DEFAULT_KD

        // Find views
        val sliderKp = view.findViewById<Slider>(R.id.sliderKp)
        val sliderKi = view.findViewById<Slider>(R.id.sliderKi)
        val sliderKd = view.findViewById<Slider>(R.id.sliderKd)
        val tvKpValue = view.findViewById<TextView>(R.id.tvKpValue)
        val tvKiValue = view.findViewById<TextView>(R.id.tvKiValue)
        val tvKdValue = view.findViewById<TextView>(R.id.tvKdValue)
        val btnSave = view.findViewById<Button>(R.id.btnSaveSettings)
        val btnClose = view.findViewById<Button>(R.id.btnCloseSettings)
        val btnReset = view.findViewById<Button>(R.id.btnResetSettings)

        // Configure sliders with ranges from Constants
        sliderKp.apply {
            valueFrom = Constants.PID.KP_RANGE.start
            valueTo = Constants.PID.KP_RANGE.endInclusive
            stepSize = 0.001f
            value = snapToStep(initialKp, valueFrom, stepSize).coerceIn(valueFrom, valueTo)
        }
        
        sliderKi.apply {
            valueFrom = Constants.PID.KI_RANGE.start
            valueTo = Constants.PID.KI_RANGE.endInclusive
            stepSize = 0.001f
            value = snapToStep(initialKi, valueFrom, stepSize).coerceIn(valueFrom, valueTo)
        }
        
        sliderKd.apply {
            valueFrom = Constants.PID.KD_RANGE.start
            valueTo = Constants.PID.KD_RANGE.endInclusive
            stepSize = 0.001f
            value = snapToStep(initialKd, valueFrom, stepSize).coerceIn(valueFrom, valueTo)
        }

        // Set initial values
        tvKpValue.text = String.format("%.3f", initialKp)
        tvKiValue.text = String.format("%.3f", initialKi)
        tvKdValue.text = String.format("%.3f", initialKd)

        // Add listeners to update value displays
        sliderKp.addOnChangeListener { _, value, _ ->
            tvKpValue.text = String.format("%.3f", value)
        }
        
        sliderKi.addOnChangeListener { _, value, _ ->
            tvKiValue.text = String.format("%.3f", value)
        }
        
        sliderKd.addOnChangeListener { _, value, _ ->
            tvKdValue.text = String.format("%.3f", value)
        }

        // Save button
        btnSave.setOnClickListener {
            try {
                val kp = sliderKp.value
                val ki = sliderKi.value
                val kd = sliderKd.value
                
                // Validate values are in acceptable range
                if (kp !in Constants.PID.KP_RANGE || 
                    ki !in Constants.PID.KI_RANGE || 
                    kd !in Constants.PID.KD_RANGE) {
                    Toast.makeText(context, "Values out of range!", Toast.LENGTH_SHORT).show()
                    return@setOnClickListener
                }
                
                // Return result via Fragment Result API
                val result = Bundle().apply {
                    putFloat(RESULT_KP, kp)
                    putFloat(RESULT_KI, ki)
                    putFloat(RESULT_KD, kd)
                }
                parentFragmentManager.setFragmentResult(REQUEST_KEY, result)
                
                dismiss()
                Toast.makeText(context, "✅ Settings Applied!", Toast.LENGTH_SHORT).show()
            } catch (e: Exception) {
                Toast.makeText(context, "❌ Error: ${e.message}", Toast.LENGTH_SHORT).show()
            }
        }

        // Reset to defaults button
        btnReset.setOnClickListener {
            sliderKp.value = snapToStep(Constants.PID.DEFAULT_KP, sliderKp.valueFrom, sliderKp.stepSize)
            sliderKi.value = snapToStep(Constants.PID.DEFAULT_KI, sliderKi.valueFrom, sliderKi.stepSize)
            sliderKd.value = snapToStep(Constants.PID.DEFAULT_KD, sliderKd.valueFrom, sliderKd.stepSize)
            Toast.makeText(context, "Reset to defaults", Toast.LENGTH_SHORT).show()
        }

        // Close button
        btnClose.setOnClickListener { dismiss() }

        return view
    }

    private fun snapToStep(value: Float, valueFrom: Float, stepSize: Float): Float {
        val steps = ((value - valueFrom) / stepSize).let { kotlin.math.round(it) }
        return valueFrom + (steps * stepSize)
    }

    companion object {
        const val REQUEST_KEY = "settings_request"
        const val RESULT_KP = "result_kp"
        const val RESULT_KI = "result_ki"
        const val RESULT_KD = "result_kd"
        
        private const val ARG_KP = "arg_kp"
        private const val ARG_KI = "arg_ki"
        private const val ARG_KD = "arg_kd"

        fun newInstance(kp: Float, ki: Float, kd: Float): SettingsFragment {
            return SettingsFragment().apply {
                arguments = Bundle().apply {
                    putFloat(ARG_KP, kp)
                    putFloat(ARG_KI, ki)
                    putFloat(ARG_KD, kd)
                }
            }
        }
    }
}
