package com.google.mediapipe.examples.poselandmarker

import android.content.Context
import android.content.SharedPreferences

/**
 * Manager for persisting application settings to SharedPreferences.
 * 
 * This ensures that user-configured PID values are retained across app restarts,
 * improving user experience by eliminating the need to re-tune settings each time.
 */
class SettingsManager(context: Context) {
    
    private val prefs: SharedPreferences = context.getSharedPreferences(
        PREFS_NAME, 
        Context.MODE_PRIVATE
    )
    
    companion object {
        private const val PREFS_NAME = "gimbal_settings"
        private const val KEY_PID_KP = "pid_kp"
        private const val KEY_PID_KI = "pid_ki"
        private const val KEY_PID_KD = "pid_kd"
    }
    
    /**
     * Save PID controller settings.
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    fun savePIDSettings(kp: Float, ki: Float, kd: Float) {
        prefs.edit()
            .putFloat(KEY_PID_KP, kp)
            .putFloat(KEY_PID_KI, ki)
            .putFloat(KEY_PID_KD, kd)
            .apply()
    }
    
    /**
     * Load PID controller settings.
     * 
     * @return Triple of (kp, ki, kd). Returns defaults if not previously saved.
     */
    fun loadPIDSettings(): Triple<Float, Float, Float> {
        return Triple(
            prefs.getFloat(KEY_PID_KP, Constants.PID.DEFAULT_KP),
            prefs.getFloat(KEY_PID_KI, Constants.PID.DEFAULT_KI),
            prefs.getFloat(KEY_PID_KD, Constants.PID.DEFAULT_KD)
        )
    }
    
    /**
     * Reset PID settings to defaults.
     */
    fun resetPIDSettings() {
        savePIDSettings(
            Constants.PID.DEFAULT_KP,
            Constants.PID.DEFAULT_KI,
            Constants.PID.DEFAULT_KD
        )
    }
}
