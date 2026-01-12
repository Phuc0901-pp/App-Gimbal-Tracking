package com.google.mediapipe.examples.poselandmarker

import kotlin.math.abs

/**
 * A simple 1D Kalman Filter implementation with static deadzone.
 * Used to smooth tracking coordinates and reduce jitter.
 *
 * @param q Process noise covariance (trust in the prediction). Higher = faster response, more jitter.
 * @param r Measurement noise covariance (uncertainty in measurement). Higher = smoother, more lag.
 * @param p Initial estimation error covariance.
 * @param staticDeadzone Ignore changes smaller than this threshold (prevents micro-jitter)
 * @param initialValue Initial value of the state.
 */
class KalmanFilter(
    private var q: Float = 0.1f,
    private var r: Float = 0.1f,
    private var p: Float = 0.1f,
    private val staticDeadzone: Float = 5.0f,  // Ignore changes < 5 pixels
    initialValue: Float = 0.0f
) {
    private var x: Float = initialValue // State estimate
    private var k: Float = 0.0f         // Kalman gain

    /**
     * Updates the filter with a new measurement and returns the filtered value.
     * Includes static deadzone to prevent oscillation around stable values.
     */
    fun update(measurement: Float): Float {
        // Static deadzone - ignore micro-movements
        val delta = abs(measurement - x)
        if (delta < staticDeadzone) {
            return x  // No update, keep current stable estimate
        }
        
        // Prediction update
        p += q

        // Measurement update
        k = p / (p + r)
        x += k * (measurement - x)
        p *= (1 - k)

        return x
    }

    /**
     * Resets the filter to a new initial state.
     * Useful when switching targets or after tracking loss.
     */
    fun reset(initialValue: Float = 0.0f) {
        x = initialValue
        p = 0.1f // Reset error covariance
    }

    fun setParameters(newQ: Float, newR: Float) {
        this.q = newQ
        this.r = newR
    }
}
