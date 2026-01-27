package com.google.mediapipe.examples.poselandmarker

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * Enhanced PID Controller with smoothing, anti-windup, and deadband.
 * 
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param minOutput Minimum output value
 * @param maxOutput Maximum output value
 * @param smoothingFactor EMA smoothing factor (0.0 = no smoothing, 1.0 = no history)
 * @param integralLimit Maximum absolute value for integral term (anti-windup)
 * @param deadband Ignore errors smaller than this threshold
 */
class PIDController(
    private var kp: Float,
    private var ki: Float,
    private var kd: Float,
    private val minOutput: Float = Constants.PID.MIN_OUTPUT,
    private val maxOutput: Float = Constants.PID.MAX_OUTPUT,
    private val smoothingFactor: Float = 0.3f,
    private val integralLimit: Float = 50f,
    private val deadband: Float = 5f
) {
    private var prevError: Float = 0f
    private var integral: Float = 0f
    private var lastTime: Long = 0
    private var smoothedOutput: Float = 0f

    fun update(error: Float): Float {
        val currentTime = System.currentTimeMillis()
        if (lastTime == 0L) {
            lastTime = currentTime
            prevError = error
            return 0f
        }

        val dt = (currentTime - lastTime) / 1000f // Convert to seconds
        if (dt <= 0) return 0f

        // 0. Apply deadband - ignore small errors to reduce jitter
        val effectiveError = if (abs(error) < deadband) 0f else error

        // 1. Proportional term
        val pOut = kp * effectiveError

        // 2. Integral term with anti-windup
        integral += effectiveError * dt
        // Anti-windup: clamp integral to prevent excessive accumulation
        integral = max(-integralLimit, min(integralLimit, integral))
        val iOut = ki * integral

        // 3. Derivative term
        val derivative = (effectiveError - prevError) / dt
        val dOut = kd * derivative

        // Total output
        var output = pOut + iOut + dOut

        // Clamp output
        output = max(minOutput, min(maxOutput, output))

        // Apply EMA smoothing to reduce jitter
        smoothedOutput = smoothingFactor * output + (1 - smoothingFactor) * smoothedOutput

        // Update state
        prevError = effectiveError
        lastTime = currentTime

        return smoothedOutput
    }

    fun reset() {
        prevError = 0f
        integral = 0f
        lastTime = 0
        smoothedOutput = 0f
    }
    
    fun setTunings(kp: Float, ki: Float, kd: Float) {
        this.kp = kp
        this.ki = ki
        this.kd = kd
    }
}
