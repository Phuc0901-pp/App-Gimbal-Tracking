package com.google.mediapipe.examples.poselandmarker

import kotlin.math.abs
import kotlin.math.min

/**
 * Adaptive EMA (Exponential Moving Average) Filter
 * 
 * Automatically adjusts smoothing based on motion velocity:
 * - Static target: Heavy smoothing (minAlpha = smooth, stable lock)
 * - Fast motion: Light smoothing (maxAlpha = responsive tracking)
 * 
 * @param minAlpha Minimum alpha (maximum smoothing for static targets)
 * @param maxAlpha Maximum alpha (minimum smoothing for fast motion)
 */
class AdaptiveEMAFilter(
    private val minAlpha: Float = 0.15f,  // Max smoothing (static)
    private val maxAlpha: Float = 0.7f    // Min smoothing (fast motion)
) {
    private var lastValue: Float = 0f
    private var lastTimestamp: Long = 0
    
    /**
     * Update filter with new measurement
     * @param measurement New input value
     * @return Smoothed output
     */
    fun update(measurement: Float): Float {
        val now = System.currentTimeMillis()
        
        // Initialize on first call
        if (lastTimestamp == 0L) {
            lastTimestamp = now
            lastValue = measurement
            return measurement
        }
        
        val dt = (now - lastTimestamp) / 1000f
        lastTimestamp = now
        
        // Avoid division by zero
        if (dt <= 0f) return lastValue
        
        // Calculate velocity (rate of change)
        val velocity = abs(measurement - lastValue) / dt
        
        // Adaptive alpha based on velocity
        // velocity = 0 → alpha = minAlpha (smooth)
        // velocity > 50 → alpha = maxAlpha (responsive)
        val normalizedVelocity = min(velocity / 50f, 1f)
        val alpha = minAlpha + (maxAlpha - minAlpha) * normalizedVelocity
        
        // Exponential Moving Average
        val smoothed = alpha * measurement + (1 - alpha) * lastValue
        
        lastValue = smoothed
        return smoothed
    }
    
    /**
     * Reset filter state
     */
    fun reset(initialValue: Float = 0f) {
        lastValue = initialValue
        lastTimestamp = 0
    }
}

/**
 * Intelligent Deadband Filter
 * 
 * Eliminates micro-jitter while preserving intentional motion.
 * Uses hysteresis to prevent toggling at threshold boundary.
 * 
 * @param threshold Center deadband size (errors < this → 0)
 * @param hysteresis Hysteresis band (prevents toggling)
 */
class IntelligentDeadband(
    private val threshold: Float = 3.0f,
    private val hysteresis: Float = 1.5f
) {
    private var wasInsideDeadband = true
    
    /**
     * Apply deadband to input value
     * @param value Input value
     * @return Filtered value (0 if inside deadband, original if outside)
     */
    fun apply(value: Float): Float {
        val absValue = abs(value)
        
        // Hysteresis logic to prevent oscillation
        val effectiveThreshold = if (wasInsideDeadband) {
            threshold + hysteresis  // Need larger value to exit deadband
        } else {
            threshold - hysteresis  // Need smaller value to enter deadband
        }
        
        return if (absValue < effectiveThreshold) {
            wasInsideDeadband = true
            0f  // Inside deadband → output zero
        } else {
            wasInsideDeadband = false
            value  // Outside deadband → pass through
        }
    }
    
    /**
     * Reset filter state
     */
    fun reset() {
        wasInsideDeadband = true
    }
}

/**
 * Rate Limiter (Slew Rate Limiter)
 * 
 * Prevents sudden jumps in output by limiting the maximum change rate.
 * Useful for preventing jerky motion when detection switches targets.
 * 
 * @param maxDelta Maximum allowed change per update
 */
class RateLimiter(
    private val maxDelta: Float = 15.0f  // Max change per frame (degrees)
) {
    private var lastOutput: Float = 0f
    
    /**
     * Limit rate of change
     * @param input Desired input value
     * @return Limited output (clamped to maxDelta per call)
     */
    fun limit(input: Float): Float {
        val delta = input - lastOutput
        
        // Clamp delta to [-maxDelta, +maxDelta]
        val clampedDelta = when {
            delta > maxDelta -> maxDelta
            delta < -maxDelta -> -maxDelta
            else -> delta
        }
        
        lastOutput += clampedDelta
        return lastOutput
    }
    
    /**
     * Reset filter state
     */
    fun reset(value: Float = 0f) {
        lastOutput = value
    }
}
