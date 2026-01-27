package com.google.mediapipe.examples.poselandmarker

import kotlin.math.abs
import kotlin.math.pow

/**
 * One Euro Filter - Industry-standard filter for interactive motion tracking.
 * 
 * Used by Microsoft HoloLens, Meta Quest, and other VR/AR systems.
 * Provides ultra-low latency at low speeds and high responsiveness at high speeds.
 * 
 * Paper: "1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input in Interactive Systems"
 * https://gery.casiez.net/1euro/
 * 
 * @param minCutoff Minimum cutoff frequency (Hz). Lower = smoother when stationary. Default: 1.0
 * @param beta Speed coefficient. Higher = more responsive to fast motion. Default: 0.007
 * @param dCutoff Derivative cutoff frequency (Hz). Default: 1.0
 */
class OneEuroFilter(
    private val minCutoff: Float = 1.0f,
    private val beta: Float = 0.007f,
    private val dCutoff: Float = 1.0f
) {
    private var x: LowPassFilter? = null
    private var dx: LowPassFilter? = null
    private var lastTime: Long = 0
    
    /**
     * Filter a new value
     * @param value New measurement
     * @param timestamp Current timestamp in milliseconds (use System.currentTimeMillis())
     * @return Filtered value
     */
    fun filter(value: Float, timestamp: Long = System.currentTimeMillis()): Float {
        // Initialize on first call
        if (x == null) {
            x = LowPassFilter(alpha(computeAlpha(minCutoff, 1f/60f)), value)
            dx = LowPassFilter(alpha(computeAlpha(dCutoff, 1f/60f)), 0f)
            lastTime = timestamp
            return value
        }
        
        // Compute time delta
        val dt = if (timestamp > lastTime) (timestamp - lastTime) / 1000f else 1f/60f
        lastTime = timestamp
        
        // Estimate derivative (velocity)
        val dValue = (value - x!!.lastValue) / dt
        val edValue = dx!!.filter(dValue, alpha(computeAlpha(dCutoff, dt)))
        
        // Adaptive cutoff based on speed
        val cutoff = minCutoff + beta * abs(edValue)
        
        // Filter the value
        return x!!.filter(value, alpha(computeAlpha(cutoff, dt)))
    }
    
    /**
     * Reset the filter (call when switching targets)
     */
    fun reset() {
        x = null
        dx = null
        lastTime = 0
    }
    
    private fun computeAlpha(cutoff: Float, dt: Float): Float {
        val tau = 1f / (2f * Math.PI.toFloat() * cutoff)
        return 1f / (1f + tau / dt)
    }
    
    private fun alpha(a: Float): Float = a.coerceIn(0f, 1f)
    
    /**
     * Simple low-pass filter helper
     */
    private class LowPassFilter(private var alpha: Float, var lastValue: Float) {
        fun filter(value: Float, newAlpha: Float): Float {
            alpha = newAlpha
            lastValue = alpha * value + (1 - alpha) * lastValue
            return lastValue
        }
    }
}

/**
 * One Euro Filter for RectF (Bounding Box)
 * Applies separate One Euro Filters to each coordinate for smoothest possible tracking.
 */
class OneEuroBoxFilter(
    minCutoff: Float = 0.5f,   // Very smooth when stationary
    beta: Float = 0.005f,      // Moderate speed response
    dCutoff: Float = 1.0f
) {
    private val filterLeft = OneEuroFilter(minCutoff, beta, dCutoff)
    private val filterTop = OneEuroFilter(minCutoff, beta, dCutoff)
    private val filterRight = OneEuroFilter(minCutoff, beta, dCutoff)
    private val filterBottom = OneEuroFilter(minCutoff, beta, dCutoff)
    
    /**
     * Filter a bounding box
     * @param box New measurement
     * @return Smoothed bounding box
     */
    fun filter(box: android.graphics.RectF): android.graphics.RectF {
        val timestamp = System.currentTimeMillis()
        return android.graphics.RectF(
            filterLeft.filter(box.left, timestamp),
            filterTop.filter(box.top, timestamp),
            filterRight.filter(box.right, timestamp),
            filterBottom.filter(box.bottom, timestamp)
        )
    }
    
    /**
     * Reset all filters (call when switching targets)
     */
    fun reset() {
        filterLeft.reset()
        filterTop.reset()
        filterRight.reset()
        filterBottom.reset()
    }
}

/**
 * Double Exponential Smoothing (Holt's Method)
 * 
 * Accounts for both level and trend (velocity).
 * Better than simple EMA for tracking moving objects.
 * 
 * @param alpha Smoothing factor for level (0-1). Lower = smoother.
 * @param gamma Smoothing factor for trend (0-1). Lower = more stable velocity estimate.
 */
class DoubleExponentialFilter(
    private val alpha: Float = 0.3f,
    private val gamma: Float = 0.1f
) {
    private var level: Float = 0f
    private var trend: Float = 0f
    private var initialized = false
    
    /**
     * Filter a new value
     * @param value New measurement
     * @return Filtered value with trend prediction
     */
    fun filter(value: Float): Float {
        if (!initialized) {
            level = value
            trend = 0f
            initialized = true
            return value
        }
        
        val prevLevel = level
        level = alpha * value + (1 - alpha) * (level + trend)
        trend = gamma * (level - prevLevel) + (1 - gamma) * trend
        
        return level + trend  // Predict one step ahead
    }
    
    fun reset() {
        initialized = false
    }
}

/**
 * Double Exponential Filter for RectF (Bounding Box)
 */
class DoubleExponentialBoxFilter(
    alpha: Float = 0.3f,
    gamma: Float = 0.1f
) {
    private val filterLeft = DoubleExponentialFilter(alpha, gamma)
    private val filterTop = DoubleExponentialFilter(alpha, gamma)
    private val filterRight = DoubleExponentialFilter(alpha, gamma)
    private val filterBottom = DoubleExponentialFilter(alpha, gamma)
    
    fun filter(box: android.graphics.RectF): android.graphics.RectF {
        return android.graphics.RectF(
            filterLeft.filter(box.left),
            filterTop.filter(box.top),
            filterRight.filter(box.right),
            filterBottom.filter(box.bottom)
        )
    }
    
    fun reset() {
        filterLeft.reset()
        filterTop.reset()
        filterRight.reset()
        filterBottom.reset()
    }
}
