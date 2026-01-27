package com.google.mediapipe.examples.poselandmarker

import android.graphics.RectF

/**
 * Filter for smoothing Bounding Box coordinates using Adaptive EMA.
 * 
 * Applies individual AdaptiveEMAFilters to left, top, right, and bottom coordinates
 * to reduce visual jitter while maintaining responsiveness to large movements.
 */
class BoxFilter(
    minAlpha: Float = 0.3f,  // Moderate smoothing for visual display
    maxAlpha: Float = 0.8f   // Responsive to fast motion
) {
    private val filterLeft = AdaptiveEMAFilter(minAlpha, maxAlpha)
    private val filterTop = AdaptiveEMAFilter(minAlpha, maxAlpha)
    private val filterRight = AdaptiveEMAFilter(minAlpha, maxAlpha)
    private val filterBottom = AdaptiveEMAFilter(minAlpha, maxAlpha)
    
    private var isInitialized = false
    
    /**
     * Update the filter with a new bounding box.
     * 
     * @param box New measurement
     * @return Smoothed bounding box
     */
    fun update(box: RectF): RectF {
        if (!isInitialized) {
            filterLeft.reset(box.left)
            filterTop.reset(box.top)
            filterRight.reset(box.right)
            filterBottom.reset(box.bottom)
            isInitialized = true
            return box
        }
        
        val l = filterLeft.update(box.left)
        val t = filterTop.update(box.top)
        val r = filterRight.update(box.right)
        val b = filterBottom.update(box.bottom)
        
        return RectF(l, t, r, b)
    }
    
    /**
     * Reset the filter (e.g. when tracking is lost/restarted)
     */
    fun reset() {
        isInitialized = false
    }
}
