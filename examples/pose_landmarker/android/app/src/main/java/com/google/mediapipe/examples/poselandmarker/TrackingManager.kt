package com.google.mediapipe.examples.poselandmarker

import android.graphics.Color
import android.graphics.PointF
import android.graphics.RectF
import kotlin.math.sqrt

/**
 * Tracking result containing calculated angles and status
 */
data class TrackingResult(
    val panAngle: Float,
    val tiltAngle: Float,
    val status: String,
    val statusColor: Int,
    val trackingError: Float,
    val lastLockLocation: RectF?
)

/**
 * Manages object tracking logic with multi-layer filtering.
 * Extracted from CameraFragment for better code organization.
 */
class TrackingManager(
    private val angleCalculator: AngleCalculator?,
    private val screenWidth: Float,
    private val screenHeight: Float
) {
    // Multi-layer filtering pipeline
    private val kalmanX = KalmanFilter(q = Constants.Filter.DEFAULT_Q, r = Constants.Filter.DEFAULT_R)
    private val kalmanY = KalmanFilter(q = Constants.Filter.DEFAULT_Q, r = Constants.Filter.DEFAULT_R)
    private val emaX = AdaptiveEMAFilter(minAlpha = 0.4f, maxAlpha = 1.0f)  // Increased from 0.15/0.7 for faster response
    private val emaY = AdaptiveEMAFilter(minAlpha = 0.4f, maxAlpha = 1.0f)
    private val deadbandX = IntelligentDeadband(threshold = 2.0f, hysteresis = 0.5f) // Reduced hysteresis to unlock faster
    private val deadbandY = IntelligentDeadband(threshold = 2.0f, hysteresis = 0.5f)
    private val rateLimiterX = RateLimiter(maxDelta = 180.0f) // Effectively disabled (was 15.0f). Hardware handles smoothing.
    private val rateLimiterY = RateLimiter(maxDelta = 180.0f)
    
    private var trackingLabel: String? = null
    private var lastLockedLocation: RectF? = null
    private var framesLost = 0
    private val MAX_FRAMES_LOST = 10
    
    /**
     * Set target to track
     */
    fun setTarget(label: String) {
        trackingLabel = label
        lastLockedLocation = null
        framesLost = 0
        resetFilters()
    }
    
    /**
     * Stop tracking
     */
    fun stopTracking() {
        trackingLabel = null
        lastLockedLocation = null
        framesLost = 0
        resetFilters()
    }
    
    /**
     * Check if currently tracking
     */
    fun isTracking(): Boolean = trackingLabel != null
    
    /**
     * Get current tracking label
     */
    fun getCurrentTarget(): String? = trackingLabel
    
    /**
     * Process detections and return tracking result
     */
    fun processDetections(
        results: List<YoloHelper.Recognition>,
        getCenterPixel: (RectF) -> PointF,
        isFrontCamera: Boolean
    ): TrackingResult {
        if (trackingLabel == null) {
            return TrackingResult(
                panAngle = 0f,
                tiltAngle = 0f,
                status = "No target selected",
                statusColor = Color.WHITE,
                trackingError = 0f,
                lastLockLocation = null
            )
        }
        
        val candidates = results.filter { it.label == trackingLabel }
        
        if (candidates.isEmpty()) {
            // Target lost - use temporal buffer
            framesLost++
            
            return if (framesLost < MAX_FRAMES_LOST) {
                TrackingResult(
                    panAngle = 0f,
                    tiltAngle = 0f,
                    status = "SEARCHING... ($framesLost/$MAX_FRAMES_LOST)",
                    statusColor = Color.YELLOW,
                    trackingError = 0f,
                    lastLockLocation = lastLockedLocation
                )
            } else {
                lastLockedLocation = null
                TrackingResult(
                    panAngle = 0f,
                    tiltAngle = 0f,
                    status = "LOST TARGET...",
                    statusColor = Color.RED,
                    trackingError = 0f,
                    lastLockLocation = null
                )
            }
        }
        
        // Select best candidate
        val selectedTarget = if (lastLockedLocation != null) {
            candidates.minByOrNull {
                calculateDistance(it.location, lastLockedLocation!!)
            } ?: candidates[0]
        } else {
            candidates.maxByOrNull { it.confidence } ?: candidates[0]
        }
        
        lastLockedLocation = selectedTarget.location
        framesLost = 0
        
        // Calculate tracking error
        val screenCenterX = screenWidth / 2f
        val screenCenterY = screenHeight / 2f
        val objCenterRaw = getCenterPixel(selectedTarget.location)
        
        val errorX = objCenterRaw.x - screenCenterX
        val errorY = objCenterRaw.y - screenCenterY
        
        // Multi-layer filtering pipeline
        val kalmanErrorX = kalmanX.update(errorX)
        val kalmanErrorY = kalmanY.update(errorY)
        val emaErrorX = emaX.update(kalmanErrorX)
        val emaErrorY = emaY.update(kalmanErrorY)
        val deadbandErrorX = deadbandX.apply(emaErrorX)
        val deadbandErrorY = deadbandY.apply(emaErrorY)
        val finalErrorX = rateLimiterX.limit(deadbandErrorX)
        val finalErrorY = rateLimiterY.limit(deadbandErrorY)
        
        // Calculate target angles
        val (targetPan, targetTilt) = if (angleCalculator != null) {
            angleCalculator.calculateAngles(finalErrorX, finalErrorY)
        } else {
            Pair(0f, 0f)
        }
        
        // Apply camera facing correction
        val panAngle = if (isFrontCamera) targetPan else -targetPan
        val tiltAngle = targetTilt
        
        // Calculate tracking error magnitude
        val trackingError = sqrt((errorX * errorX + errorY * errorY).toDouble()).toFloat()
        
        return TrackingResult(
            panAngle = panAngle,
            tiltAngle = tiltAngle,
            status = "LOCK: ${selectedTarget.label.uppercase()} | PAN: ${panAngle.toInt()}° TILT: ${tiltAngle.toInt()}°",
            statusColor = Color.GREEN,
            trackingError = trackingError,
            lastLockLocation = lastLockedLocation
        )
    }
    
    /**
     * Reset all filters
     */
    private fun resetFilters() {
        kalmanX.reset()
        kalmanY.reset()
        emaX.reset()
        emaY.reset()
        deadbandX.reset()
        deadbandY.reset()
        rateLimiterX.reset()
        rateLimiterY.reset()
    }
    
    /**
     * Calculate distance between two boxes
     */
    private fun calculateDistance(boxA: RectF, boxB: RectF): Float {
        val dx = boxA.centerX() - boxB.centerX()
        val dy = boxA.centerY() - boxB.centerY()
        return sqrt((dx * dx + dy * dy).toDouble()).toFloat()
    }
}
