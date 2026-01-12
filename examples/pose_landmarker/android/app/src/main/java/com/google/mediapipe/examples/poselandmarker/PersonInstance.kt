package com.google.mediapipe.examples.poselandmarker

import android.graphics.PointF
import android.graphics.RectF

/**
 * Represents a single person instance that may have both face and person detections.
 * 
 * This class merges face and person bounding boxes into a single trackable entity,
 * making it easier to track a specific person in a crowd.
 * 
 * @param id Unique identifier for this person instance
 * @param faceBox Bounding box for detected face (null if no face detected)
 * @param personBox Bounding box for detected person (null if only face detected)
 * @param centerPoint Center point of the instance (prioritizes face center)
 * @param confidence Combined confidence score
 * @param timestamp When this instance was created
 */
data class PersonInstance(
    val id: Int,
    val faceBox: RectF?,
    val personBox: RectF?,
    val centerPoint: PointF,
    val confidence: Float,
    val timestamp: Long = System.currentTimeMillis()
) {
    /**
     * Get the primary bounding box for this person.
     * Prioritizes face box if available, otherwise uses person box.
     */
    fun getPrimaryBox(): RectF {
        return faceBox ?: personBox ?: RectF()
    }
    
    /**
     * Get the best box for tracking (larger box for better stability).
     * Prioritizes person box if available, otherwise uses face box.
     */
    fun getTrackingBox(): RectF {
        return personBox ?: faceBox ?: RectF()
    }
    
    /**
     * Check if this instance contains a given point.
     * Useful for tap-to-select functionality.
     */
    fun contains(point: PointF): Boolean {
        return getPrimaryBox().contains(point.x, point.y)
    }
    
    /**
     * Calculate distance from this instance's center to a given point.
     */
    fun distanceTo(point: PointF): Float {
        val dx = centerPoint.x - point.x
        val dy = centerPoint.y - point.y
        return kotlin.math.sqrt((dx * dx + dy * dy).toDouble()).toFloat()
    }
    
    /**
     * Check if this instance has both face and person detections.
     */
    fun isComplete(): Boolean {
        return faceBox != null && personBox != null
    }
}
