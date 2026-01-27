package com.google.mediapipe.examples.poselandmarker

import android.graphics.PointF
import android.graphics.RectF
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Persistent ID Tracker for PersonInstances.
 * 
 * Maintains consistent IDs across frames using spatial matching.
 * When user selects a person, their ID is locked and only that person is tracked.
 */
class PersonTracker {
    
    // Counter for generating unique IDs
    private var nextId = 0
    
    // Previous frame's instances for matching
    private var previousInstances: List<TrackedPerson> = emptyList()
    
    // The ID of the person we're actively tracking (null = not tracking anyone)
    var lockedTargetId: Int? = null
        private set
    
    // Maximum distance (normalized 0..1) for matching between frames
    private val maxMatchDistance = 0.2f
    
    // Minimum IoU for considering a match
    private val minMatchIoU = 0.3f
    
    data class TrackedPerson(
        val id: Int,
        val faceBox: RectF?,
        val personBox: RectF?,
        val centerPoint: PointF,
        val confidence: Float,
        val lastSeen: Long = System.currentTimeMillis()
    ) {
        fun getPrimaryBox(): RectF = faceBox ?: personBox ?: RectF()
        
        fun distanceTo(other: TrackedPerson): Float {
            val dx = centerPoint.x - other.centerPoint.x
            val dy = centerPoint.y - other.centerPoint.y
            return sqrt((dx * dx + dy * dy).toDouble()).toFloat()
        }
        
        fun distanceToPoint(point: PointF): Float {
            val dx = centerPoint.x - point.x
            val dy = centerPoint.y - point.y
            return sqrt((dx * dx + dy * dy).toDouble()).toFloat()
        }
    }
    
    /**
     * Process new detections and assign persistent IDs.
     * 
     * @param rawInstances Raw PersonInstances from PersonMerger
     * @return List of TrackedPerson with persistent IDs
     */
    fun processDetections(rawInstances: List<PersonInstance>): List<TrackedPerson> {
        val currentTime = System.currentTimeMillis()
        val newTracked = mutableListOf<TrackedPerson>()
        val usedPreviousIds = mutableSetOf<Int>()
        
        // Convert raw instances to TrackedPerson candidates
        val candidates = rawInstances.map { instance ->
            TrackedPerson(
                id = -1, // Temporary, will be assigned
                faceBox = instance.faceBox,
                personBox = instance.personBox,
                centerPoint = instance.centerPoint,
                confidence = instance.confidence,
                lastSeen = currentTime
            )
        }
        
        // Match each candidate with previous instances
        for (candidate in candidates) {
            var bestMatch: TrackedPerson? = null
            var bestScore = 0f
            
            for (prev in previousInstances) {
                if (prev.id in usedPreviousIds) continue
                
                // Calculate matching score (IoU + Distance)
                val iou = calculateIoU(candidate.getPrimaryBox(), prev.getPrimaryBox())
                val dist = candidate.distanceTo(prev)
                
                // Score = IoU weight + (1 - normalized distance)
                val score = if (iou > minMatchIoU || dist < maxMatchDistance) {
                    iou * 0.6f + (1f - min(dist / maxMatchDistance, 1f)) * 0.4f
                } else {
                    0f
                }
                
                if (score > bestScore && score > 0.3f) {
                    bestScore = score
                    bestMatch = prev
                }
            }
            
            val assignedId: Int
            if (bestMatch != null) {
                // Matched with previous - keep their ID
                assignedId = bestMatch.id
                usedPreviousIds.add(assignedId)
            } else {
                // New person - assign new ID
                assignedId = nextId++
            }
            
            newTracked.add(candidate.copy(id = assignedId))
        }
        
        // Update previous instances
        previousInstances = newTracked
        
        return newTracked
    }
    
    /**
     * Lock onto a specific person by ID.
     * After locking, only this person can be tracked.
     * 
     * @param personId ID of the person to lock onto
     */
    fun lockTarget(personId: Int) {
        lockedTargetId = personId
    }
    
    /**
     * Find the TrackedPerson closest to a tap point.
     * 
     * @param trackedPersons List of tracked persons
     * @param tapPoint Normalized tap point (0..1)
     * @param maxDistance Maximum distance to consider
     * @return The closest TrackedPerson or null
     */
    fun findPersonAtPoint(
        trackedPersons: List<TrackedPerson>,
        tapPoint: PointF,
        maxDistance: Float = 0.15f
    ): TrackedPerson? {
        return trackedPersons
            .filter { it.distanceToPoint(tapPoint) <= maxDistance }
            .minByOrNull { it.distanceToPoint(tapPoint) }
    }
    
    /**
     * Get the currently locked target from the list.
     * 
     * @param trackedPersons Current frame's tracked persons
     * @return The locked TrackedPerson or null if not found/not locked
     */
    fun getLockedTarget(trackedPersons: List<TrackedPerson>): TrackedPerson? {
        val targetId = lockedTargetId ?: return null
        return trackedPersons.find { it.id == targetId }
    }
    
    /**
     * Unlock the current target (stop tracking specific person).
     */
    fun unlockTarget() {
        lockedTargetId = null
    }
    
    /**
     * Reset all tracking state.
     */
    fun reset() {
        previousInstances = emptyList()
        lockedTargetId = null
        // Don't reset nextId to avoid ID collisions after reset
    }
    
    /**
     * Check if we're currently locked onto someone.
     */
    fun isLocked(): Boolean = lockedTargetId != null
    
    private fun calculateIoU(a: RectF, b: RectF): Float {
        val interLeft = max(a.left, b.left)
        val interTop = max(a.top, b.top)
        val interRight = min(a.right, b.right)
        val interBottom = min(a.bottom, b.bottom)
        
        if (interLeft >= interRight || interTop >= interBottom) return 0f
        
        val interArea = (interRight - interLeft) * (interBottom - interTop)
        val areaA = a.width() * a.height()
        val areaB = b.width() * b.height()
        val unionArea = areaA + areaB - interArea
        
        return if (unionArea > 0) interArea / unionArea else 0f
    }
}
