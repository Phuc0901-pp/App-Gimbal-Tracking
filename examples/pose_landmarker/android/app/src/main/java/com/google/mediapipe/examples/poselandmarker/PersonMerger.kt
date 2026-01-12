package com.google.mediapipe.examples.poselandmarker

import android.graphics.PointF
import android.graphics.RectF
import kotlin.math.max
import kotlin.math.min

/**
 * Utility object for merging face and person detections into PersonInstances.
 * 
 * This handles the logic of combining overlapping face and person bounding boxes
 * into single trackable entities, making it easier to track specific people.
 */
object PersonMerger {
    
    /**
     * Merge face and person detections into PersonInstances.
     * 
     * Algorithm:
     * 1. For each face, find the person box with highest IoU
     * 2. If IoU > threshold, merge them into one PersonInstance
     * 3. If no match, create face-only PersonInstance
     * 4. Create person-only PersonInstances for unmatched persons
     * 
     * @param detections List of YOLO detections (face and person)
     * @return List of PersonInstances
     */
    fun mergeDetections(detections: List<YoloHelper.Recognition>): List<PersonInstance> {
        val faces = detections.filter { it.label == "face" }
        val persons = detections.filter { it.label == "person" }
        val instances = mutableListOf<PersonInstance>()
        val usedPersons = mutableSetOf<Int>()
        
        // Step 1: Merge faces with persons
        faces.forEach { face ->
            var bestPerson: YoloHelper.Recognition? = null
            var bestIoU = 0f
            var bestPersonIdx = -1
            
            persons.forEachIndexed { personIdx, person ->
                if (personIdx !in usedPersons) {
                    val iou = calculateIoU(face.location, person.location)
                    if (iou > bestIoU && iou > Constants.Tracking.MERGE_IOU_THRESHOLD) {
                        bestIoU = iou
                        bestPerson = person
                        bestPersonIdx = personIdx
                    }
                }
            }
            
            if (bestPerson != null) {
                // Merged instance (face + person)
                usedPersons.add(bestPersonIdx)
                instances.add(PersonInstance(
                    id = instances.size,
                    faceBox = face.location,
                    personBox = bestPerson!!.location,
                    centerPoint = PointF(face.location.centerX(), face.location.centerY()),
                    confidence = (face.confidence + bestPerson!!.confidence) / 2f
                ))
            } else {
                // Face-only instance
                instances.add(PersonInstance(
                    id = instances.size,
                    faceBox = face.location,
                    personBox = null,
                    centerPoint = PointF(face.location.centerX(), face.location.centerY()),
                    confidence = face.confidence
                ))
            }
        }
        
        // Step 2: Add remaining persons (no matching face)
        persons.forEachIndexed { idx, person ->
            if (idx !in usedPersons) {
                instances.add(PersonInstance(
                    id = instances.size,
                    faceBox = null,
                    personBox = person.location,
                    centerPoint = PointF(person.location.centerX(), person.location.centerY()),
                    confidence = person.confidence
                ))
            }
        }
        
        return instances
    }
    
    /**
     * Calculate Intersection over Union (IoU) between two bounding boxes.
     * 
     * @param a First bounding box
     * @param b Second bounding box
     * @return IoU value (0.0 to 1.0)
     */
    private fun calculateIoU(a: RectF, b: RectF): Float {
        // Calculate intersection area
        val interLeft = max(a.left, b.left)
        val interTop = max(a.top, b.top)
        val interRight = min(a.right, b.right)
        val interBottom = min(a.bottom, b.bottom)
        
        if (interLeft >= interRight || interTop >= interBottom) {
            return 0f // No intersection
        }
        
        val interArea = (interRight - interLeft) * (interBottom - interTop)
        
        // Calculate union area
        val areaA = a.width() * a.height()
        val areaB = b.width() * b.height()
        val unionArea = areaA + areaB - interArea
        
        return if (unionArea > 0) interArea / unionArea else 0f
    }
    
    /**
     * Find the PersonInstance closest to a given tap point.
     * 
     * @param instances List of PersonInstances
     * @param tapPoint Point where user tapped
     * @param maxDistance Maximum distance to consider (in pixels)
     * @return Closest PersonInstance or null if none within maxDistance
     */
    fun findClosestInstance(
        instances: List<PersonInstance>,
        tapPoint: PointF,
        maxDistance: Float = Constants.Tracking.MAX_TAP_DISTANCE
    ): PersonInstance? {
        return instances
            .filter { it.distanceTo(tapPoint) <= maxDistance }
            .minByOrNull { it.distanceTo(tapPoint) }
    }
    
    /**
     * Match a PersonInstance from previous frame to current frame instances.
     * Uses spatial proximity to maintain tracking across frames.
     * 
     * @param previousInstance PersonInstance from previous frame
     * @param currentInstances List of PersonInstances in current frame
     * @param maxDistance Maximum distance to consider a match
     * @return Matched PersonInstance or null if no match found
     */
    fun matchInstance(
        previousInstance: PersonInstance,
        currentInstances: List<PersonInstance>,
        maxDistance: Float = 150f
    ): PersonInstance? {
        return currentInstances
            .filter { it.distanceTo(previousInstance.centerPoint) <= maxDistance }
            .minByOrNull { it.distanceTo(previousInstance.centerPoint) }
    }
}
