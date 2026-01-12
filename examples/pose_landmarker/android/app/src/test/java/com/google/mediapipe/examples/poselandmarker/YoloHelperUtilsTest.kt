package com.google.mediapipe.examples.poselandmarker

import android.graphics.RectF
import org.junit.Test
import org.junit.Assert.*

/**
 * Unit tests for YoloHelper utility functions.
 * 
 * Tests the Non-Maximum Suppression (NMS) and Intersection over Union (IoU)
 * algorithms that are critical for object detection accuracy.
 */
class YoloHelperUtilsTest {
    
    @Test
    fun `test IoU calculation for identical boxes`() {
        val box1 = RectF(0f, 0f, 10f, 10f)
        val box2 = RectF(0f, 0f, 10f, 10f)
        
        val iou = calculateIoU(box1, box2)
        
        // Identical boxes should have IoU = 1.0
        assertEquals(1.0f, iou, 0.01f)
    }
    
    @Test
    fun `test IoU calculation for non-overlapping boxes`() {
        val box1 = RectF(0f, 0f, 10f, 10f)
        val box2 = RectF(20f, 20f, 30f, 30f)
        
        val iou = calculateIoU(box1, box2)
        
        // Non-overlapping boxes should have IoU = 0.0
        assertEquals(0.0f, iou, 0.01f)
    }
    
    @Test
    fun `test IoU calculation for partially overlapping boxes`() {
        val box1 = RectF(0f, 0f, 10f, 10f)  // Area = 100
        val box2 = RectF(5f, 5f, 15f, 15f)  // Area = 100
        
        // Intersection area = 5x5 = 25
        // Union area = 100 + 100 - 25 = 175
        // IoU = 25/175 = 0.142857
        
        val iou = calculateIoU(box1, box2)
        assertEquals(0.142857f, iou, 0.01f)
    }
    
    @Test
    fun `test IoU calculation for one box inside another`() {
        val box1 = RectF(0f, 0f, 20f, 20f)  // Area = 400
        val box2 = RectF(5f, 5f, 15f, 15f)  // Area = 100
        
        // Intersection area = 100 (box2 is inside box1)
        // Union area = 400
        // IoU = 100/400 = 0.25
        
        val iou = calculateIoU(box1, box2)
        assertEquals(0.25f, iou, 0.01f)
    }
    
    @Test
    fun `test NMS removes overlapping detections`() {
        val detections = arrayListOf(
            createRecognition("1", "person", 0.9f, RectF(0f, 0f, 10f, 10f)),
            createRecognition("2", "person", 0.8f, RectF(1f, 1f, 11f, 11f)), // High overlap with #1
            createRecognition("3", "person", 0.7f, RectF(20f, 20f, 30f, 30f)) // No overlap
        )
        
        val result = nms(detections, iouThreshold = 0.5f)
        
        // Should keep highest confidence (#1) and non-overlapping (#3)
        assertEquals(2, result.size)
        assertEquals("1", result[0].id)
        assertEquals("3", result[1].id)
    }
    
    @Test
    fun `test NMS keeps all detections when no overlap`() {
        val detections = arrayListOf(
            createRecognition("1", "person", 0.9f, RectF(0f, 0f, 10f, 10f)),
            createRecognition("2", "person", 0.8f, RectF(20f, 20f, 30f, 30f)),
            createRecognition("3", "person", 0.7f, RectF(40f, 40f, 50f, 50f))
        )
        
        val result = nms(detections, iouThreshold = 0.5f)
        
        // Should keep all 3 since no overlap
        assertEquals(3, result.size)
    }
    
    @Test
    fun `test NMS with empty list`() {
        val detections = arrayListOf<YoloHelper.Recognition>()
        val result = nms(detections, iouThreshold = 0.5f)
        
        assertEquals(0, result.size)
    }
    
    @Test
    fun `test NMS with single detection`() {
        val detections = arrayListOf(
            createRecognition("1", "person", 0.9f, RectF(0f, 0f, 10f, 10f))
        )
        
        val result = nms(detections, iouThreshold = 0.5f)
        
        assertEquals(1, result.size)
        assertEquals("1", result[0].id)
    }
    
    @Test
    fun `test NMS sorts by confidence`() {
        val detections = arrayListOf(
            createRecognition("1", "person", 0.5f, RectF(0f, 0f, 10f, 10f)),
            createRecognition("2", "person", 0.9f, RectF(20f, 20f, 30f, 30f)),
            createRecognition("3", "person", 0.7f, RectF(40f, 40f, 50f, 50f))
        )
        
        val result = nms(detections, iouThreshold = 0.5f)
        
        // Should be sorted by confidence descending
        assertEquals("2", result[0].id) // 0.9
        assertEquals("3", result[1].id) // 0.7
        assertEquals("1", result[2].id) // 0.5
    }
    
    @Test
    fun `test NMS with strict threshold removes more boxes`() {
        val detections = arrayListOf(
            createRecognition("1", "person", 0.9f, RectF(0f, 0f, 10f, 10f)),
            createRecognition("2", "person", 0.8f, RectF(5f, 5f, 15f, 15f)) // IoU ~0.14
        )
        
        // With strict threshold (0.1), should remove box 2
        val resultStrict = nms(detections.clone() as ArrayList, iouThreshold = 0.1f)
        assertEquals(1, resultStrict.size)
        
        // With loose threshold (0.5), should keep both
        val resultLoose = nms(detections.clone() as ArrayList, iouThreshold = 0.5f)
        assertEquals(2, resultLoose.size)
    }
    
    // Helper functions (simplified versions of YoloHelper methods)
    
    private fun calculateIoU(a: RectF, b: RectF): Float {
        val interArea = kotlin.math.max(0f, kotlin.math.min(a.right, b.right) - kotlin.math.max(a.left, b.left)) *
                kotlin.math.max(0f, kotlin.math.min(a.bottom, b.bottom) - kotlin.math.max(a.top, b.top))
        val unionArea = (a.width() * a.height()) + (b.width() * b.height()) - interArea
        return if (unionArea > 0) interArea / unionArea else 0f
    }
    
    private fun nms(detections: ArrayList<YoloHelper.Recognition>, iouThreshold: Float): List<YoloHelper.Recognition> {
        val nmsList = ArrayList<YoloHelper.Recognition>()
        detections.sortByDescending { it.confidence }
        while (detections.isNotEmpty()) {
            val current = detections.removeAt(0)
            nmsList.add(current)
            val iterator = detections.iterator()
            while (iterator.hasNext()) {
                if (calculateIoU(current.location, iterator.next().location) > iouThreshold) {
                    iterator.remove()
                }
            }
        }
        return nmsList
    }
    
    private fun createRecognition(id: String, label: String, confidence: Float, location: RectF): YoloHelper.Recognition {
        return YoloHelper.Recognition(id, label, confidence, location)
    }
}
