package com.google.mediapipe.examples.poselandmarker

import kotlin.math.atan
import kotlin.math.tan

/**
 * Calculator for converting pixel coordinates to Pan/Tilt angles.
 * 
 * This class handles the geometric conversion from 2D screen coordinates
 * to real-world gimbal angles based on camera Field of View (FOV).
 * 
 * @param screenWidth Width of the screen/preview in pixels
 * @param screenHeight Height of the screen/preview in pixels
 * @param horizontalFOV Horizontal Field of View in degrees (default: 60°)
 * @param verticalFOV Vertical Field of View in degrees (default: 45°)
 */
class AngleCalculator(
    private val screenWidth: Float,
    private val screenHeight: Float,
    private val horizontalFOV: Float = Constants.Camera.HORIZONTAL_FOV,
    private val verticalFOV: Float = Constants.Camera.VERTICAL_FOV
) {
    
    // Convert FOV from degrees to radians for calculations
    private val hFovRad = Math.toRadians(horizontalFOV.toDouble())
    private val vFovRad = Math.toRadians(verticalFOV.toDouble())
    
    // Calculate focal length in pixels
    private val focalLengthX = (screenWidth / 2.0) / tan(hFovRad / 2.0)
    private val focalLengthY = (screenHeight / 2.0) / tan(vFovRad / 2.0)
    
    /**
     * Calculate Pan and Tilt angles from pixel offset.
     * 
     * @param errorX Horizontal pixel offset from center (positive = right)
     * @param errorY Vertical pixel offset from center (positive = down)
     * @return Pair of (panAngle, tiltAngle) in degrees
     */
    fun calculateAngles(errorX: Float, errorY: Float): Pair<Float, Float> {
        // Pan angle (horizontal)
        // Using arctan to convert pixel offset to angle
        val panAngle = Math.toDegrees(atan((errorX / focalLengthX).toDouble())).toFloat()
        
        // Tilt angle (vertical)
        // Negative because screen Y increases downward, but tilt angle increases upward
        val tiltAngle = -Math.toDegrees(atan((errorY / focalLengthY).toDouble())).toFloat()
        
        return Pair(panAngle, tiltAngle)
    }
    
    /**
     * Calculate Pan and Tilt angles from absolute pixel coordinates.
     * 
     * @param pixelX X coordinate in pixels
     * @param pixelY Y coordinate in pixels
     * @return Pair of (panAngle, tiltAngle) in degrees
     */
    fun calculateAnglesFromPixels(pixelX: Float, pixelY: Float): Pair<Float, Float> {
        val centerX = screenWidth / 2f
        val centerY = screenHeight / 2f
        
        val errorX = pixelX - centerX
        val errorY = pixelY - centerY
        
        return calculateAngles(errorX, errorY)
    }
    
    /**
     * Calculate the maximum possible Pan angle based on screen width.
     * 
     * @return Maximum pan angle in degrees
     */
    fun getMaxPanAngle(): Float {
        return (horizontalFOV / 2f)
    }
    
    /**
     * Calculate the maximum possible Tilt angle based on screen height.
     * 
     * @return Maximum tilt angle in degrees
     */
    fun getMaxTiltAngle(): Float {
        return (verticalFOV / 2f)
    }
    
    /**
     * Get angle per pixel ratio for Pan.
     * Useful for understanding sensitivity.
     * 
     * @return Degrees per pixel for horizontal movement
     */
    fun getDegreesPerPixelX(): Float {
        return horizontalFOV / screenWidth
    }
    
    /**
     * Get angle per pixel ratio for Tilt.
     * Useful for understanding sensitivity.
     * 
     * @return Degrees per pixel for vertical movement
     */
    fun getDegreesPerPixelY(): Float {
        return verticalFOV / screenHeight
    }
}
