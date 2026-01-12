package com.google.mediapipe.examples.poselandmarker

/**
 * Snapshot of current system metrics
 */
data class MetricsSnapshot(
    val fps: Float,
    val inferenceTime: Long,
    val frameDropRate: Float,
    val panAngle: Float,
    val tiltAngle: Float,
    val targetDistance: Float,
    val bleConnected: Boolean,
    val bleLatency: Long,
    val errorCount: Int,
    val lastError: String,
    val uptime: Long
)

/**
 * Error log entry
 */
data class ErrorEntry(
    val code: Int,
    val message: String,
    val timestamp: Long
)

/**
 * Singleton metrics collector for performance monitoring
 * Collects FPS, latency, angles, errors, etc.
 */
object MetricsCollector {
    private val errors = mutableListOf<ErrorEntry>()
    private var fpsCounter = 0
    private var lastFpsTime = System.currentTimeMillis()
    private var currentFps = 0f
    
    // Tracking state (updated by CameraFragment)
    var lastInferenceTime = 0L
    var panAngle = 0f
    var tiltAngle = 0f
    var targetDistance = 0f
    var bleConnected = false
    var bleLatency = 0L
    var totalFrames = 0L
    var droppedFrames = 0L
    private var startTime = System.currentTimeMillis()
    
    // BLE reliability metrics
    var blePacketsSent = 0
    var blePacketsLost = 0
    var bleSequenceErrors = 0
    var bleChecksumErrors = 0
    
    /**
     * Record a processed frame (for FPS calculation)
     */
    fun recordFrame() {
        totalFrames++
        fpsCounter++
        val now = System.currentTimeMillis()
        if (now - lastFpsTime >= 1000) {
            currentFps = fpsCounter.toFloat()
            fpsCounter = 0
            lastFpsTime = now
        }
    }
    
    /**
     * Record a dropped frame
     */
    fun recordFrameDrop() {
        droppedFrames++
    }
    
    /**
     * Record an error with code
     */
    fun recordError(code: Int) {
        synchronized(errors) {
            errors.add(ErrorEntry(code, ErrorCodes.getMessage(code), System.currentTimeMillis()))
            if (errors.size > 100) errors.removeAt(0)  // Keep last 100
        }
    }
    
    /**
     * Get current metrics snapshot
     */
    fun getSnapshot(): MetricsSnapshot {
        val dropRate = if (totalFrames > 0) (droppedFrames.toFloat() / totalFrames * 100) else 0f
        return MetricsSnapshot(
            fps = currentFps,
            inferenceTime = lastInferenceTime,
            frameDropRate = dropRate,
            panAngle = panAngle,
            tiltAngle = tiltAngle,
            targetDistance = targetDistance,
            bleConnected = bleConnected,
            bleLatency = bleLatency,
            errorCount = errors.size,
            lastError = errors.lastOrNull()?.message ?: "None",
            uptime = System.currentTimeMillis() - startTime
        )
    }
    
    /**
     * Get recent error entries
     */
    fun getRecentErrors(count: Int = 20): List<ErrorEntry> {
        synchronized(errors) {
            return errors.takeLast(count).reversed()
        }
    }
    
    /**
     * Reset all metrics
     */
    fun reset() {
        errors.clear()
        totalFrames = 0
        droppedFrames = 0
        startTime = System.currentTimeMillis()
    }
}
