package com.google.mediapipe.examples.poselandmarker

/**
 * Standardized error codes for debugging and analytics.
 * Error ranges:
 * - 100-199: Detection/AI errors
 * - 200-299: Filter/Processing errors
 * - 300-399: Bluetooth/Communication errors
 * - 400-499: Camera errors
 * - 500-599: Tracking errors
 */
object ErrorCodes {
    // ========================================================================
    // DETECTION ERRORS (100-199)
    // ========================================================================
    const val ERR_YOLO_INIT_FAILED = 101
    const val ERR_YOLO_INFERENCE_TIMEOUT = 102
    const val ERR_NO_GPU_DELEGATE = 103
    const val ERR_MODEL_LOAD_FAILED = 104
    const val ERR_INFERENCE_FAILED = 105
    
    // ========================================================================
    // FILTER ERRORS (200-299)
    // ========================================================================
    const val ERR_KALMAN_DIVERGENCE = 201
    const val ERR_FILTER_OVERFLOW = 202
    const val ERR_RATE_LIMITER_STUCK = 203
    
    // ========================================================================
    // BLUETOOTH ERRORS (300-399)
    // ========================================================================
    const val ERR_BLE_DISCONNECTED = 301
    const val ERR_BLE_SEND_FAILED = 302
    const val ERR_BLE_MTU_TOO_SMALL = 303
    const val ERR_BLE_PERMISSION_DENIED = 304
    const val ERR_BLE_ADAPTER_NULL = 305
    const val ERR_BLE_PACKET_LOSS = 306
    const val ERR_BLE_CHECKSUM_MISMATCH = 307
    const val ERR_BLE_SEQUENCE_GAP = 308
    const val ERR_BLE_TIMEOUT = 309
    const val ERR_BLE_BUFFER_OVERFLOW = 310
    
    // ========================================================================
    // CAMERA ERRORS (400-499)
    // ========================================================================
    const val ERR_CAMERA_PERMISSION_DENIED = 401
    const val ERR_CAMERA_INIT_FAILED = 402
    const val ERR_FRAME_DROP_EXCESSIVE = 403
    const val ERR_CAMERA_DISCONNECTED = 404
    
    // ========================================================================
    // TRACKING ERRORS (500-599)
    // ========================================================================
    const val ERR_TARGET_LOST = 501
    const val ERR_DETECTION_QUALITY_LOW = 502
    const val ERR_NO_TARGETS_FOUND = 503
    const val ERR_MULTI_TARGET_AMBIGUITY = 504
    
    /**
     * Get human-readable error message for error code
     */
    fun getMessage(code: Int): String = when (code) {
        // Detection errors
        ERR_YOLO_INIT_FAILED -> "YOLO initialization failed"
        ERR_YOLO_INFERENCE_TIMEOUT -> "Inference timeout (>500ms)"
        ERR_NO_GPU_DELEGATE -> "GPU delegate unavailable, using CPU"
        ERR_MODEL_LOAD_FAILED -> "Failed to load AI model"
        ERR_INFERENCE_FAILED -> "Inference execution failed"
        
        // Filter errors
        ERR_KALMAN_DIVERGENCE -> "Kalman filter divergence detected"
        ERR_FILTER_OVERFLOW -> "Filter output overflow"
        ERR_RATE_LIMITER_STUCK -> "Rate limiter stuck"
        
        // Bluetooth errors
        ERR_BLE_DISCONNECTED -> "Bluetooth disconnected"
        ERR_BLE_SEND_FAILED -> "Failed to send BLE data"
        ERR_BLE_MTU_TOO_SMALL -> "BLE MTU too small"
        ERR_BLE_PERMISSION_DENIED -> "Bluetooth permission denied"
        ERR_BLE_ADAPTER_NULL -> "Bluetooth adapter not available"
        ERR_BLE_PACKET_LOSS -> "BLE packet loss detected"
        ERR_BLE_CHECKSUM_MISMATCH -> "BLE checksum validation failed"
        ERR_BLE_SEQUENCE_GAP -> "BLE sequence number gap detected"
        ERR_BLE_TIMEOUT -> "BLE response timeout"
        ERR_BLE_BUFFER_OVERFLOW -> "BLE receive buffer overflow"
        
        // Camera errors
        ERR_CAMERA_PERMISSION_DENIED -> "Camera permission denied"
        ERR_CAMERA_INIT_FAILED -> "Camera initialization failed"
        ERR_FRAME_DROP_EXCESSIVE -> "Frame drop >50%, performance issue"
        ERR_CAMERA_DISCONNECTED -> "Camera disconnected"
        
        // Tracking errors
        ERR_TARGET_LOST -> "Tracking target lost"
        ERR_DETECTION_QUALITY_LOW -> "Detection quality too low"
        ERR_NO_TARGETS_FOUND -> "No targets found in frame"
        ERR_MULTI_TARGET_AMBIGUITY -> "Multiple targets detected, ambiguous"
        
        else -> "Unknown error ($code)"
    }
}
