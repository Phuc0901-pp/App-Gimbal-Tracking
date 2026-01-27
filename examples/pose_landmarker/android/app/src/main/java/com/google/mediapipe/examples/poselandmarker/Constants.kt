package com.google.mediapipe.examples.poselandmarker

/**
 * Constants and configuration values for the Gimbal Tracking application.
 * 
 * This file centralizes all magic numbers and hardcoded values to improve
 * maintainability and make it easier to tune the application.
 */
object Constants {
    
    // ========================================================================
    // AI/ML Configuration
    // ========================================================================
    
    object AI {
        /** Path to the YOLOv8 model file in assets */
        const val MODEL_PATH = "best_int8.tflite"
        
        /** Input size for the YOLO model (320x320 for performance) */
        const val INPUT_SIZE = 320
        
        /** Minimum confidence threshold for detections (0.0 - 1.0) */
        const val CONFIDENCE_THRESHOLD = 0.5f
        
        /** IoU threshold for Non-Maximum Suppression (0.0 - 1.0) */
        const val IOU_THRESHOLD = 0.5f
        
        /** Labels that the model can detect */
        val LABELS = listOf("face", "person")
    }
    
    // ========================================================================
    // Camera Configuration
    // ========================================================================
    
    object Camera {
        /** Preview resolution width */
        const val PREVIEW_WIDTH = 640
        
        /** Preview resolution height */
        const val PREVIEW_HEIGHT = 480
        
        /** Analysis resolution width (lower for performance) */
        const val ANALYSIS_WIDTH = 320
        
        /** Analysis resolution height (lower for performance) */
        const val ANALYSIS_HEIGHT = 240
        
        /** Frame skip ratio (process 1 out of every N frames) */
        /** Frame skip ratio (process 1 out of every N frames) */
        const val FRAME_SKIP_RATIO = 2 // [REVERT] Trả về 2 để giảm tải. 1 frame detect, 1 frame predict.
        
        /** Horizontal Field of View in degrees (typical for smartphone cameras) */
        const val HORIZONTAL_FOV = 73.7f
        
        /** Vertical Field of View in degrees (typical for smartphone cameras) */
        const val VERTICAL_FOV = 53.1f
    }
    
    // ========================================================================
    // Tracking Configuration
    // ========================================================================
    
    object Tracking {
        /** IoU threshold for merging face + person into one PersonInstance */
        const val MERGE_IOU_THRESHOLD = 0.3f
        
        /** Maximum distance (pixels) for tap-to-select */
        const val MAX_TAP_DISTANCE = 200f
        
        /** Maximum distance (pixels) for matching person across frames */
        const val MAX_MATCH_DISTANCE = 150f
        
        /** Person instance timeout (milliseconds) */
        const val PERSON_TIMEOUT_MS = 3000L
    }
    
    // ========================================================================
    // PID Controller Configuration
    // ========================================================================
    
    object PID {
        /** Default Proportional gain (Optimized for FPS=9 via MATLAB simulation) */
        const val DEFAULT_KP = 5.0f
        
        /** Default Integral gain (Optimized for FPS=9 via MATLAB simulation) */
        const val DEFAULT_KI = 0.035f
        
        /** Default Derivative gain (Optimized for FPS=9 via MATLAB simulation) */
        const val DEFAULT_KD = 0.44f
        
        /** Minimum output value */
        const val MIN_OUTPUT = -100f
        
        /** Maximum output value */
        const val MAX_OUTPUT = 100f
        
        /** Recommended Kp range (adjusted for optimized values) */
        val KP_RANGE = 0.1f..10.0f
        
        /** Recommended Ki range (adjusted for optimized values) */
        val KI_RANGE = 0.0f..0.5f
        
        /** Recommended Kd range (adjusted for optimized values) */
        val KD_RANGE = 0.0f..1.0f
    }
    
    // ========================================================================
    // Bluetooth Configuration
    // ========================================================================
    
    object Bluetooth {
        /** UART Service UUID (Nordic standard) */
        const val UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
        
        /** RX Characteristic UUID (write to device) */
        const val RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
        
        /** TX Characteristic UUID (read from device) */
        const val TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
        
        /** Client Characteristic Configuration Descriptor UUID */
        const val CCCD_UUID = "00002902-0000-1000-8000-00805f9b34fb"
        
        /** MTU size for BLE connection */
        const val MTU_SIZE = 512
        
        /** Minimum interval between BLE sends (milliseconds) */
        const val MIN_SEND_INTERVAL_MS = 30L
        
        /** Remote capture command */
        const val REMOTE_CAPTURE_COMMAND = "CAPTURE"
        
        /** Lost target status message */
        const val LOST_TARGET_MESSAGE = "{{[status]:LOST}}"
        
        /** Minimum interval between remote captures (milliseconds) */
        const val MIN_CAPTURE_INTERVAL_MS = 2000L
    }
    
    // ========================================================================
    // UI Configuration
    // ========================================================================
    
    object UI {
        /** HUD text size */
        const val HUD_TEXT_SIZE = 40f
        
        /** Bounding box stroke width */
        const val BOX_STROKE_WIDTH = 5f
        
        /** Center crosshair size */
        const val CROSSHAIR_SIZE = 40f
        
        /** Center circle radius */
        const val CENTER_CIRCLE_RADIUS = 15f
        
        /** Flash effect duration (milliseconds) */
        const val FLASH_DURATION_MS = 50L
    }

    // ========================================================================
    // Filter Configuration (Kalman)
    // ========================================================================

    object Filter {
        /** Process noise (Q). Higher = follow measurement closer (more jitter). Lower = smoother (more lag). */
        const val DEFAULT_Q = 0.01f

        /** Measurement noise (R). Higher = trust measurement less (smoother). Lower = trust measurement more (faster). */
        const val DEFAULT_R = 0.1f
    }
    
    // ========================================================================
    // File Storage Configuration
    // ========================================================================
    
    object Storage {
        /** Directory for saved photos */
        const val PHOTO_DIRECTORY = "Pictures/PoseLandmarker"
        
        /** Directory for saved videos */
        const val VIDEO_DIRECTORY = "Movies/PoseLandmarker"
        
        /** Photo file prefix */
        const val PHOTO_PREFIX = "PoseLandmarker_"
        
        /** Video file prefix */
        const val VIDEO_PREFIX = "PoseVideo_"
        
        /** Photo file extension */
        const val PHOTO_EXTENSION = ".jpg"
        
        /** Video file extension */
        const val VIDEO_EXTENSION = ".mp4"
        
        /** Photo MIME type */
        const val PHOTO_MIME_TYPE = "image/jpeg"
        
        /** Video MIME type */
        const val VIDEO_MIME_TYPE = "video/mp4"
    }
    
    // ========================================================================
    // Power Saving Configuration
    // ========================================================================
    
    object PowerSaving {
        /** Time threshold for considering target locked (milliseconds) */
        const val STABLE_LOCK_THRESHOLD_MS = 3000L
        
        /** Error threshold for considering tracking stable (pixels) */
        const val STABLE_ERROR_THRESHOLD_PX = 20f
        
        /** Frame skip ratio in power saving mode */
        const val POWER_SAVE_FRAME_SKIP = 4
        
        /** Normal frame skip ratio */
        const val NORMAL_FRAME_SKIP = 2
    }
}
