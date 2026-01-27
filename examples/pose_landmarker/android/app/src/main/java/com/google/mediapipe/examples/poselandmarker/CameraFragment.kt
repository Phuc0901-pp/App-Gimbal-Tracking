package com.google.mediapipe.examples.poselandmarker

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Color
import android.graphics.PointF
import android.graphics.RectF
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.util.Size
import android.view.LayoutInflater
import android.view.MotionEvent
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.core.*
import androidx.camera.video.*
import androidx.camera.video.VideoCapture
import androidx.core.util.Consumer
import androidx.core.content.PermissionChecker
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import com.google.android.material.bottomsheet.BottomSheetDialog
import com.google.mediapipe.examples.poselandmarker.databinding.FragmentCameraBinding
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import kotlin.math.atan
import kotlin.math.roundToInt
import kotlin.math.sqrt

class CameraFragment : Fragment(), YoloHelper.DetectorListener {

    private var _binding: FragmentCameraBinding? = null
    private val binding get() = _binding!!

    private lateinit var yoloHelper: YoloHelper
    private var cameraFacing = CameraSelector.LENS_FACING_BACK
    private var imageCapture: ImageCapture? = null // Thêm ImageCapture
    private var videoCapture: VideoCapture<Recorder>? = null // Thêm VideoCapture
    private var recording: Recording? = null
    private var cameraControl: CameraControl? = null
    private var cameraInfo: CameraInfo? = null
    private var backgroundExecutor: ExecutorService? = null

    // Bluetooth
    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bleScanner: android.bluetooth.le.BluetoothLeScanner? = null
    private var scanDialog: BottomSheetDialog? = null
    private val foundDevices = ArrayList<BluetoothDevice>()
    private var deviceAdapter: DeviceAdapter? = null
    private lateinit var bleManager: BleManager
    private var lastBleSendTime = 0L
    private var lastCaptureTime = 0L // Debounce cho remote capture
    
    // Remote Menu Navigation State
    private var isSelectionDialogOpen = false
    private var currentSelectionIndex = 0
    private var labelAdapter: LabelAdapter? = null
    private var selectionDialog: BottomSheetDialog? = null

    // Tracking & Logic
    private var trackingLabel: String? = null  // Keep for backward compat (face/person filter)
    private var lastLockedLocation: RectF? = null  // Last known position of tracked person
    private var lastDetectedResults: List<YoloHelper.Recognition> = listOf()
    private var currentImgW = 0
    private var currentImgH = 0
    private var currentRotation = 0
    private var trackingManager: TrackingManager? = null
    
    // ===== PERSISTENT ID TRACKER (Da bo - Stateless override) =====
    // private val personTracker = PersonTracker()
    
    // Power Saving Mode
    private var stableLockStartTime = 0L
    private var isInPowerSaveMode = false
    
    // Temporal buffer for occlusion handling
    private var framesLost = 0
    private val MAX_FRAMES_LOST = 10 // Only declare LOST after 10 consecutive frames
    
    // ========================================================================
    // STICKY TRACKING: Velocity Prediction + Anti-Hijack
    // ========================================================================
    private var lastCenterX = 0f
    private var lastCenterY = 0f
    private var velocityX = 0f
    private var velocityY = 0f
    private var lastTrackingTime = 0L
    private val MAX_HIJACK_DISTANCE = 0.15f  // Normalized (0..1). Reject matches farther than 15% of screen
    
    // ===== ADVANCED VISUAL SMOOTHING (One Euro Filter - VR/AR Industry Standard) =====
    // minCutoff: Lower = smoother when stationary (0.3 = very smooth)
    // beta: Higher = more responsive to fast motion (0.005 = moderate)
    private val oneEuroBoxFilter = OneEuroBoxFilter(minCutoff = 0.3f, beta = 0.005f, dCutoff = 1.0f)
    
    // Adaptive frame skip
    private var adaptiveFrameSkip = Constants.Camera.FRAME_SKIP_RATIO

    // Angle Calculator
    private var angleCalculator: AngleCalculator? = null

    // ========================================================================
    // MULTI-LAYER FILTERING PIPELINE (5 Layers for Ultra-Smooth Control)
    // ========================================================================
    
    // Layer 0: One Euro Filter (VR/AR Industry Standard - Adaptive Low-Pass)
    // Best for human motion: smooth when still, responsive when moving
    private val oneEuroX = OneEuroFilter(minCutoff = 0.5f, beta = 0.007f)
    private val oneEuroY = OneEuroFilter(minCutoff = 0.5f, beta = 0.007f)
    
    // Layer 1: Kalman Filters (Statistical Noise Reduction)
    private val kalmanX = KalmanFilter(q = Constants.Filter.DEFAULT_Q, r = Constants.Filter.DEFAULT_R)
    private val kalmanY = KalmanFilter(q = Constants.Filter.DEFAULT_Q, r = Constants.Filter.DEFAULT_R)
    
    // Layer 2: Adaptive EMA (Motion-based Smoothing)
    private val emaX = AdaptiveEMAFilter(minAlpha = 0.05f, maxAlpha = 0.8f)
    private val emaY = AdaptiveEMAFilter(minAlpha = 0.05f, maxAlpha = 0.8f)
    
    // Layer 3: Intelligent Deadband (Eliminate Micro-Jitter at Center)
    private val deadbandX = IntelligentDeadband(threshold = 12.0f, hysteresis = 5.0f)
    private val deadbandY = IntelligentDeadband(threshold = 12.0f, hysteresis = 5.0f)
    
    // Layer 4: Rate Limiter (Prevent Sudden Jumps)
    private val rateLimiterX = RateLimiter(maxDelta = 15.0f)
    private val rateLimiterY = RateLimiter(maxDelta = 15.0f)
    
    // Monitoring
    private var metricsPusher: MetricsPusher? = null
    private val handler = android.os.Handler(android.os.Looper.getMainLooper())
    private var isPushingMetrics = false
    

    private val requestPermLauncher = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) {
        if (it[Manifest.permission.CAMERA] == true) view?.post { setUpCamera() }
    }

    private val enableBtLauncher = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) {
        if (it.resultCode == Activity.RESULT_OK) showScanDialogSafe()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val btManager = requireContext().getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = btManager.adapter

        bleManager = BleManager(requireContext(), bluetoothAdapter, object : BleManager.Listener {
            @SuppressLint("MissingPermission")
            override fun onConnected(d: BluetoothDevice) {
                MetricsCollector.bleConnected = true // [FIX] Update Metrics
                activity?.runOnUiThread { if(_binding!=null){binding.tvHudWarn.text="BT: ${d.name}"; binding.tvHudWarn.setTextColor(Color.GREEN)} }
            }
            override fun onDisconnected() {
                MetricsCollector.bleConnected = false // [FIX] Update Metrics
                activity?.runOnUiThread { 
                    if(_binding!=null){
                        binding.tvHudWarn.text="BT Disconnected" 
                        binding.tvHudWarn.setTextColor(Color.RED)
                        Toast.makeText(context, "⚠️ BLE Disconnected", Toast.LENGTH_SHORT).show()
                        MetricsCollector.recordError(ErrorCodes.ERR_BLE_DISCONNECTED)
                    }
                }
            }
            override fun onConnectFailed(d: BluetoothDevice) {}
            override fun onDataReceived(data: String) {
                // [FIX] Update Metrics for Dashboard Log
                MetricsCollector.lastReceivedData = data
                MetricsCollector.lastReceivedDataTime = System.currentTimeMillis()

                // LOG: In ra mọi data nhận được
                Log.d("BLE_DEBUG", "========== BLE Data Received ==========")
                Log.d("BLE_DEBUG", "Raw data: '$data'")
                Log.d("BLE_DEBUG", "Data length: ${data.length}")
                Log.d("BLE_DEBUG", "Contains CAPTURE: ${data.contains(Constants.Bluetooth.REMOTE_CAPTURE_COMMAND)}")
                Log.d("BLE_DEBUG", "Time since last capture: ${System.currentTimeMillis() - lastCaptureTime}ms")
                Log.d("BLE_DEBUG", "Min interval: ${Constants.Bluetooth.MIN_CAPTURE_INTERVAL_MS}ms")
                
                // Logic Remote Capture
                if (data.contains(Constants.Bluetooth.REMOTE_CAPTURE_COMMAND) && System.currentTimeMillis() - lastCaptureTime > Constants.Bluetooth.MIN_CAPTURE_INTERVAL_MS) {
                    Log.d("BLE_DEBUG", "✓ CAPTURE command accepted! Taking photo...")
                    lastCaptureTime = System.currentTimeMillis()
                    activity?.runOnUiThread {
                        takePhoto()
                        Toast.makeText(context, "Remote Capture!", Toast.LENGTH_SHORT).show()
                    }
                } else {
                    if (data.contains(Constants.Bluetooth.REMOTE_CAPTURE_COMMAND)) {
                        Log.d("BLE_DEBUG", "✗ CAPTURE command rejected - too soon (debounce)")
                    } else {
                        Log.d("BLE_DEBUG", "Data is not a CAPTURE command")
                    }
                }
                
                // [NEW] Joystick Navigation Commands
                // UP: Menu Up OR Start Video Recording
                if (data.contains("UP")) {
                     Log.d("BLE_DEBUG", "✓ CMD: UP")
                     activity?.runOnUiThread {
                        if (isSelectionDialogOpen && labelAdapter != null) {
                            // Menu Navigation
                            currentSelectionIndex = (currentSelectionIndex - 1).coerceAtLeast(0)
                            labelAdapter?.setSelection(currentSelectionIndex)
                            Log.d("BLE_DEBUG", "  -> Menu UP: Index $currentSelectionIndex")
                        } else {
                            // Video Control: START Record (if not recording)
                            if (recording == null) {
                                Log.d("BLE_DEBUG", "  -> Action: START Recording")
                                captureVideo()
                            } else {
                                Log.d("BLE_DEBUG", "  -> Ignored: Already recording")
                            }
                        }
                     }
                }

                // DOWN: Menu Down OR Stop Video Recording
                if (data.contains("DOWN")) {
                     Log.d("BLE_DEBUG", "✓ CMD: DOWN")
                     activity?.runOnUiThread {
                        if (isSelectionDialogOpen && labelAdapter != null) {
                            // Menu Navigation
                            val maxIndex = (labelAdapter?.itemCount ?: 1) - 1
                            currentSelectionIndex = (currentSelectionIndex + 1).coerceAtMost(maxIndex)
                            labelAdapter?.setSelection(currentSelectionIndex)
                            Log.d("BLE_DEBUG", "  -> Menu DOWN: Index $currentSelectionIndex")
                        } else {
                            // Video Control: STOP Record (if recording)
                            if (recording != null) {
                                Log.d("BLE_DEBUG", "  -> Action: STOP Recording")
                                captureVideo()
                            } else {
                                Log.d("BLE_DEBUG", "  -> Ignored: Not recording")
                            }
                        }
                     }
                }

                // LEFT: Open Menu
                if (data.contains("LEFT")) {
                    Log.d("BLE_DEBUG", "✓ CMD: LEFT (Open Menu)")
                    activity?.runOnUiThread {
                        // Debounce mechanism for menu opening
                        if (System.currentTimeMillis() - lastCaptureTime > 500) {
                            if (!isSelectionDialogOpen) {
                                showTargetSelectionDialog() 
                            } else {
                                Log.d("BLE_DEBUG", "  -> Menu already open, ignored")
                            }
                            lastCaptureTime = System.currentTimeMillis()
                        } else {
                            Log.d("BLE_DEBUG", "  -> Debounced LEFT")
                        }
                    }
                }
                
                // RIGHT: Select (if Open) OR Switch Camera (if Closed)
                if (data.contains("RIGHT")) {
                    Log.d("BLE_DEBUG", "✓ CMD: RIGHT (Action)")
                    activity?.runOnUiThread {
                         if (System.currentTimeMillis() - lastCaptureTime > 500) {
                            if (isSelectionDialogOpen) {
                                // SELECT CURRENT ITEM
                                Log.d("BLE_DEBUG", "  -> Action: Select Item Index $currentSelectionIndex")
                                val options = listOf("Start Tracking FACE", "Start Tracking BODY (Person)") // Hardcoded list sync with dialog
                                if (currentSelectionIndex in options.indices) {
                                    handleTrackingSelection(options[currentSelectionIndex])
                                    selectionDialog?.dismiss()
                                }
                            } else {
                                // SWITCH CAMERA
                                Log.d("BLE_DEBUG", "  -> Action: Switch Camera")
                                binding.btnSwitch.performClick()
                                Toast.makeText(context, "Remote: Camera Switch", Toast.LENGTH_SHORT).show()
                            }
                            lastCaptureTime = System.currentTimeMillis()
                         } else {
                            Log.d("BLE_DEBUG", "  -> Debounced RIGHT")
                         }
                    }
                }

                // [NEW] Parse Roll angle from ESP32: {{[roll]:XX.X}}
                if (data.contains("[roll]:")) {
                    try {
                        val rollMatch = Regex("""\[roll\]:(-?\d+\.?\d*)""").find(data)
                        rollMatch?.groupValues?.get(1)?.toFloatOrNull()?.let { rollValue ->
                            MetricsCollector.rollAngle = rollValue
                            Log.d("BLE_DEBUG", "✓ Roll angle received: $rollValue°")
                            
                            // Update UI on main thread
                            activity?.runOnUiThread {
                                if (_binding != null) {
                                    binding.tvHudWarn.text = "Roll: ${rollValue.toInt()}°"
                                }
                            }
                        }
                    } catch (e: Exception) {
                        Log.e("BLE_DEBUG", "Error parsing roll: ${e.message}")
                    }
                }
                
                Log.d("BLE_DEBUG", "=======================================")
            }
            override fun onMessage(msg: String) {}
        })
    }

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View {
        _binding = FragmentCameraBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        backgroundExecutor = Executors.newSingleThreadExecutor()
        try { yoloHelper = YoloHelper(requireContext(), this) } catch(e:Exception){Log.e("CAM", "Yolo Init Fail")}
        
        // Load monitoring settings
        loadMonitoringSettings()

        checkPermissions()

        binding.btnSwitch.setOnClickListener {
            cameraFacing = if (cameraFacing == CameraSelector.LENS_FACING_FRONT) CameraSelector.LENS_FACING_BACK else CameraSelector.LENS_FACING_FRONT
            setUpCamera()
        }

        binding.btnBluetooth.setOnClickListener {
            if (Build.VERSION.SDK_INT >= 31 && ActivityCompat.checkSelfPermission(requireContext(), Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                requestPermLauncher.launch(arrayOf(Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_SCAN))
                return@setOnClickListener
            }
            if (bluetoothAdapter?.isEnabled == true) showScanDialogSafe()
            else enableBtLauncher.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
        }
        
        binding.btnMonitoring.setOnClickListener {
            showMonitoringDialog()
        }

        binding.btnTargets.setOnClickListener { showTargetSelectionDialog() }
        binding.btnCancelTrack.setOnClickListener { stopTracking() }
        binding.btnCapture.setOnClickListener { takePhoto() }
        binding.btnRecordVideo.setOnClickListener { captureVideo() }
    }
    


    private fun stopTracking() {
        trackingLabel = null
        lastLockedLocation = null
        lastLockedLocation = null
        framesLost = 0
        
        // [UPDATE] Set Tracking Status for Monitoring
        MetricsCollector.isTracking = false
        
        // Reset all filters
        oneEuroX.reset()
        oneEuroY.reset()
        oneEuroBoxFilter.reset()
        kalmanX.reset()
        kalmanY.reset()
        emaX.reset()
        emaY.reset()
        deadbandX.reset()
        deadbandY.reset()
        rateLimiterX.reset()
        rateLimiterY.reset()
        
        // Reset velocity prediction
        velocityX = 0f
        velocityY = 0f
        lastCenterX = 0f
        lastCenterY = 0f
        lastTrackingTime = 0L

        // [FIX] Explicitly tell TrackingManager to stop!
        // This ensures trackingManager.isTracking() returns false, blocking BLE.
        trackingManager?.stopTracking()
        
        // Unlock the PersonTracker
        // personTracker.unlockTarget()
        
        binding.btnCancelTrack.visibility = View.GONE
        binding.tvHudLine1.text = "Tracking Stopped"
        binding.tvHudLine1.setTextColor(Color.WHITE)
    }
    
    private fun handleTrackingSelection(selectedItem: String) {
        val label = if (selectedItem.contains("FACE")) "face" else "person"
            trackingLabel = label
            
            // Kích hoạt Tracking Manager
            trackingManager?.setTarget(label)
            
            // [UPDATE] Set Tracking Status for Monitoring
            MetricsCollector.isTracking = true
            
            // RESET TRẠNG THÁI UI
            lastLockedLocation = null 
            framesLost = 0
            
            binding.btnCancelTrack.visibility = View.VISIBLE
            binding.btnCancelTrack.text = "STOP TRACKING"
            
            Toast.makeText(context, "Started Tracking: ${label.uppercase()}", Toast.LENGTH_SHORT).show()
    }

    private fun showTargetSelectionDialog() {
        // Chỉ cho chọn chế độ: Track FACE hoặc Track BODY (Person)
        val options = listOf("Start Tracking FACE", "Start Tracking BODY (Person)")
        
        selectionDialog = BottomSheetDialog(requireContext())
        val rv = androidx.recyclerview.widget.RecyclerView(requireContext())
        rv.layoutManager = androidx.recyclerview.widget.LinearLayoutManager(context)
        
        // Init Adapter
        labelAdapter = LabelAdapter(options) { selectedItem ->
            handleTrackingSelection(selectedItem)
            selectionDialog?.dismiss()
        }
        rv.adapter = labelAdapter
        
        // State Management
        isSelectionDialogOpen = true
        currentSelectionIndex = 0 // Reset selection to top
        labelAdapter?.setSelection(0)
        
        selectionDialog?.setOnDismissListener {
            isSelectionDialogOpen = false
            labelAdapter = null
            selectionDialog = null
        }
        
        selectionDialog?.setContentView(rv)
        selectionDialog?.show()
    }

    private fun takePhoto() {
        // ... (Giữ nguyên code chụp ảnh cũ) ...
        val imageCapture = imageCapture ?: return
        
        // Hiệu ứng chớp màn hình
        binding.root.foreground = android.graphics.drawable.ColorDrawable(Color.WHITE)
        binding.root.postDelayed({ binding.root.foreground = null }, 50)

        val name = "PoseLandmarker_" + System.currentTimeMillis() + ".jpg"
        val contentValues = android.content.ContentValues().apply {
            put(android.provider.MediaStore.MediaColumns.DISPLAY_NAME, name)
            put(android.provider.MediaStore.MediaColumns.MIME_TYPE, "image/jpeg")
            if(Build.VERSION.SDK_INT > Build.VERSION_CODES.P) {
                put(android.provider.MediaStore.Images.Media.RELATIVE_PATH, "Pictures/PoseLandmarker")
            }
        }

        val outputOptions = ImageCapture.OutputFileOptions.Builder(
            requireContext().contentResolver,
            android.provider.MediaStore.Images.Media.EXTERNAL_CONTENT_URI,
            contentValues
        ).build()

        imageCapture.takePicture(
            outputOptions,
            ContextCompat.getMainExecutor(requireContext()),
            object : ImageCapture.OnImageSavedCallback {
                override fun onError(exc: ImageCaptureException) {
                    Toast.makeText(context, "Lỗi chụp ảnh: ${exc.message}", Toast.LENGTH_SHORT).show()
                }
                override fun onImageSaved(output: ImageCapture.OutputFileResults) {
                    Toast.makeText(context, "Đã lưu ảnh!", Toast.LENGTH_SHORT).show()
                }
            }
        )
    }

    @SuppressLint("MissingPermission")
    private fun captureVideo() {
        val videoCapture = this.videoCapture ?: return

        // 1. Nếu đang quay -> STOP
        val curRecording = recording
        if (curRecording != null) {
            curRecording.stop()
            recording = null
            return
        }

        // 2. Nếu chưa quay -> START
        val name = "PoseVideo_" + System.currentTimeMillis() + ".mp4"
        val contentValues = android.content.ContentValues().apply {
            put(android.provider.MediaStore.Video.Media.DISPLAY_NAME, name)
            put(android.provider.MediaStore.Video.Media.MIME_TYPE, "video/mp4")
            if (Build.VERSION.SDK_INT > Build.VERSION_CODES.P) {
                put(android.provider.MediaStore.Video.Media.RELATIVE_PATH, "Movies/PoseLandmarker")
            }
        }

        val mediaStoreOutputOptions = MediaStoreOutputOptions
            .Builder(requireContext().contentResolver, android.provider.MediaStore.Video.Media.EXTERNAL_CONTENT_URI)
            .setContentValues(contentValues)
            .build()
        
        recording = videoCapture.output
            .prepareRecording(requireContext(), mediaStoreOutputOptions)
            .apply {
                if (PermissionChecker.checkSelfPermission(requireContext(), Manifest.permission.RECORD_AUDIO) == PermissionChecker.PERMISSION_GRANTED) {
                    withAudioEnabled()
                }
            }
            .start(ContextCompat.getMainExecutor(requireContext())) { recordEvent ->
                when(recordEvent) {
                    is VideoRecordEvent.Start -> {
                        binding.btnRecordVideo.setImageResource(android.R.drawable.ic_media_pause)
                        binding.tvHudLine1.text = "RECORDING..."
                        binding.tvHudLine1.setTextColor(Color.RED)
                    }
                    is VideoRecordEvent.Finalize -> {
                        if (!recordEvent.hasError()) {
                            Toast.makeText(context, "Video Saved!", Toast.LENGTH_SHORT).show()
                        } else {
                            recording?.close()
                            recording = null
                            Log.e("Video", "Error: ${recordEvent.error}")
                        }
                        binding.btnRecordVideo.setImageResource(android.R.drawable.ic_menu_camera) // Quay về icon cũ
                         binding.tvHudLine1.text = "Tracking Stopped" // Reset text
                         binding.tvHudLine1.setTextColor(Color.WHITE)
                    }
                }
            }
    }

    private fun checkPermissions() {
        val perms = mutableListOf(Manifest.permission.CAMERA, Manifest.permission.RECORD_AUDIO)
        if (Build.VERSION.SDK_INT >= 31) {
            perms.add(Manifest.permission.BLUETOOTH_SCAN)
            perms.add(Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            perms.add(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        requestPermLauncher.launch(perms.toTypedArray())
    }

    @SuppressLint("MissingPermission")
    private fun showScanDialogSafe() {
        foundDevices.clear()
        
        val recyclerView = androidx.recyclerview.widget.RecyclerView(requireContext())
        recyclerView.layoutManager = androidx.recyclerview.widget.LinearLayoutManager(requireContext())
        
        deviceAdapter = DeviceAdapter(foundDevices) { device ->
            scanDialog?.dismiss()
            bleScanner?.stopScan(scanCallback)
            bleManager.connect(device.address)
        }
        recyclerView.adapter = deviceAdapter
        
        scanDialog = BottomSheetDialog(requireContext())
        scanDialog?.setContentView(recyclerView)
        scanDialog?.setOnDismissListener {
            bleScanner?.stopScan(scanCallback)
        }
        scanDialog?.show()
        
        bleScanner = bluetoothAdapter?.bluetoothLeScanner
        bleScanner?.startScan(scanCallback)
        
        // Auto-stop scan after 10 seconds
        Handler(Looper.getMainLooper()).postDelayed({
            bleScanner?.stopScan(scanCallback)
        }, 10000)
    }
    
    private val scanCallback = object : android.bluetooth.le.ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: android.bluetooth.le.ScanResult) {
            val device = result.device
            if (device.name != null && !foundDevices.contains(device)) {
                foundDevices.add(device)
                deviceAdapter?.notifyDataSetChanged()
            }
        }
    }

    private fun setUpCamera() {
        val providerFuture = ProcessCameraProvider.getInstance(requireContext())
        providerFuture.addListener({
            if (_binding == null || !isAdded) return@addListener
            try {
                val provider = providerFuture.get()
                provider.unbindAll()

                val preview = Preview.Builder().setTargetResolution(Size(Constants.Camera.PREVIEW_WIDTH, Constants.Camera.PREVIEW_HEIGHT)).build()

                val analyzer = ImageAnalysis.Builder()
                    .setTargetResolution(Size(Constants.Camera.ANALYSIS_WIDTH, Constants.Camera.ANALYSIS_HEIGHT))
                    .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                    .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                    .build()

                imageCapture = ImageCapture.Builder()
                    .setCaptureMode(ImageCapture.CAPTURE_MODE_MINIMIZE_LATENCY)
                    .build()
                
                // Video Capture Setup
                val recorder = Recorder.Builder()
                    .setQualitySelector(QualitySelector.from(Quality.SD)) // SD for performance
                    .build()
                videoCapture = VideoCapture.withOutput(recorder)

                backgroundExecutor?.let { executor ->
                    var frameCounter = 0L // Biến đếm frame
                    analyzer.setAnalyzer(executor) { proxy ->
                        // Adaptive frame skip based on performance
                        if (frameCounter++ % adaptiveFrameSkip.toLong() != 0L) {
                            proxy.close()
                            return@setAnalyzer
                        }
                        try {
                            val bitmap = proxy.toBitmap()
                            val rotation = proxy.imageInfo.rotationDegrees
                            if (bitmap != null && ::yoloHelper.isInitialized) {
                                currentImgW = bitmap.width
                                currentImgH = bitmap.height
                                currentRotation = rotation
                                yoloHelper.detect(bitmap, rotation)
                            }
                        } catch (e: Exception) {
                            Log.e("Analyzer", "Error", e)
                        } finally {
                            proxy.close()
                        }
                    }
                }
                preview.setSurfaceProvider(binding.viewFinder.surfaceProvider)
                val camera = provider.bindToLifecycle(viewLifecycleOwner, CameraSelector.Builder().requireLensFacing(cameraFacing).build(), preview, analyzer, imageCapture, videoCapture)
                cameraControl = camera.cameraControl
                cameraInfo = camera.cameraInfo
                
                // Initialize AngleCalculator with actual preview dimensions
                binding.viewFinder.post {
                    val w = binding.viewFinder.width.toFloat()
                    val h = binding.viewFinder.height.toFloat()
                    
                    angleCalculator = AngleCalculator(w, h)
                    
                    // [Refactor] Init Tracking Manager (Robust Filtering)
                    trackingManager = TrackingManager(angleCalculator, w, h)
                }
            } catch (e: Exception) { Log.e("Camera", "Bind failed", e) }
        }, ContextCompat.getMainExecutor(requireContext()))
    }

    override fun onResults(results: List<YoloHelper.Recognition>, inferenceTime: Long) {
        activity?.runOnUiThread {
            if (_binding == null) return@runOnUiThread
            
            // Record metrics
            MetricsCollector.recordFrame()
            MetricsCollector.lastInferenceTime = inferenceTime

            // Step 1: Filter valid detections
            // CHỈ LẤY FACE HOẶC PERSON (Tuỳ nhu cầu)
            val validResults = results.filter { it.label == "face" || it.label == "person" }
            
            lastDetectedResults = validResults

            // ===== TRACKING LOGIC (DELEGATED TO TRACKING MANAGER) =====
            // Sử dụng bộ lọc cao cấp (Kalman, EMA...) trong TrackingManager thay vì logic rời rạc
            
            if (trackingManager != null && trackingManager!!.isTracking()) {
                
                val result = trackingManager!!.processDetections(
                    validResults,
                    { rect -> binding.overlay.getCenterPixel(rect) }, // Hàm chuyển đổi toạ độ
                    cameraFacing == CameraSelector.LENS_FACING_FRONT
                )
                
                // Update Local State for UI
                lastLockedLocation = result.lastLockLocation
                
                // Update HUD
                binding.tvHudLine1.text = result.status
                binding.tvHudLine1.setTextColor(result.statusColor)
                
                // [FIX] Update Metrics for Monitoring (Even if BLE disconnected)
                MetricsCollector.panAngle = result.panAngle
                MetricsCollector.tiltAngle = result.tiltAngle
                MetricsCollector.targetDistance = result.trackingError
                
                // Logic Gửi BLE & Overlay
                if (result.lastLockLocation != null) {
                    // === LOCKED ===
                    
                    // Gửi BLE (Góc đã được lọc kỹ 5 bước)
                    if (::bleManager.isInitialized && bleManager.isConnected && System.currentTimeMillis() - lastBleSendTime > Constants.Bluetooth.MIN_SEND_INTERVAL_MS) {
                        // [FIX] Đảo dấu Tilt khi dùng camera trước (mirror correction)
                        val bleTiltAngle = if (cameraFacing == CameraSelector.LENS_FACING_FRONT) -result.tiltAngle else result.tiltAngle
                        val data = "{{[ax]:${result.panAngle.toInt()};[ay]:${bleTiltAngle.toInt()}}}"
                        bleManager.sendReliable(data)
                        lastBleSendTime = System.currentTimeMillis()
                    }
                    
                    // Chỉ hiển thị box đang được lock
                    val displayList = validResults.filter { it.location == result.lastLockLocation }
                    binding.overlay.setResults(displayList, currentImgH, currentImgW, cameraFacing == CameraSelector.LENS_FACING_FRONT, result.trackingError)
                    
                } else {
                    // === SEARCHING / LOST ===
                    // Không gửi BLE (để Gimbal tự xử lý Coasting)
                    
                    // [FIX] Reset metrics when lost
                    MetricsCollector.panAngle = 0f
                    MetricsCollector.tiltAngle = 0f
                    
                    // Hiển thị tất cả candidate để người dùng biết
                    binding.overlay.setResults(validResults, currentImgH, currentImgW, cameraFacing == CameraSelector.LENS_FACING_FRONT, 0f)
                }

            } else {
                // === NO TRACKING ===
                val fps = if (inferenceTime > 0) 1000 / inferenceTime else 0
                binding.tvHudLine1.text = "FPS: $fps | ${validResults.size} detected"
                binding.tvHudLine1.setTextColor(Color.WHITE)
                
                // Reset metrics
                MetricsCollector.panAngle = 0f
                MetricsCollector.tiltAngle = 0f
                MetricsCollector.targetDistance = 0f
                
                // Hiển thị tất cả
                binding.overlay.setResults(validResults, currentImgH, currentImgW, cameraFacing == CameraSelector.LENS_FACING_FRONT, 0f)
            }
        }
    }

    private fun calculateDistance(boxA: RectF, boxB: RectF): Float {
        val dx = boxA.centerX() - boxB.centerX()
        val dy = boxA.centerY() - boxB.centerY()
        return sqrt((dx * dx + dy * dy).toDouble()).toFloat()
    }


    override fun onError(e: String) { Log.e("YOLO", e) }
    
    private fun loadMonitoringSettings() {
        val prefs = requireContext().getSharedPreferences("monitoring_prefs", android.content.Context.MODE_PRIVATE)
        val laptopIP = prefs.getString("laptop_ip", "") ?: ""
        val enabled = prefs.getBoolean("monitoring_enabled", false)
        if (enabled && laptopIP.isNotEmpty()) {
            metricsPusher = MetricsPusher(laptopIP)
            startMetricsPushing()
        }
    }
    
    private fun showMonitoringDialog() {
        val dialogView = layoutInflater.inflate(R.layout.dialog_monitoring_settings, null)
        val prefs = requireContext().getSharedPreferences("monitoring_prefs", android.content.Context.MODE_PRIVATE)
        val etLaptopIP = dialogView.findViewById<android.widget.EditText>(R.id.etLaptopIP)
        val switchMonitoring = dialogView.findViewById<android.widget.Switch>(R.id.switchMonitoring)
        val btnSave = dialogView.findViewById<android.widget.Button>(R.id.btnSave)
        val btnCancel = dialogView.findViewById<android.widget.Button>(R.id.btnCancel)
        etLaptopIP.setText(prefs.getString("laptop_ip", ""))
        switchMonitoring.isChecked = prefs.getBoolean("monitoring_enabled", false)
        val dialog = androidx.appcompat.app.AlertDialog.Builder(requireContext()).setView(dialogView).create()
        btnSave.setOnClickListener {
            val laptopIP = etLaptopIP.text.toString().trim()
            val enabled = switchMonitoring.isChecked
            prefs.edit().apply { putString("laptop_ip", laptopIP); putBoolean("monitoring_enabled", enabled); apply() }
            stopMetricsPushing()
            if (enabled && laptopIP.isNotEmpty()) {
                metricsPusher = MetricsPusher(laptopIP)
                startMetricsPushing()
                Toast.makeText(context, " $laptopIP:5000", Toast.LENGTH_SHORT).show()
            } else {
                metricsPusher = null
                Toast.makeText(context, "Monitoring off", Toast.LENGTH_SHORT).show()
            }
            dialog.dismiss()
        }
        btnCancel.setOnClickListener { dialog.dismiss() }
        dialog.show()
    }
    
    private fun startMetricsPushing() {
        if (isPushingMetrics) return
        isPushingMetrics = true
        val pushRunnable = object : Runnable { override fun run() { metricsPusher?.push(); if (isPushingMetrics) handler.postDelayed(this, 100) } }
        handler.post(pushRunnable)
    }
    
    private fun stopMetricsPushing() {
        isPushingMetrics = false
        handler.removeCallbacksAndMessages(null)
    }

    override fun onDestroyView() {
        super.onDestroyView()
        stopMetricsPushing()
        try { ProcessCameraProvider.getInstance(requireContext()).get().unbindAll() } catch (e: Exception) {}
        _binding = null
        backgroundExecutor?.shutdown()
        if (::yoloHelper.isInitialized) yoloHelper.close()
    }
}