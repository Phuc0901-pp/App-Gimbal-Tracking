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
import android.graphics.RectF
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.util.Size
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.core.*
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
    private var backgroundExecutor: ExecutorService? = null

    // Bluetooth
    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bleScanner: android.bluetooth.le.BluetoothLeScanner? = null
    private var scanDialog: BottomSheetDialog? = null
    private val foundDevices = ArrayList<BluetoothDevice>()
    private var deviceAdapter: DeviceAdapter? = null
    private lateinit var bleClient: BleUartClient
    private var lastBleSendTime = 0L

    // Tracking & Logic
    private var trackingLabel: String? = null
    private var lastDetectedResults: List<YoloHelper.Recognition> = listOf()
    private var currentImgW = 0
    private var currentImgH = 0
    private var currentRotation = 0
    private var lastLockedLocation: RectF? = null

    // Bộ làm mượt (Smoothing)
    // alpha = 0.4: Mượt mà, ít rung. Nếu thấy trễ quá thì tăng lên 0.6
    private val smootherX = CoordinateSmoother(alpha = 0.4f)
    private val smootherY = CoordinateSmoother(alpha = 0.4f)

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
        bleClient = BleUartClient(requireContext(), bluetoothAdapter, object : BleUartClient.Listener {
            override fun onConnected(d: BluetoothDevice) {
                activity?.runOnUiThread { if(_binding!=null){binding.tvHudWarn.text="BT: ${d.name}"; binding.tvHudWarn.setTextColor(Color.GREEN)} }
            }
            override fun onDisconnected() {
                activity?.runOnUiThread { if(_binding!=null){binding.tvHudWarn.text="BT Disconnected"; binding.tvHudWarn.setTextColor(Color.RED)} }
            }
            override fun onConnectFailed(d: BluetoothDevice) {}
            override fun onDataReceived(data: String) {}
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

        binding.btnTargets.setOnClickListener { showTargetSelectionDialog() }
        binding.btnCancelTrack.setOnClickListener { stopTracking() }
    }

    private fun stopTracking() {
        trackingLabel = null
        lastLockedLocation = null
        smootherX.reset()
        smootherY.reset()
        binding.btnCancelTrack.visibility = View.GONE
        binding.tvHudLine1.text = "Tracking Stopped"
        binding.tvHudLine1.setTextColor(Color.WHITE)
    }

    private fun showTargetSelectionDialog() {
        val availableLabels = lastDetectedResults.map { it.label }.distinct()
        if (availableLabels.isEmpty()) {
            Toast.makeText(context, "Chưa thấy vật thể nào!", Toast.LENGTH_SHORT).show()
            return
        }
        val dialog = BottomSheetDialog(requireContext())
        val rv = androidx.recyclerview.widget.RecyclerView(requireContext())
        rv.layoutManager = androidx.recyclerview.widget.LinearLayoutManager(context)
        rv.adapter = LabelAdapter(availableLabels) { selectedLabel ->
            trackingLabel = selectedLabel
            lastLockedLocation = null
            smootherX.reset()
            smootherY.reset()
            binding.btnCancelTrack.visibility = View.VISIBLE
            binding.btnCancelTrack.text = "STOP: ${selectedLabel.uppercase()}"
            dialog.dismiss()
        }
        dialog.setContentView(rv)
        dialog.show()
    }

    private fun checkPermissions() {
        val perms = mutableListOf(Manifest.permission.CAMERA)
        if (Build.VERSION.SDK_INT >= 31) {
            perms.add(Manifest.permission.BLUETOOTH_SCAN)
            perms.add(Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            perms.add(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        requestPermLauncher.launch(perms.toTypedArray())
    }

    private fun setUpCamera() {
        val providerFuture = ProcessCameraProvider.getInstance(requireContext())
        providerFuture.addListener({
            if (_binding == null || !isAdded) return@addListener
            try {
                val provider = providerFuture.get()
                provider.unbindAll()

                val preview = Preview.Builder().setTargetResolution(Size(640, 480)).build()

                val analyzer = ImageAnalysis.Builder()
                    .setTargetResolution(Size(640, 480))
                    // [QUAN TRỌNG] Chỉ giữ frame mới nhất
                    .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                    .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                    .build()

                backgroundExecutor?.let { executor ->
                    analyzer.setAnalyzer(executor) { proxy ->
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
                            proxy.close() // BẮT BUỘC ĐÓNG PROXY
                        }
                    }
                }
                preview.setSurfaceProvider(binding.viewFinder.surfaceProvider)
                provider.bindToLifecycle(viewLifecycleOwner, CameraSelector.Builder().requireLensFacing(cameraFacing).build(), preview, analyzer)
            } catch (e: Exception) { Log.e("Camera", "Bind failed", e) }
        }, ContextCompat.getMainExecutor(requireContext()))
    }

    // ==================================================================================
    // LOGIC XỬ LÝ CHÍNH
    // ==================================================================================
    override fun onResults(results: List<YoloHelper.Recognition>, inferenceTime: Long) {
        activity?.runOnUiThread {
            if (_binding == null) return@runOnUiThread

            val validDetections = results.filter { it.label == "face" || it.label == "person" }
            val finalResults = ArrayList<YoloHelper.Recognition>()

            if (trackingLabel != null) {
                // --- CHẾ ĐỘ TRACKING ---
                val candidates = validDetections.filter { it.label == trackingLabel }

                if (candidates.isNotEmpty()) {
                    val selectedTarget: YoloHelper.Recognition

                    if (lastLockedLocation != null) {
                        // Nhớ mặt: Chọn người gần vị trí cũ nhất
                        selectedTarget = candidates.minByOrNull {
                            calculateDistance(it.location, lastLockedLocation!!)
                        } ?: candidates[0]
                    } else {
                        // Mới track: Chọn người confidence cao nhất
                        selectedTarget = candidates.maxByOrNull { it.confidence } ?: candidates[0]
                    }

                    lastLockedLocation = selectedTarget.location
                    finalResults.add(selectedTarget)
                } else {
                    lastLockedLocation = null
                }
            } else {
                // --- CHẾ ĐỘ THƯỜNG ---
                lastLockedLocation = null
                val bestFace = validDetections.filter { it.label == "face" }.maxByOrNull { it.confidence }
                val bestPerson = validDetections.filter { it.label == "person" }.maxByOrNull { it.confidence }
                if (bestFace != null) finalResults.add(bestFace)
                if (bestPerson != null) finalResults.add(bestPerson)
            }

            lastDetectedResults = finalResults
            val finalW = if (currentRotation == 90 || currentRotation == 270) currentImgH else currentImgW
            val finalH = if (currentRotation == 90 || currentRotation == 270) currentImgW else currentImgH

            // Vẽ Overlay
            binding.overlay.setResults(finalResults, finalH, finalW, cameraFacing == CameraSelector.LENS_FACING_FRONT)

            // Xử lý gửi Bluetooth
            if (trackingLabel != null) {
                if (finalResults.isNotEmpty()) {
                    val target = finalResults[0]

                    val screenCenterX = binding.overlay.width / 2f
                    val screenCenterY = binding.overlay.height / 2f
                    val objCenterRaw = binding.overlay.getCenterPixel(target.location)

                    // 1. LÀM MƯỢT TỌA ĐỘ
                    val smoothX = smootherX.update(objCenterRaw.x, isX = true)
                    val smoothY = smootherY.update(objCenterRaw.y, isX = false)

                    // 2. TÍNH OFFSET
                    val dX = smoothX - screenCenterX
                    val dY = smoothY - screenCenterY

                    // 3. TÍNH GÓC (ANGLE)
                    val focalLength = binding.overlay.width.toFloat() * 0.866f
                    val angleX = Math.toDegrees(atan(dX / focalLength).toDouble()).roundToInt()
                    val angleY = Math.toDegrees(atan(dY / focalLength).toDouble()).roundToInt()

                    binding.tvHudLine1.text = "LOCK: ${target.label.uppercase()} | Ang: $angleX°, $angleY°"
                    binding.tvHudLine1.setTextColor(Color.GREEN)

                    // 4. GỬI BLUETOOTH (30ms ~ 33Hz)
                    if (::bleClient.isInitialized && System.currentTimeMillis() - lastBleSendTime > 30) {
                        val data = "{{[x]:${dX.toInt()};[y]:${dY.toInt()};[ax]:$angleX;[ay]:$angleY}}"
                        bleClient.send(data)
                        lastBleSendTime = System.currentTimeMillis()
                    }

                } else {
                    binding.tvHudLine1.text = "LOST TARGET..."
                    binding.tvHudLine1.setTextColor(Color.RED)
                    // Gửi LOST thưa hơn (500ms) để không spam
                    if (::bleClient.isInitialized && System.currentTimeMillis() - lastBleSendTime > 500) {
                        bleClient.send("{{[status]:LOST}}")
                        lastBleSendTime = System.currentTimeMillis()
                    }
                }
            } else {
                val fps = if (inferenceTime > 0) 1000 / inferenceTime else 0
                val names = finalResults.joinToString { it.label }
                binding.tvHudLine1.text = "FPS: $fps | Seeing: $names"
                binding.tvHudLine1.setTextColor(Color.WHITE)
            }
        }
    }

    // Hàm tính khoảng cách an toàn (Sửa lỗi pow/Float)
    private fun calculateDistance(boxA: RectF, boxB: RectF): Float {
        val dx = boxA.centerX() - boxB.centerX()
        val dy = boxA.centerY() - boxB.centerY()
        return sqrt((dx * dx + dy * dy).toDouble()).toFloat()
    }

    // Class làm mượt (EMA)
    class CoordinateSmoother(private val alpha: Float = 0.5f) {
        private var lastX: Float? = null
        private var lastY: Float? = null
        fun update(newValue: Float, isX: Boolean): Float {
            val lastVal = if (isX) lastX else lastY
            if (lastVal == null) {
                if (isX) lastX = newValue else lastY = newValue
                return newValue
            }
            val smoothed = (alpha * newValue) + ((1 - alpha) * lastVal)
            if (isX) lastX = smoothed else lastY = smoothed
            return smoothed
        }
        fun reset() { lastX = null; lastY = null }
    }

    // Bluetooth UI
    @SuppressLint("MissingPermission")
    private fun showScanDialogSafe() {
        try {
            scanDialog = BottomSheetDialog(requireContext())
            val rv = androidx.recyclerview.widget.RecyclerView(requireContext())
            rv.layoutManager = androidx.recyclerview.widget.LinearLayoutManager(context)
            foundDevices.clear()
            deviceAdapter = DeviceAdapter(foundDevices) { bleClient.connect(it.address); scanDialog?.dismiss() }
            rv.adapter = deviceAdapter
            scanDialog?.setContentView(rv)
            scanDialog?.show()
            bleScanner = bluetoothAdapter?.bluetoothLeScanner
            bleScanner?.startScan(object : ScanCallback() {
                override fun onScanResult(t: Int, r: ScanResult) {
                    if (r.device.name != null && !foundDevices.any { it.address == r.device.address }) {
                        foundDevices.add(r.device)
                        deviceAdapter?.notifyDataSetChanged()
                    }
                }
            })
        } catch (e: Exception) { Toast.makeText(context, "Lỗi BT", Toast.LENGTH_SHORT).show() }
    }

    override fun onError(e: String) { Log.e("YOLO", e) }

    override fun onDestroyView() {
        super.onDestroyView()
        try { ProcessCameraProvider.getInstance(requireContext()).get().unbindAll() } catch (e: Exception) {}
        _binding = null
        backgroundExecutor?.shutdown()
        if (::yoloHelper.isInitialized) yoloHelper.close()
    }
}