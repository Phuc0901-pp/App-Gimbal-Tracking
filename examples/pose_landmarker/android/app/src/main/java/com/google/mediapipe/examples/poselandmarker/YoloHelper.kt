package com.google.mediapipe.examples.poselandmarker

import android.content.Context
import android.graphics.Bitmap
import android.graphics.RectF
import android.os.SystemClock
import android.util.Log
import org.tensorflow.lite.DataType
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.gpu.CompatibilityList
import org.tensorflow.lite.gpu.GpuDelegate
import org.tensorflow.lite.nnapi.NnApiDelegate // ✅ Cần thêm dòng này
import org.tensorflow.lite.support.common.FileUtil
import org.tensorflow.lite.support.common.ops.CastOp
import org.tensorflow.lite.support.common.ops.NormalizeOp
import org.tensorflow.lite.support.image.ImageProcessor
import org.tensorflow.lite.support.image.TensorImage
import org.tensorflow.lite.support.image.ops.ResizeOp
import org.tensorflow.lite.support.image.ops.Rot90Op
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.max
import kotlin.math.min

class YoloHelper(val context: Context, val listener: DetectorListener) {

    companion object {
        const val MODEL_PATH = "best_int8.tflite" // Đảm bảo file này là bản imgsz=320
        const val INPUT_SIZE = 320 // ✅ ĐÃ GIẢM XUỐNG 320 ĐỂ TĂNG TỐC
        const val CONFIDENCE_THRESHOLD = 0.55f
        const val IOU_THRESHOLD = 0.5f
        val LABELS = listOf("face", "person")
    }

    data class Recognition(val id: String, val label: String, val confidence: Float, val location: RectF)

    private var interpreter: Interpreter? = null
    private var gpuDelegate: GpuDelegate? = null
    private var nnApiDelegate: NnApiDelegate? = null // ✅ Thêm biến NNAPI
    private val isProcessing = AtomicBoolean(false)
    private var inputImageBuffer: TensorImage? = null
    private var isChannelLast = true
    private var outputShape = intArrayOf(0, 0, 0)

    init {
        setupInterpreter()
    }

    private fun setupInterpreter() {
        try {
            val options = Interpreter.Options()
            val compatList = CompatibilityList()

            // 1. Ưu tiên thử GPU trước
            if (compatList.isDelegateSupportedOnThisDevice) {
                Log.d("YoloHelper", "✅ GPU Supported. Loading...")
                val delegateOptions = GpuDelegate.Options().apply {
                    setInferencePreference(GpuDelegate.Options.INFERENCE_PREFERENCE_SUSTAINED_SPEED)
                    setPrecisionLossAllowed(true)
                }
                gpuDelegate = GpuDelegate(delegateOptions)
                options.addDelegate(gpuDelegate)
            }
            // 2. Nếu không có GPU, thử dùng NNAPI (Tăng tốc phần cứng Android)
            else if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.P) {
                Log.d("YoloHelper", "⚠️ GPU Not Supported. Trying NNAPI...")
                try {
                    nnApiDelegate = NnApiDelegate()
                    options.addDelegate(nnApiDelegate)
                    Log.d("YoloHelper", "✅ NNAPI Delegate Loaded!")
                } catch (e: Exception) {
                    Log.e("YoloHelper", "NNAPI Failed, falling back to CPU", e)
                    options.setUseXNNPACK(true)
                    options.setNumThreads(4)
                }
            }
            // 3. Đường cùng: Dùng CPU thuần (XNNPACK)
            else {
                Log.w("YoloHelper", "⚠️ Hardware accel failed. Using CPU XNNPACK.")
                options.setUseXNNPACK(true)
                options.setNumThreads(4)
            }

            val modelFile = FileUtil.loadMappedFile(context, MODEL_PATH)
            interpreter = Interpreter(modelFile, options)

            val outputTensor = interpreter!!.getOutputTensor(0)
            outputShape = outputTensor.shape()
            isChannelLast = outputShape[2] < outputShape[1]

            Log.i("YoloHelper", "Model Loaded: ${outputShape.contentToString()} ChannelLast=$isChannelLast")

        } catch (e: Exception) {
            listener.onError("Init Error: ${e.message}")
        }
    }

    fun detect(bitmap: Bitmap, rotation: Int) {
        if (interpreter == null) return

        // Nếu đang xử lý frame cũ chưa xong thì bỏ qua frame mới (Drop Frame)
        // Giúp UI không bị lag
        if (!isProcessing.compareAndSet(false, true)) {
            return
        }

        try {
            val startTime = SystemClock.uptimeMillis()
            val numRotation = rotation / 90
            val imageProcessor = ImageProcessor.Builder()
                .add(Rot90Op(-numRotation))
                .add(ResizeOp(INPUT_SIZE, INPUT_SIZE, ResizeOp.ResizeMethod.BILINEAR))
                .add(CastOp(DataType.FLOAT32))
                .add(NormalizeOp(0f, 255f))
                .build()

            if (inputImageBuffer == null) inputImageBuffer = TensorImage(DataType.FLOAT32)
            inputImageBuffer!!.load(bitmap)
            val processedImage = imageProcessor.process(inputImageBuffer)

            val outputBuffer = TensorBuffer.createFixedSize(outputShape, DataType.FLOAT32)
            interpreter!!.run(processedImage.buffer, outputBuffer.buffer.rewind())

            val bestBoxes = parseAndNMS(outputBuffer.floatArray)
            listener.onResults(bestBoxes, SystemClock.uptimeMillis() - startTime)

        } catch (e: Exception) {
            Log.e("YoloHelper", "Error: ${e.message}")
        } finally {
            isProcessing.set(false)
        }
    }

    private fun parseAndNMS(output: FloatArray): List<Recognition> {
        val detections = ArrayList<Recognition>()
        val numAnchors = if (isChannelLast) outputShape[1] else outputShape[2]
        val numChannels = if (isChannelLast) outputShape[2] else outputShape[1]

        for (i in 0 until numAnchors) {
            var maxScore = 0f
            var classIndex = -1

            for (c in 0 until (numChannels - 4)) {
                val score = if (isChannelLast) output[i * numChannels + (4 + c)]
                else output[(4 + c) * numAnchors + i]
                if (score > maxScore) { maxScore = score; classIndex = c }
            }

            if (maxScore > CONFIDENCE_THRESHOLD) {
                val cx: Float; val cy: Float; val w: Float; val h: Float
                if (isChannelLast) {
                    val offset = i * numChannels
                    cx = output[offset + 0]; cy = output[offset + 1]
                    w  = output[offset + 2]; h  = output[offset + 3]
                } else {
                    cx = output[0 * numAnchors + i]; cy = output[1 * numAnchors + i]
                    w  = output[2 * numAnchors + i]; h  = output[3 * numAnchors + i]
                }

                // Auto-detect scale
                val scale = if (cx > 1.0f || w > 1.0f) INPUT_SIZE.toFloat() else 1.0f

                val left = (cx - w/2) / scale
                val top = (cy - h/2) / scale
                val right = (cx + w/2) / scale
                val bottom = (cy + h/2) / scale

                val rect = RectF(max(0f, left), max(0f, top), min(1f, right), min(1f, bottom))
                detections.add(Recognition(i.toString(), LABELS.getOrElse(classIndex){"?"}, maxScore, rect))
            }
        }
        return nms(detections)
    }

    private fun nms(detections: ArrayList<Recognition>): List<Recognition> {
        val nmsList = ArrayList<Recognition>()
        detections.sortByDescending { it.confidence }
        while (detections.isNotEmpty()) {
            val current = detections.removeAt(0)
            nmsList.add(current)
            val iterator = detections.iterator()
            while (iterator.hasNext()) {
                if (calculateIoU(current.location, iterator.next().location) > IOU_THRESHOLD) iterator.remove()
            }
        }
        return nmsList
    }

    private fun calculateIoU(a: RectF, b: RectF): Float {
        val interArea = max(0f, min(a.right,b.right)-max(a.left,b.left)) * max(0f, min(a.bottom,b.bottom)-max(a.top,b.top))
        val unionArea = (a.width()*a.height()) + (b.width()*b.height()) - interArea
        return interArea / unionArea
    }

    fun close() {
        interpreter?.close()
        gpuDelegate?.close()
        nnApiDelegate?.close() // Đóng NNAPI
    }

    interface DetectorListener {
        fun onError(error: String)
        fun onResults(results: List<Recognition>, inferenceTime: Long)
    }
}