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
import org.tensorflow.lite.support.image.ops.ResizeWithCropOrPadOp // ✅ Thêm Import
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.max
import kotlin.math.min

class YoloHelper(val context: Context, val listener: DetectorListener) {

    companion object {
        const val MODEL_PATH = Constants.AI.MODEL_PATH
        const val INPUT_SIZE = Constants.AI.INPUT_SIZE
        const val CONFIDENCE_THRESHOLD = Constants.AI.CONFIDENCE_THRESHOLD
        const val IOU_THRESHOLD = Constants.AI.IOU_THRESHOLD
        val LABELS = Constants.AI.LABELS
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

            // 1. [DISABLED] GPU (Int8 unfriendly)
            // if (compatList.isDelegateSupportedOnThisDevice) { ... }

            // 2. [DISABLED] NNAPI (Causes lag on some devices)
            // if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.P) { ... }

            // 3. FORCE CPU XNNPACK (Best Stability & FPS for Int8)
            Log.w("YoloHelper", "🚀 FORCE CPU MODE: Using XNNPACK with 4 Threads")
            options.setUseXNNPACK(true)
            options.setNumThreads(4)

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
            
            // [FIX] Tính toán kích thước Crop vuông để tránh méo hình
            // Lấy kích thước cạnh nhỏ nhất -> Crop vuông ở giữa (Center Crop)
            val cropSize = min(bitmap.width, bitmap.height)
            
            val imageProcessor = ImageProcessor.Builder()
                .add(ResizeWithCropOrPadOp(cropSize, cropSize)) // 1. Cắt vuông trước
                .add(Rot90Op(-numRotation))                     // 2. Xoay
                .add(ResizeOp(INPUT_SIZE, INPUT_SIZE, ResizeOp.ResizeMethod.BILINEAR)) // 3. Resize về 320x320
                .add(CastOp(DataType.FLOAT32))
                .add(NormalizeOp(0f, 255f))
                .build()

            if (inputImageBuffer == null) inputImageBuffer = TensorImage(DataType.FLOAT32)
            inputImageBuffer!!.load(bitmap)
            val processedImage = imageProcessor.process(inputImageBuffer)

            val outputBuffer = TensorBuffer.createFixedSize(outputShape, DataType.FLOAT32)
            interpreter!!.run(processedImage.buffer, outputBuffer.buffer.rewind())

            val bestBoxes = parseAndNMS(outputBuffer.floatArray, bitmap.width, bitmap.height)
            listener.onResults(bestBoxes, SystemClock.uptimeMillis() - startTime)
 
         } catch (e: Exception) {
             Log.e("YoloHelper", "Error: ${e.message}")
         } finally {
             isProcessing.set(false)
         }
     }
 
     private fun parseAndNMS(output: FloatArray, srcW: Int, srcH: Int): List<Recognition> {
         val detections = ArrayList<Recognition>()
         val numAnchors = if (isChannelLast) outputShape[1] else outputShape[2]
         val numChannels = if (isChannelLast) outputShape[2] else outputShape[1]
         
         // Tính toán tham số Crop để map ngược toạ độ
         val cropSize = min(srcW, srcH).toFloat()
         val padX = (srcW - cropSize) / 2f
         val padY = (srcH - cropSize) / 2f
 
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
 
                 // 1. Normalized (0..1) relative to CROP (320x320)
                 val scale = if (cx > 1.0f || w > 1.0f) INPUT_SIZE.toFloat() else 1.0f
                 
                 val cxCrop = cx / scale
                 val cyCrop = cy / scale
                 val wCrop = w / scale
                 val hCrop = h / scale
                 
                 // 2. Convert to Full Image Normalized (0..1)
                 // Công thức: (PosInCrop * CropSize + Padding) / FullSize
                 val cxFull = (cxCrop * cropSize + padX) / srcW
                 val cyFull = (cyCrop * cropSize + padY) / srcH
                 val wFull = (wCrop * cropSize) / srcW
                 val hFull = (hCrop * cropSize) / srcH
 
                 val left = cxFull - wFull/2
                 val top = cyFull - hFull/2
                 val right = cxFull + wFull/2
                 val bottom = cyFull + hFull/2
 
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