package com.google.mediapipe.examples.poselandmarker

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.PointF
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View
import kotlin.math.max

class OverlayView(context: Context?, attrs: AttributeSet?) : View(context, attrs) {
    private var results: List<YoloHelper.Recognition> = listOf()
    private val boxPaint = Paint()
    private val textPaint = Paint()
    private val centerPaint = Paint()
    private val linePaint = Paint()
    private val confidencePaint = Paint()
    private var isFrontCamera = false
    
    // Tracking quality for dynamic coloring
    private var trackingError: Float = 0f
    
    // Person instances for tap-to-select
    private var personInstances: List<PersonInstance> = listOf()
    private var selectedPerson: PersonInstance? = null

    // Kích thước ảnh nguồn (từ Camera/AI)
    private var sourceWidth = 1
    private var sourceHeight = 1

    init {
        boxPaint.color = Color.GREEN
        boxPaint.strokeWidth = 5f
        boxPaint.style = Paint.Style.STROKE

        textPaint.color = Color.WHITE
        textPaint.style = Paint.Style.FILL
        textPaint.textSize = 40f
        textPaint.isFakeBoldText = true
        textPaint.setShadowLayer(3f, 0f, 0f, Color.BLACK)

        centerPaint.color = Color.CYAN
        centerPaint.strokeWidth = 4f
        centerPaint.style = Paint.Style.STROKE

        linePaint.color = Color.YELLOW
        linePaint.strokeWidth = 3f
        
        confidencePaint.color = Color.GREEN
        confidencePaint.style = Paint.Style.FILL
        confidencePaint.alpha = 180
    }

    fun setResults(detectionResults: List<YoloHelper.Recognition>, imgHeight: Int, imgWidth: Int, isFront: Boolean, error: Float = 0f) {
        results = detectionResults
        // LƯU Ý: Với code cũ của bạn, imgHeight/Width truyền vào có thể bị đảo.
        // Ta cần đảm bảo sourceWidth/Height phản ánh đúng chiều của ảnh khi hiển thị.
        sourceWidth = imgWidth
        sourceHeight = imgHeight
        isFrontCamera = isFront
        trackingError = error
        postInvalidate()
    }
    
    /**
     * Set PersonInstances for rendering (new tap-to-select mode).
     * 
     * @param instances List of PersonInstances to render
     * @param selected Currently selected PersonInstance (null if none)
     * @param imgHeight Image height
     * @param imgWidth Image width
     * @param isFront Whether using front camera
     * @param error Tracking error for quality indicator
     */
    fun setPersonInstances(
        instances: List<PersonInstance>,
        selected: PersonInstance?,
        imgHeight: Int,
        imgWidth: Int,
        isFront: Boolean,
        error: Float = 0f
    ) {
        personInstances = instances
        selectedPerson = selected
        sourceWidth = imgWidth
        sourceHeight = imgHeight
        isFrontCamera = isFront
        trackingError = error
        postInvalidate()
    }

    override fun draw(canvas: Canvas) {
        super.draw(canvas)

        val viewW = width.toFloat()
        val viewH = height.toFloat()
        val cx = viewW / 2f
        val cy = viewH / 2f

        // Vẽ tâm
        canvas.drawLine(cx - 40f, cy, cx + 40f, cy, centerPaint)
        canvas.drawLine(cx, cy - 40f, cx, cy + 40f, centerPaint)
        canvas.drawCircle(cx, cy, 15f, centerPaint)

        // Render PersonInstances if available (new mode)
        if (personInstances.isNotEmpty()) {
            drawPersonInstances(canvas, viewW, viewH, cx, cy)
            return
        }
        
        // Fallback: render old-style results
        if (results.isEmpty()) return

        // === [MATCH PreviewView Scale Type] ===
        // PreviewView uses fillCenter (CENTER_CROP) by default
        // So OverlayView must use the same scale calculation
        val scaleInfo = calculateScale(viewW, viewH)
        val scale = scaleInfo.first
        val offsetX = scaleInfo.second
        val offsetY = scaleInfo.third

        for (result in results) {
            val box = result.location // Box dạng 0..1

            // Chuyển từ 0..1 sang Pixel màn hình
            var left = (box.left * sourceWidth * scale) + offsetX
            var top = (box.top * sourceHeight * scale) + offsetY
            var right = (box.right * sourceWidth * scale) + offsetX
            var bottom = (box.bottom * sourceHeight * scale) + offsetY

            // Xử lý Camera trước (Mirror)
            if (isFrontCamera) {
                val newLeft = viewW - right
                val newRight = viewW - left
                left = newLeft
                right = newRight
            }
            
            // Dynamic color based on tracking quality
            boxPaint.color = when {
                trackingError < 20 -> Color.GREEN   // Excellent tracking
                trackingError < 50 -> Color.YELLOW  // Good tracking
                trackingError < 100 -> Color.rgb(255, 165, 0) // Orange - Fair tracking
                else -> Color.RED                   // Poor tracking
            }

            // Vẽ box
            canvas.drawRect(left, top, right, bottom, boxPaint)

            val label = "${result.label} ${(result.confidence * 100).toInt()}%"
            canvas.drawText(label, max(10f, left), max(50f, top - 10), textPaint)
            
            // Draw confidence bar below bounding box
            val barWidth = (right - left) * result.confidence
            canvas.drawRect(left, bottom + 5, left + barWidth, bottom + 15, confidencePaint)

            val objCx = (left + right) / 2
            val objCy = (top + bottom) / 2
            canvas.drawLine(cx, cy, objCx, objCy, linePaint)
        }
    }
    
    /**
     * Draw PersonInstances with visual differentiation.
     */
    private fun drawPersonInstances(canvas: Canvas, viewW: Float, viewH: Float, cx: Float, cy: Float) {
        val scaleInfo = calculateScale(viewW, viewH)
        val scale = scaleInfo.first
        val offsetX = scaleInfo.second
        val offsetY = scaleInfo.third
        
        for (instance in personInstances) {
            val isSelected = instance == selectedPerson
            
            // Get primary box (face if available, otherwise person)
            val box = instance.getPrimaryBox()
            
            // Convert from 0..1 to pixel coordinates
            var left = (box.left * sourceWidth * scale) + offsetX
            var top = (box.top * sourceHeight * scale) + offsetY
            var right = (box.right * sourceWidth * scale) + offsetX
            var bottom = (box.bottom * sourceHeight * scale) + offsetY
            
            // Mirror for front camera
            if (isFrontCamera) {
                val newLeft = viewW - right
                val newRight = viewW - left
                left = newLeft
                right = newRight
            }
            
            // Visual differentiation
            if (isSelected) {
                // Selected person: green, thick, dynamic color based on tracking quality
                boxPaint.color = when {
                    trackingError < 20 -> Color.GREEN
                    trackingError < 50 -> Color.YELLOW
                    trackingError < 100 -> Color.rgb(255, 165, 0)
                    else -> Color.RED
                }
                boxPaint.strokeWidth = 8f
            } else {
                // Other persons: gray, thin
                boxPaint.color = Color.GRAY
                boxPaint.strokeWidth = 3f
            }
            
            // Draw box
            canvas.drawRect(left, top, right, bottom, boxPaint)
            
            // Label
            val label = if (isSelected) {
                "TRACKING (${(instance.confidence * 100).toInt()}%)"
            } else {
                "Person ${instance.id} (${(instance.confidence * 100).toInt()}%)"
            }
            canvas.drawText(label, max(10f, left), max(50f, top - 10), textPaint)
            
            // Confidence bar
            val barWidth = (right - left) * instance.confidence
            confidencePaint.color = if (isSelected) Color.GREEN else Color.GRAY
            canvas.drawRect(left, bottom + 5, left + barWidth, bottom + 15, confidencePaint)
            
            // Draw line to center for selected person
            if (isSelected) {
                val objCx = (left + right) / 2
                val objCy = (top + bottom) / 2
                canvas.drawLine(cx, cy, objCx, objCy, linePaint)
            }
        }
        
        // Reset stroke width
        boxPaint.strokeWidth = 5f
    }
    
    /**
     * Calculate scale and offsets for CENTER_CROP mode.
     * Extracted to avoid code duplication between draw() and getCenterPixel().
     */
    private fun calculateScale(viewW: Float, viewH: Float): Triple<Float, Float, Float> {
        val scaleX = viewW / sourceWidth
        val scaleY = viewH / sourceHeight
        val scale = max(scaleX, scaleY) // CENTER_CROP
        
        val scaledWidth = sourceWidth * scale
        val scaledHeight = sourceHeight * scale
        val offsetX = (viewW - scaledWidth) / 2f
        val offsetY = (viewH - scaledHeight) / 2f
        
        return Triple(scale, offsetX, offsetY)
    }

    // Hàm lấy toạ độ tâm (đã fix scale giống hàm draw)
    fun getCenterPixel(box: RectF): PointF {
        val viewW = width.toFloat()
        val viewH = height.toFloat()

        // Use same scale calculation as draw()
        val scaleInfo = calculateScale(viewW, viewH)
        val scale = scaleInfo.first
        val offsetX = scaleInfo.second
        val offsetY = scaleInfo.third

        val cxRaw = box.centerX()
        val cyRaw = box.centerY()

        // Tính toạ độ pixel trước khi mirror
        var screenX = (cxRaw * sourceWidth * scale) + offsetX
        val screenY = (cyRaw * sourceHeight * scale) + offsetY

        if (isFrontCamera) {
            screenX = viewW - screenX
        }

        return PointF(screenX, screenY)
    }
}