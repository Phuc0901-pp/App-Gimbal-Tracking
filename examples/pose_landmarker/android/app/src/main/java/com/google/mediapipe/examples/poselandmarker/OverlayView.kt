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
    private var isFrontCamera = false

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
    }

    fun setResults(detectionResults: List<YoloHelper.Recognition>, imgHeight: Int, imgWidth: Int, isFront: Boolean) {
        results = detectionResults
        // LƯU Ý: Với code cũ của bạn, imgHeight/Width truyền vào có thể bị đảo.
        // Ta cần đảm bảo sourceWidth/Height phản ánh đúng chiều của ảnh khi hiển thị.
        sourceWidth = imgWidth
        sourceHeight = imgHeight
        isFrontCamera = isFront
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

        if (results.isEmpty()) return

        // === [FIX LỖI SCALE TẠI ĐÂY] ===
        // Tính tỷ lệ scale sao cho ảnh phủ kín màn hình (Center Crop)
        // Đây là cách hiển thị mặc định của PreviewView
        val scale = max(viewW / sourceWidth, viewH / sourceHeight)

        // Tính toán phần thừa để căn giữa
        val scaledWidth = sourceWidth * scale
        val scaledHeight = sourceHeight * scale
        val offsetX = (viewW - scaledWidth) / 2f
        val offsetY = (viewH - scaledHeight) / 2f

        for (result in results) {
            val box = result.location // Box dạng 0..1

            // Chuyển từ 0..1 sang Pixel màn hình
            // Công thức: (Toạ độ 0..1 * Kích thước gốc * Scale) + Offset căn giữa
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

            // Vẽ
            canvas.drawRect(left, top, right, bottom, boxPaint)

            val label = "${result.label} ${(result.confidence * 100).toInt()}%"
            canvas.drawText(label, max(10f, left), max(50f, top - 10), textPaint)

            val objCx = (left + right) / 2
            val objCy = (top + bottom) / 2
            canvas.drawLine(cx, cy, objCx, objCy, linePaint)
        }
    }

    // Hàm lấy toạ độ tâm (đã fix scale giống hàm draw)
    fun getCenterPixel(box: RectF): PointF {
        val viewW = width.toFloat()
        val viewH = height.toFloat()

        val scale = max(viewW / sourceWidth, viewH / sourceHeight)

        val scaledWidth = sourceWidth * scale
        val scaledHeight = sourceHeight * scale
        val offsetX = (viewW - scaledWidth) / 2f
        val offsetY = (viewH - scaledHeight) / 2f

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