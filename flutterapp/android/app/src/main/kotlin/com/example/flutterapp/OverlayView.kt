package com.example.flutterapp

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import android.util.Log
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarkerResult
import kotlin.math.max
import kotlin.math.min

class OverlayView(context: Context?, attrs: AttributeSet?) : View(context, attrs) {

    private var results: HandLandmarkerResult? = null
    private var linePaint = Paint()
    private var pointPaint = Paint()

    private var scaleFactor: Float = 1f
    private var imageWidth: Int = 1
    private var imageHeight: Int = 1

    init {
        initPaints()
        // FORCE DRAWING: Ensures onDraw is called
        setWillNotDraw(false) 
    }

    fun clear() {
        results = null
        linePaint.reset()
        pointPaint.reset()
        invalidate()
        initPaints()
    }

    private fun initPaints() {
        // Make it huge and red so you can't miss it
        linePaint.color = Color.RED
        linePaint.strokeWidth = 10F
        linePaint.style = Paint.Style.STROKE

        pointPaint.color = Color.GREEN
        pointPaint.strokeWidth = 20F // Big dots
        pointPaint.style = Paint.Style.FILL
    }

    override fun draw(canvas: Canvas) {
        super.draw(canvas)
        
        if (results == null) return

        // 1. Calculate the Offset to center the drawing
        val scaledWidth = imageWidth * scaleFactor
        val scaledHeight = imageHeight * scaleFactor
        val offsetX = (width - scaledWidth) / 2
        val offsetY = (height - scaledHeight) / 2
        
        results?.let { handLandmarkerResult ->
            for (landmark in handLandmarkerResult.landmarks()) {
                for (normalizedLandmark in landmark) {
                    // 2. Apply Offset to X and Y
                    val x = normalizedLandmark.x() * scaledWidth + offsetX
                    val y = normalizedLandmark.y() * scaledHeight + offsetY
                    
                    canvas.drawCircle(x, y, 12f, pointPaint)
                }

                HandLandmarker.HAND_CONNECTIONS.forEach {
                    val start = landmark.get(it!!.start())
                    val end = landmark.get(it.end())

                    // 3. Apply Offset to Lines as well
                    val startX = start.x() * scaledWidth + offsetX
                    val startY = start.y() * scaledHeight + offsetY
                    val endX = end.x() * scaledWidth + offsetX
                    val endY = end.y() * scaledHeight + offsetY
                    
                    canvas.drawLine(startX, startY, endX, endY, linePaint)
                }
            }
        }
    }

    fun setResults(
        handLandmarkerResults: HandLandmarkerResult,
        imageHeight: Int,
        imageWidth: Int,
        runningMode: RunningMode = RunningMode.IMAGE
    ) {
        results = handLandmarkerResults

        this.imageHeight = imageHeight
        this.imageWidth = imageWidth

        // Recalculate scale factor based on current view size
        scaleFactor = when (runningMode) {
            RunningMode.IMAGE,
            RunningMode.VIDEO -> {
                min(width * 1f / imageWidth, height * 1f / imageHeight)
            }
            RunningMode.LIVE_STREAM -> {
                max(width * 1f / imageWidth, height * 1f / imageHeight)
            }
        }
        
        // LOGGING: Verify coordinates
        Log.d("OverlayView", "Drawing! Scale: $scaleFactor ViewSize: ${width}x${height} ImageSize: ${imageWidth}x${imageHeight}")
        
        invalidate() // Trigger redraw
    }
}