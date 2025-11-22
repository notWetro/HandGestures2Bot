package com.example.flutterapp

import android.util.Log
import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.widget.FrameLayout
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.*
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import com.google.mediapipe.tasks.vision.core.RunningMode
import io.flutter.plugin.platform.PlatformView
import java.util.concurrent.Executors

// NOTE: HandLandmarkerHelper and OverlayView must be in this package

class MyCameraView(private val activity: MainActivity, id: Int) : PlatformView {

    private val containerView: FrameLayout
    private lateinit var cameraProviderFuture: com.google.common.util.concurrent.ListenableFuture<ProcessCameraProvider>
    private lateinit var handLandmarkerHelper: HandLandmarkerHelper
    private lateinit var previewView: PreviewView
    private lateinit var overlayView: OverlayView
    
    // Executor for the background image analysis
    private val backgroundExecutor = Executors.newSingleThreadExecutor()

    init {
        // 1. Inflate Layout
        val inflater = activity.getSystemService(Context.LAYOUT_INFLATER_SERVICE) as LayoutInflater
        containerView = inflater.inflate(R.layout.camera_view_layout, null) as FrameLayout
        
        previewView = containerView.findViewById(R.id.preview_view)
        overlayView = containerView.findViewById(R.id.overlay_view)

        // 🔧 FIX 1: Force Camera to use TextureView so Overlay can sit on top
        previewView.implementationMode = PreviewView.ImplementationMode.COMPATIBLE

        // 🔧 FIX 2: Explicitly bring Overlay to front just in case
        overlayView.bringToFront()

        // 2. Initialize MediaPipe Helper (Your existing code...)
        handLandmarkerHelper = HandLandmarkerHelper(
            context = activity,
            runningMode = RunningMode.LIVE_STREAM,
            minHandDetectionConfidence = 0.5f, 
            minHandTrackingConfidence = 0.5f,
            minHandPresenceConfidence = 0.5f,
            handLandmarkerHelperListener = object : HandLandmarkerHelper.LandmarkerListener {
                 override fun onResults(resultBundle: HandLandmarkerHelper.ResultBundle) {
                    activity.runOnUiThread {
                        val firstResult = if (resultBundle.results.isNotEmpty()) resultBundle.results.first() else null
                        if (firstResult != null) {
                             overlayView.setResults(
                                handLandmarkerResults = firstResult, 
                                imageHeight = resultBundle.inputImageHeight, 
                                imageWidth = resultBundle.inputImageWidth,
                                runningMode = RunningMode.LIVE_STREAM
                            )
                            // Force redraw
                            overlayView.invalidate() 
                        } else {
                            overlayView.clear()
                        }
                    }
                }
                override fun onError(error: String, errorCode: Int) {
                    Log.e("MyCameraView", "Error: $error")
                }
            }
        )
        
        startCamera()
    }

    private fun startCamera() {
        cameraProviderFuture = ProcessCameraProvider.getInstance(activity)

        cameraProviderFuture.addListener({
            // Used to bind the lifecycle of cameras to the lifecycle owner
            val cameraProvider: ProcessCameraProvider = cameraProviderFuture.get()

            // Preview
            val preview = Preview.Builder()
                .build()
                .also {
                    it.setSurfaceProvider(previewView.surfaceProvider)
                }

            // Image Analysis (MediaPipe)
            val imageAnalyzer = ImageAnalysis.Builder()
                .setTargetRotation(previewView.display.rotation)
                .setBackpressureStrategy(androidx.camera.core.ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(androidx.camera.core.ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .build()
                .also {
                    it.setAnalyzer(backgroundExecutor) { imageProxy ->
                        // Process the image frame with MediaPipe
                        handLandmarkerHelper.detectLiveStream(imageProxy, isFrontCamera = false) 
                    }
                }

            // Select back camera as a default
            val cameraSelector = CameraSelector.DEFAULT_BACK_CAMERA

            try {
                // Unbind use cases before rebinding
                cameraProvider.unbindAll()

                // Bind use cases to camera
                cameraProvider.bindToLifecycle(
                    activity, 
                    cameraSelector, 
                    preview, 
                    imageAnalyzer
                )

            } catch(exc: Exception) {
                Log.e("MyCameraView", "Use case binding failed", exc)
            }

        }, ContextCompat.getMainExecutor(activity))
    }
    
    override fun getView(): View {
        return containerView
    }

    override fun dispose() {
        if (::cameraProviderFuture.isInitialized && cameraProviderFuture.isDone) {
             // Run on main thread to safely unbind
             activity.runOnUiThread {
                 cameraProviderFuture.get().unbindAll()
             }
        }
        handLandmarkerHelper.clearHandLandmarker()
        backgroundExecutor.shutdown()
    }
}