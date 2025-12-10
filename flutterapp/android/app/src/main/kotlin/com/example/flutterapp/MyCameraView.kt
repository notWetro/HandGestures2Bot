package com.example.flutterapp

import android.util.Log
import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.widget.FrameLayout
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import com.google.mediapipe.tasks.vision.core.RunningMode
import io.flutter.plugin.platform.PlatformView
import java.util.concurrent.Executors
import io.flutter.plugin.common.BinaryMessenger
import io.flutter.plugin.common.MethodChannel

class MyCameraView(
    private val activity: MainActivity, // Needs Activity for Lifecycle
    messenger: BinaryMessenger, 
    id: Int
) : PlatformView {

    private val containerView: FrameLayout
    private lateinit var cameraProviderFuture: com.google.common.util.concurrent.ListenableFuture<ProcessCameraProvider>
    private lateinit var handLandmarkerHelper: HandLandmarkerHelper
    private lateinit var previewView: PreviewView
    private lateinit var overlayView: OverlayView
    
    // Logic Variables
    private val gestureRecognizer = GestureRecognizer(activity) // Pass Context
    private var lastGesture = "UNKNOWN"
    
    // Flag to capture next frame
    private var pendingSaveName: String? = null

    // Channel
    private val methodChannel: MethodChannel = MethodChannel(messenger, "gesture_channel")
    private val backgroundExecutor = Executors.newSingleThreadExecutor()

    init {
        // Setup Layout
        val inflater = activity.getSystemService(Context.LAYOUT_INFLATER_SERVICE) as LayoutInflater
        containerView = inflater.inflate(R.layout.camera_view_layout, null) as FrameLayout
        previewView = containerView.findViewById(R.id.preview_view)
        overlayView = containerView.findViewById(R.id.overlay_view)
        previewView.implementationMode = PreviewView.ImplementationMode.COMPATIBLE
        overlayView.bringToFront()

        // Listen for Flutter Commands ("saveGesture")
        methodChannel.setMethodCallHandler { call, result ->
            when (call.method) {
                "saveGesture" -> {
                    val name = call.arguments as String
                    pendingSaveName = name
                    Log.d("MyCameraView", "Save Request Received for: $name")
                    result.success(null)
                }
                "getGestureList" -> {
                    // Ask the brain for the list
                    val names = gestureRecognizer.getSavedGestureNames()
                    // Send List<String> back to Flutter
                    result.success(names)
                }
                // -------------------
                else -> {
                    result.notImplemented()
                }
            }
        }

        // Initialize AI
        handLandmarkerHelper = HandLandmarkerHelper(
            context = activity,
            runningMode = RunningMode.LIVE_STREAM,
            handLandmarkerHelperListener = object : HandLandmarkerHelper.LandmarkerListener {
                 override fun onResults(resultBundle: HandLandmarkerHelper.ResultBundle) {
                    activity.runOnUiThread {
                        val firstResult = if (resultBundle.results.isNotEmpty()) resultBundle.results.first() else null
                        
                        if (firstResult != null && firstResult.landmarks().isNotEmpty()) {
                            val landmarks = firstResult.landmarks()[0]
                            
                            // --- SAVE LOGIC ---
                            if (pendingSaveName != null) {
                                gestureRecognizer.saveTemplate(pendingSaveName!!, landmarks)
                                pendingSaveName = null // Reset flag
                                methodChannel.invokeMethod("onSaveSuccess", "Saved!")
                            }

                            // --- RECOGNITION LOGIC ---
                            val currentGesture = gestureRecognizer.recognize(landmarks)

                            if (currentGesture != lastGesture) {
                                lastGesture = currentGesture
                                methodChannel.invokeMethod("onGesture", currentGesture)
                            }

                             overlayView.setResults(
                                handLandmarkerResults = firstResult, 
                                imageHeight = resultBundle.inputImageHeight, 
                                imageWidth = resultBundle.inputImageWidth,
                                runningMode = RunningMode.LIVE_STREAM
                            )
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
            val cameraProvider = cameraProviderFuture.get()
            val preview = Preview.Builder().build().also { it.setSurfaceProvider(previewView.surfaceProvider) }
            val imageAnalyzer = ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .build()
                .also {
                    it.setAnalyzer(backgroundExecutor) { imageProxy ->
                        handLandmarkerHelper.detectLiveStream(imageProxy, false) 
                    }
                }

            try {
                cameraProvider.unbindAll()
                cameraProvider.bindToLifecycle(activity, CameraSelector.DEFAULT_BACK_CAMERA, preview, imageAnalyzer)
            } catch(exc: Exception) {
                Log.e("MyCameraView", "Binding failed", exc)
            }
        }, ContextCompat.getMainExecutor(activity))
    }
    
    override fun getView(): View = containerView

    override fun dispose() {
        if (::cameraProviderFuture.isInitialized) {
             activity.runOnUiThread { cameraProviderFuture.get().unbindAll() }
        }
        handLandmarkerHelper.clearHandLandmarker()
        backgroundExecutor.shutdown()
    }
}