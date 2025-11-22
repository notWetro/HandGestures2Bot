package com.example.flutterapp

import android.Manifest
import android.content.pm.PackageManager
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodChannel

class MainActivity: FlutterActivity() {
    private val CHANNEL = "camera_permission"
    private val CAMERA_PERMISSION_CODE = 101
    
    // Variable to hold the result object so we can reply after the user clicks Allow/Deny
    private var pendingResult: MethodChannel.Result? = null

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)

        // 1. Register your Camera View Factory
        flutterEngine
            .platformViewsController
            .registry
            .registerViewFactory(
                "my_camera_view",
                MyCameraViewFactory(this)
            )

        // 2. Handle the Permission Method Channel
        MethodChannel(flutterEngine.dartExecutor.binaryMessenger, CHANNEL).setMethodCallHandler { call, result ->
            if (call.method == "getCameraPermission") {
                if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
                    // Permission is already granted
                    result.success(true)
                } else {
                    // We need to ask for permission. 
                    // Save the 'result' to use it later in onRequestPermissionsResult
                    pendingResult = result
                    ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), CAMERA_PERMISSION_CODE)
                }
            } else {
                result.notImplemented()
            }
        }
    }

    // 3. Listen for the user's response (Allow/Deny)
    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        
        if (requestCode == CAMERA_PERMISSION_CODE) {
            if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // User clicked "Allow"
                pendingResult?.success(true)
            } else {
                // User clicked "Deny"
                pendingResult?.success(false)
            }
            pendingResult = null
        }
    }
}