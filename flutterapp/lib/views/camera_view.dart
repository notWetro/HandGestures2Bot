import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
// Ensure this import points to your actual native view wrapper
import 'package:flutterapp/my_camera_view.dart';

class CameraView extends StatefulWidget {
  const CameraView({super.key});

  @override
  State<CameraView> createState() => _CameraViewState();
}

class _CameraViewState extends State<CameraView> {
  // 1. Permission State
  bool isPermissionGranted = Platform.isAndroid ? false : true;
  static const cameraPermission = MethodChannel('camera_permission');

  // 2. Gesture State (This will update when Android detects a hand)
  String detectedGesture = "Waiting...";
  static const gestureChannel = MethodChannel('gesture_channel');

  @override
  void initState() {
    super.initState();
    // Check permissions on startup
    if (Platform.isAndroid) {
      _getCameraPermissionAndroid();
    }
    // Start listening to the "Brain"
    _startListeningForGestures();
  }

  // --- PERMISSION LOGIC ---
  Future<void> _getCameraPermissionAndroid() async {
    try {
      final bool result = await cameraPermission.invokeMethod('getCameraPermission');
      if (mounted) {
        setState(() {
          isPermissionGranted = result;
        });
      }
    } on PlatformException catch (e) {
      debugPrint("Failed to get camera permission: '${e.message}'.");
    }
  }

  // --- GESTURE LISTENER LOGIC ---
  void _startListeningForGestures() {
    gestureChannel.setMethodCallHandler((call) async {
      // Android shouts "onGesture" -> We update the text
      if (call.method == 'onGesture') {
        final String newGesture = call.arguments as String;
        if (mounted) {
          setState(() {
            detectedGesture = newGesture;
          });
        }
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    // If we don't have permission, show a simple text message
    if (!isPermissionGranted) {
      return const SafeArea(
        child: Center(child: Text("Waiting for camera permission...")),
      );
    }

    // If we have permission, show the Camera + The Text Overlay
    return SafeArea(
      child: Stack(
        children: [
          // LAYER 1: The Native Android Camera (Background)
          const Positioned.fill(
            child: MyCameraView(),
          ),

          // LAYER 2: The Gesture Text Overlay (Foreground)
          Positioned(
            top: 40, 
            left: 0, 
            right: 0,
            child: Align(
              alignment: Alignment.topCenter,
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 12),
                decoration: BoxDecoration(
                  color: Colors.black54, // Semi-transparent background
                  borderRadius: BorderRadius.circular(30),
                  border: Border.all(color: Colors.white30, width: 1),
                ),
                child: Text(
                  // This text updates automatically via setState above
                  "Gesture: $detectedGesture",
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 22,
                    fontWeight: FontWeight.bold,
                    letterSpacing: 1.0,
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}