import 'dart:async';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutterapp/services/gesture_service.dart';

class CameraView extends StatefulWidget {
  const CameraView({super.key});

  @override
  State<CameraView> createState() => _CameraViewState();
}

class _CameraViewState extends State<CameraView> {
  bool isPermissionGranted = Platform.isAndroid ? false : true;
  static const cameraPermission = MethodChannel('camera_permission');

  // Gesture Service and State
  final GestureService _gestureService = GestureService();
  StreamSubscription? _gestureSubscription;
  String detectedGesture = "Waiting...";

  @override
  void initState() {
    super.initState();
    if (Platform.isAndroid) {
      _getCameraPermissionAndroid();
    }
    // Start listening to the service's stream
    _gestureSubscription = _gestureService.onGesture.listen((gesture) {
      if (mounted) {
        setState(() {
          detectedGesture = gesture;
        });
      }
    });
  }

  @override
  void dispose() {
    // Clean up to prevent memory leaks
    _gestureSubscription?.cancel();
    _gestureService.dispose();
    super.dispose();
  }

  // PERMISSION LOGIC
  Future<void> _getCameraPermissionAndroid() async {
    try {
      final bool result =
          await cameraPermission.invokeMethod('getCameraPermission');
      if (mounted) {
        setState(() {
          isPermissionGranted = result;
        });
      }
    } on PlatformException catch (e) {
      debugPrint("Failed to get camera permission: '${e.message}'.");
    }
  }

  @override
  Widget build(BuildContext context) {
    if (!isPermissionGranted) {
      return const SafeArea(
        child: Center(child: Text("Waiting for camera permission...")),
      );
    }

    return SafeArea(
      child: Stack(
        children: [
          // The Native Camera View, provided by the service
          Positioned.fill(
            child: _gestureService.buildCameraView(),
          ),

          // The Gesture Text Overlay
          Positioned(
            top: 40,
            left: 0,
            right: 0,
            child: Align(
              alignment: Alignment.topCenter,
              child: Container(
                padding:
                    const EdgeInsets.symmetric(horizontal: 24, vertical: 12),
                decoration: BoxDecoration(
                  color: Colors.black54, // Semi-transparent background
                  borderRadius: BorderRadius.circular(30),
                  border: Border.all(color: Colors.white30, width: 1),
                ),
                child: Text(
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