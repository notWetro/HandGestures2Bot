import 'dart:async';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutterapp/services/gesture_service.dart';
import 'package:flutterapp/services/robot_service.dart';

class CameraView extends StatefulWidget {
  const CameraView({super.key});

  @override
  State<CameraView> createState() => _CameraViewState();
}

class _CameraViewState extends State<CameraView> {
  bool isPermissionGranted = Platform.isAndroid ? false : true;
  static const cameraPermission = MethodChannel('camera_permission');

  // Services
  final GestureService _gestureService = GestureService();
  final RobotService _robotService = RobotService();

  // State
  StreamSubscription? _gestureSubscription;
  String detectedGesture = "Waiting...";
  String robotCommand = "stop";

  @override
  void initState() {
    super.initState();
    if (Platform.isAndroid) {
      _getCameraPermissionAndroid();
    } else {
      _initializeServices();
    }
  }

  Future<void> _initializeServices() async {
    // Connect to the robot
    try {
      await _robotService.connect();
    } catch (e) {
      debugPrint("Failed to connect to robot on view init: $e");
    }

    // Start listening to the gesture service's stream
    _gestureSubscription = _gestureService.onGesture.listen((gesture) {
      if (mounted) {
        String newCommand = _mapGestureToCommand(gesture);
        setState(() {
          detectedGesture = gesture;
          robotCommand = newCommand;
        });
        debugPrint("Detected Gesture: $gesture, Mapped Command: $newCommand");
        _robotService.sendCommand(robotCommand);
      }
    });
  }

  String _mapGestureToCommand(String gesture) {
    switch (gesture.toLowerCase()) {
      case 'move forward':
        return 'forward';
      case 'move backward':
        return 'backward';
      case 'stop robot':
        return 'stop';
      case 'turn left':
        return 'left';
      case 'turn right':
        return 'right';
      default:
        return 'stop';
    }
  }

  @override
  void dispose() {
    // Clean up to prevent memory leaks
    _gestureSubscription?.cancel();
    _gestureService.dispose();
    _robotService.disconnect();
    super.dispose();
  }

  // PERMISSION LOGIC FOR ANDROID
  Future<void> _getCameraPermissionAndroid() async {
    try {
      final bool result =
          await cameraPermission.invokeMethod('getCameraPermission');
      if (mounted) {
        setState(() {
          isPermissionGranted = result;
        });
        if (result) {
          _initializeServices();
        }
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

          // The Gesture and Command Text Overlay
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
                child: Column(
                  children: [
                    Text(
                      "Gesture: $detectedGesture",
                      style: const TextStyle(
                        color: Colors.white,
                        fontSize: 22,
                        fontWeight: FontWeight.bold,
                        letterSpacing: 1.0,
                      ),
                    ),
                    const SizedBox(height: 8),
                    Text(
                      "Command: $robotCommand",
                      style: const TextStyle(
                        color: Colors.lightBlueAccent,
                        fontSize: 18,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}