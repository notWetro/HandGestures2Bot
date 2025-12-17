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
  List<String> commandHistory = [];

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
    // Start listening to the gesture service's stream FIRST
    _gestureSubscription = _gestureService.onGesture.listen((gesture) {
      if (mounted) {
        String newCommand = _mapGestureToCommand(gesture);
        setState(() {
          detectedGesture = gesture;
          robotCommand = newCommand;
          // Add command to history (keep last 5)
          commandHistory.insert(0, newCommand);
          if (commandHistory.length > 5) {
            commandHistory.removeLast();
          }
        });
        debugPrint("Detected Gesture: $gesture -> Command: $newCommand");
        _robotService.sendCommand(robotCommand);
      }
    });
    
    // Connect to the robot
    try {
      await _robotService.connect();
    } catch (e) {
      debugPrint("Failed to connect to robot on view init: $e");
    }
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

    return Padding(
      padding: const EdgeInsets.all(24.0),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const SizedBox(height: 20),
          const Text(
            "Position your hand in the camera view:",
            style: TextStyle(fontSize: 16, color: Colors.grey),
          ),
          const SizedBox(height: 20),
          Center(
            child: Container(
              height: 400,
              width: double.infinity,
              decoration: BoxDecoration(
                border: Border.all(color: Colors.blue, width: 2),
                borderRadius: BorderRadius.circular(24),
                boxShadow: [
                  BoxShadow(
                    color: Colors.black.withOpacity(0.1),
                    blurRadius: 10,
                    offset: const Offset(0, 5),
                  ),
                ],
              ),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(22),
                child: _gestureService.buildCameraView(),
              ),
            ),
          ),
          const SizedBox(height: 30),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
            decoration: BoxDecoration(
              color: Colors.blue.withOpacity(0.1),
              borderRadius: BorderRadius.circular(16),
              border: Border.all(color: Colors.blue.withOpacity(0.3), width: 1),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  children: [
                    const Icon(Icons.pan_tool, color: Colors.blue),
                    const SizedBox(width: 12),
                    Expanded(
                      child: Text(
                        detectedGesture,
                        style: const TextStyle(
                          fontSize: 20,
                          fontWeight: FontWeight.bold,
                          color: Colors.blue,
                        ),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 16),
                const Divider(),
                const SizedBox(height: 8),
                Row(
                  children: [
                    const Icon(Icons.history, color: Colors.green, size: 20),
                    const SizedBox(width: 8),
                    const Text(
                      "Command History:",
                      style: TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.bold,
                        color: Colors.green,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 8),
                if (commandHistory.isEmpty)
                  const Padding(
                    padding: EdgeInsets.symmetric(vertical: 8.0),
                    child: Text(
                      "No commands yet...",
                      style: TextStyle(
                        fontSize: 14,
                        color: Colors.grey,
                        fontStyle: FontStyle.italic,
                      ),
                    ),
                  )
                else
                  ...commandHistory.asMap().entries.map((entry) {
                    int index = entry.key;
                    String command = entry.value;
                    return Padding(
                      padding: const EdgeInsets.symmetric(vertical: 4.0),
                      child: Row(
                        children: [
                          Container(
                            width: 24,
                            height: 24,
                            decoration: BoxDecoration(
                              color: index == 0 
                                  ? Colors.green 
                                  : Colors.grey.withOpacity(0.3),
                              shape: BoxShape.circle,
                            ),
                            child: Center(
                              child: Text(
                                "${index + 1}",
                                style: TextStyle(
                                  fontSize: 12,
                                  fontWeight: FontWeight.bold,
                                  color: index == 0 ? Colors.white : Colors.black54,
                                ),
                              ),
                            ),
                          ),
                          const SizedBox(width: 12),
                          Text(
                            command,
                            style: TextStyle(
                              fontSize: 16,
                              fontWeight: index == 0 ? FontWeight.bold : FontWeight.normal,
                              color: index == 0 ? Colors.green : Colors.black87,
                            ),
                          ),
                        ],
                      ),
                    );
                  }).toList(),
              ],
            ),
          ),
        ],
      ),
    );
  }
}