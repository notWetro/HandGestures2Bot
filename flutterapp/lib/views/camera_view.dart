import 'dart:io';
import 'dart:convert';
import 'package:flutter/services.dart';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:flutterapp/my_camera_view.dart';

class CameraView extends StatefulWidget {
  const CameraView({super.key});

  @override
  State<CameraView> createState() => _CameraViewState();
}

class _CameraViewState extends State<CameraView> {
  var isPermissionGranted = Platform.isAndroid ? false : true;
  static const cameraPermission = MethodChannel('camera_permission');

  @override
  void initState() {
    super.initState();
    if (Platform.isAndroid) {
      _getCameraPermissionAndroid();
    }
  }

  Future<void> _getCameraPermissionAndroid() async {
    try {
      final bool result = await cameraPermission.invokeMethod(
        'getCameraPermission',
      );
      setState(() {
        isPermissionGranted = result;
      });
    } on PlatformException catch (e) {
      debugPrint("Failed to get camera permission: '${e.message}'.");
    }
  }

  // Test function to connect to WebSocket server and send "Fist" gesture
  Future<void> connectionTest() async {
    try {
      debugPrint('Connecting to ws://localhost:8765...');
      final channel = WebSocketChannel.connect(
        Uri.parse('ws://172.20.10.2:8765'),
      );

      await channel.ready;
      debugPrint('Sending: Fist (Holding gesture for 5 seconds...)');

      for (int i = 0; i < 50; i++) {
        channel.sink.add(jsonEncode({'gesture': 'Fist'}));
        await Future.delayed(const Duration(milliseconds: 100));
      }

      debugPrint('Finished. Watchdog should stop it now.');
      await channel.sink.close();
    } catch (e) {
      debugPrint('Connection test failed: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return isPermissionGranted
        ? SafeArea(
            child: Stack(
              children: [
                const MyCameraView(),
                Positioned(
                  bottom: 16,
                  left: 16,
                  child: ElevatedButton(
                    onPressed: connectionTest,
                    child: const Text('connection to bot test'),
                  ),
                ),
              ],
            ),
          )
        : const SafeArea(
            child: Center(child: Text("Waiting for camera permission...")),
          );
  }
}
