import 'dart:io';
import 'dart:convert';
import 'package:flutter/services.dart';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
// Ensure this import points to the file where you created the AndroidView widget
import 'package:flutterapp/my_camera_view.dart';

void main() {
  runApp(const MainApp());
}

class MainApp extends StatelessWidget {
  const MainApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      // ❌ CHANGE THIS LINE:
      // home: Scaffold(body: Center(child: Text('Hello World!'))),

      // ✅ TO THIS:
      home: MyHomePage(),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  var isPermissionGranted = Platform.isAndroid ? false : true;
  static const cameraPermission = MethodChannel(
    'camera_permission',
  ); // Fixed typo in variable name

  @override
  void initState() {
    super.initState();
    // auch für IOS
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

  // Ergänze innerhalb deiner State-Klasse:

  Future<void> connectionTest() async {
    try {
      debugPrint('Connecting to ws://localhost:8765...');
      final channel = WebSocketChannel.connect(
        Uri.parse('ws://localhost:8765'),
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
    return Scaffold(
      body: isPermissionGranted
          ? SafeArea(
              child: Stack(
                children: [
                  const MyCameraView(), // deine vorhandene Kameraansicht
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
            ),
    );
  }
}
