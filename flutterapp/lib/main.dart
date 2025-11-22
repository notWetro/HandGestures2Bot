import 'dart:io';
import 'package:flutter/services.dart';
import 'package:flutter/material.dart';
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
  static const cameraPermission = MethodChannel('camera_permission'); // Fixed typo in variable name

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
      final bool result = await cameraPermission.invokeMethod('getCameraPermission');
      setState(() {
        isPermissionGranted = result;
      });
    } on PlatformException catch (e) {
      debugPrint("Failed to get camera permission: '${e.message}'.");
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: isPermissionGranted
          ? const SafeArea(child: MyCameraView()) // Your native view wrapper
          : const SafeArea(
              child: Center(
                child: Text("Waiting for camera permission..."),
              ),
            ),
    );
  }
}