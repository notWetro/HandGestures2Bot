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
    return const MaterialApp(home: MyHomePage());
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  var isPermissionGranted = Platform.isAndroid ? false : true;
  static const cameraPermission = MethodChannel('camera_permission');
  int _selectedIndex = 0;

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

  Widget cameraView() {
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

  Widget voiceView() {
    return const SafeArea(
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.mic, size: 80, color: Colors.blue),
            SizedBox(height: 20),
            Text(
              'Voice Control',
              style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 10),
            Text('Voice control features coming soon...'),
          ],
        ),
      ),
    );
  }

  Widget settingsView() {
    return const SafeArea(
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.settings, size: 80, color: Colors.grey),
            SizedBox(height: 20),
            Text(
              'Settings',
              style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 10),
            Text('App settings will be available here...'),
          ],
        ),
      ),
    );
  }

  void _onItemTapped(int index) {
    setState(() {
      _selectedIndex = index;
    });
  }

  @override
  Widget build(BuildContext context) {
    List<Widget> pages = [cameraView(), voiceView(), settingsView()];

    return Scaffold(
      body: pages[_selectedIndex],
      bottomNavigationBar: BottomNavigationBar(
        items: const <BottomNavigationBarItem>[
          BottomNavigationBarItem(icon: Icon(Icons.camera_alt), label: 'Cam'),
          BottomNavigationBarItem(icon: Icon(Icons.mic), label: 'Voice'),
          BottomNavigationBarItem(
            icon: Icon(Icons.settings),
            label: 'Settings',
          ),
        ],
        currentIndex: _selectedIndex,
        selectedItemColor: Colors.blue,
        onTap: _onItemTapped,
      ),
    );
  }
}
