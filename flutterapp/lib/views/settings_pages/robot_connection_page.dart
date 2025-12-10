import 'dart:convert';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

class RobotConnectionPage extends StatefulWidget {
  const RobotConnectionPage({super.key});

  @override
  State<RobotConnectionPage> createState() => _RobotConnectionPageState();
}

class _RobotConnectionPageState extends State<RobotConnectionPage> {
  final TextEditingController _ipController = TextEditingController();
  final TextEditingController _portController = TextEditingController();

  @override
  void initState() {
    super.initState();
    // Set default values
    _ipController.text = '172.20.10.3';
    _portController.text = '8765';
  }

  Future<void> _testConnection() async {
  try {
    final ip = _ipController.text;
    final port = _portController.text;

    final channel = WebSocketChannel.connect(Uri.parse('ws://$ip:$port'));
    await channel.ready;

    debugPrint('Connected! Sending forward for 3 seconds...');

    // Send forward commands at 10Hz (just like ros2 topic pub)
    for (int i = 0; i < 30; i++) {   // 30 × 100ms = 3 seconds
      channel.sink.add(jsonEncode({'movement': 'forward'}));
      await Future.delayed(const Duration(milliseconds: 100));
    }

    // NOW send stop
    channel.sink.add(jsonEncode({'movement': 'stop'}));
    debugPrint('Sent STOP after 3 seconds of stable forward.');

    await channel.sink.close();

  } catch (e) {
    debugPrint("Error: $e");
  }
}



  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Robot Connection'),
        backgroundColor: Colors.blue,
        foregroundColor: Colors.white,
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Configure Robot Connection',
              style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 20),

            TextField(
              controller: _ipController,
              decoration: const InputDecoration(
                labelText: 'Robot IP Address',
                border: OutlineInputBorder(),
                prefixIcon: Icon(Icons.router),
                hintText: '192.168.1.100',
              ),
            ),
            const SizedBox(height: 16),

            TextField(
              controller: _portController,
              decoration: const InputDecoration(
                labelText: 'Port',
                border: OutlineInputBorder(),
                prefixIcon: Icon(Icons.settings_ethernet),
                hintText: '8765',
              ),
              keyboardType: TextInputType.number,
            ),
            const SizedBox(height: 20),

            SizedBox(
              width: double.infinity,
              child: ElevatedButton(
                onPressed: () {
                  // TODO: Save connection settings
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: Text(
                        'Settings saved: ${_ipController.text}:${_portController.text}',
                      ),
                    ),
                  );
                },
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.blue,
                  foregroundColor: Colors.white,
                  padding: const EdgeInsets.symmetric(vertical: 16),
                ),
                child: const Text('Save Settings'),
              ),
            ),
            const SizedBox(height: 16),

            SizedBox(
              width: double.infinity,
              child: OutlinedButton(
                onPressed: _testConnection,
                style: OutlinedButton.styleFrom(
                  side: const BorderSide(color: Colors.blue),
                  padding: const EdgeInsets.symmetric(vertical: 16),
                ),
                child: const Text(
                  'Test Connection',
                  style: TextStyle(color: Colors.blue),
                ),
              ),
            ),

            const SizedBox(height: 20),
            Container(
              width: double.infinity,
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: Colors.blue.shade50,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.blue.shade200),
              ),
              child: Column(
                children: [
                  const Icon(Icons.info_outline, color: Colors.blue),
                  const SizedBox(height: 8),
                  const Text(
                    'Connection Info',
                    style: TextStyle(fontWeight: FontWeight.bold),
                  ),
                  const SizedBox(height: 4),
                  Text(
                    'Make sure your robot is running and accessible on the network.',
                    textAlign: TextAlign.center,
                    style: TextStyle(color: Colors.grey.shade700),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  @override
  void dispose() {
    _ipController.dispose();
    _portController.dispose();
    super.dispose();
  }
}
