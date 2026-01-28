import 'package:flutter/material.dart';
import 'package:flutterapp/views/settings_pages/robot_connection_page.dart';
import 'package:flutterapp/views/settings_pages/hand_gestures_page.dart';
import 'package:flutterapp/views/settings_pages/bluetooth_setup_page.dart';

class SettingsView extends StatefulWidget {
  const SettingsView({super.key});

  @override
  State<SettingsView> createState() => _SettingsViewState();
}

class _SettingsViewState extends State<SettingsView> {
  void _navigateToBluetoothSetup() {
    Navigator.push(
      context,
      MaterialPageRoute(builder: (context) => const BluetoothSetupPage()),
    );
  }

  void _navigateToRobotConnection() {
    Navigator.push(
      context,
      MaterialPageRoute(builder: (context) => const RobotConnectionPage()),
    );
  }

  void _navigateToHandGestures() {
    Navigator.push(
      context,
      MaterialPageRoute(builder: (context) => const HandGesturesPage()),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Settings'),
        backgroundColor: Colors.blue,
        foregroundColor: Colors.white,
      ),
      body: ListView(
        padding: const EdgeInsets.all(16.0),
        children: [
          // Bluetooth WiFi Setup (NEW - Primary method)
          Card(
            child: ListTile(
              leading: const Icon(Icons.bluetooth_connected, color: Colors.blue),
              title: const Text('Bluetooth WiFi Setup'),
              subtitle: const Text('Connect robot via Bluetooth and configure WiFi'),
              trailing: const Icon(Icons.arrow_forward_ios),
              onTap: _navigateToBluetoothSetup,
            ),
          ),
          const SizedBox(height: 8),

          // Robot Connection Setting (Manual fallback)
          Card(
            child: ListTile(
              leading: const Icon(Icons.wifi, color: Colors.orange),
              title: const Text('Manual Connection'),
              subtitle: const Text('Enter IP address manually (fallback)'),
              trailing: const Icon(Icons.arrow_forward_ios),
              onTap: _navigateToRobotConnection,
            ),
          ),
          const SizedBox(height: 8),

          // Hand Gestures Setting
          Card(
            child: ListTile(
              leading: const Icon(Icons.pan_tool, color: Colors.orange),
              title: const Text('Change Hand Gestures'),
              subtitle: const Text('Customize gesture recognition'),
              trailing: const Icon(Icons.arrow_forward_ios),
              onTap: _navigateToHandGestures,
            ),
          ),
          const SizedBox(height: 8),

          // Additional Settings (for future use)
          Card(
            child: ListTile(
              leading: const Icon(Icons.info, color: Colors.grey),
              title: const Text('About'),
              subtitle: const Text('App information and version'),
              trailing: const Icon(Icons.arrow_forward_ios),
              onTap: () {
                // TODO: Implement About page
                ScaffoldMessenger.of(context).showSnackBar(
                  const SnackBar(content: Text('About page coming soon...')),
                );
              },
            ),
          ),
        ],
      ),
    );
  }
}