import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutterapp/services/native_bluetooth_service.dart';
import 'package:flutterapp/services/robot_service.dart';

class BluetoothSetupPage extends StatefulWidget {
  const BluetoothSetupPage({super.key});

  @override
  State<BluetoothSetupPage> createState() => _BluetoothSetupPageState();
}

class _BluetoothSetupPageState extends State<BluetoothSetupPage> {
  final NativeBluetoothService _bluetoothService = NativeBluetoothService();

  final TextEditingController _ssidController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();

  List<Map<String, String>> _devices = [];
  String? _selectedDeviceId;
  String? _selectedDeviceName;
  bool _isScanning = false;
  bool _isConnecting = false;
  bool _isConnected = false;
  bool _isSendingCredentials = false;
  String? _receivedIpAddress;
  String _currentStatus = "idle"; // idle, connecting, connected, failed
  StreamSubscription? _ipSubscription;
  StreamSubscription? _statusSubscription;
  StreamSubscription? _deviceSubscription;

  @override
  void initState() {
    super.initState();
    _loadSavedIp();

    _ipSubscription = _bluetoothService.onIpAddressReceived.listen((
      ipAddress,
    ) async {
      debugPrint('UI: Received IP Address from Robot: $ipAddress');
      if (mounted) {
        setState(() {
          _receivedIpAddress = ipAddress;
        });

        await _bluetoothService.saveIpAddress(ipAddress);

        RobotService().setConnection(ipAddress, '8765');
        RobotService().connect();

        _showSuccessDialog(ipAddress);
      }
    });

    // Listen for status updates from robot
    _statusSubscription = _bluetoothService.onStatusUpdate.listen((status) {
      if (mounted) {
        setState(() {
          _currentStatus = status['status'] ?? 'idle';
        });

        if (status['status'] == 'failed') {
          _showErrorDialog(
            "WiFi connection failed: ${status['reason'] ?? 'Unknown error'}",
          );
        }
      }
    });

    // Listen for discovered devices from the stream
    _deviceSubscription = _bluetoothService.onDeviceFound.listen((device) {
      if (mounted) {
        final name = device['name'] ?? '';
        // Double-check filter in UI to ensure only TurtleBot is shown
        if (name.contains('TurtleBot')) {
          setState(() {
            final exists = _devices.any(
              (d) => d['address'] == device['address'],
            );
            if (!exists) {
              _devices.add(device);
            }
          });
        }
      }
    });
  }

  @override
  void dispose() {
    _ssidController.dispose();
    _passwordController.dispose();
    _ipSubscription?.cancel();
    _statusSubscription?.cancel();
    _deviceSubscription?.cancel();
    super.dispose();
  }

  Future<void> _loadSavedIp() async {
    final savedIp = await _bluetoothService.loadSavedIpAddress();
    if (savedIp != null && mounted) {
      setState(() {
        _receivedIpAddress = savedIp;
      });
    }
  }

  Future<void> _scanForDevices() async {
    setState(() {
      _isScanning = true;
      _devices = [];
    });

    try {
      // Start BLE scan - will return devices as they are discovered
      final success = await _bluetoothService.startScanning();

      if (mounted && success) {
        // Wait a bit to see if any devices are found via the stream
        await Future.delayed(const Duration(seconds: 2));
        if (_devices.isEmpty && mounted) {
          _showInfoDialog(
            "No Devices Found",
            "Make sure:\n"
                "• TurtleBot is powered on\n"
                "• TurtleBot BLE service is running\n"
                "• Bluetooth is enabled on iPhone\n"
                "• Robot is advertising with service UUID:\n"
                "  12345678-1234-5678-1234-56789abcdef0",
          );
        }
      }
    } catch (e) {
      _showErrorDialog("Error: $e");
    } finally {
      if (mounted) {
        setState(() {
          _isScanning = false;
        });
      }
    }
  }

  Future<void> _connectToDevice(String deviceId, String deviceName) async {
    setState(() {
      _isConnecting = true;
      _selectedDeviceId = deviceId;
      _selectedDeviceName = deviceName;
    });

    try {
      final success = await _bluetoothService.connectToDevice(deviceId);
      if (mounted) {
        setState(() {
          _isConnected = success;
          _isConnecting = false;
        });

        if (!success) {
          _showErrorDialog(
            "Failed to connect. Make sure the robot service is running.",
          );
        }
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _isConnecting = false;
        });
        _showErrorDialog("Connection error: $e");
      }
    }
  }

  Future<void> _sendWifiCredentials() async {
    if (_ssidController.text.isEmpty) {
      _showErrorDialog("Please enter WiFi SSID");
      return;
    }

    setState(() {
      _isSendingCredentials = true;
    });

    try {
      final success = await _bluetoothService.sendWiFiCredentials(
        _ssidController.text,
        _passwordController.text,
      );

      if (mounted) {
        setState(() {
          _isSendingCredentials = false;
        });

        if (success) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(
              content: Text(
                "WiFi credentials sent! Waiting for robot to connect...",
              ),
              backgroundColor: Colors.green,
            ),
          );
        } else {
          _showErrorDialog("Failed to send credentials");
        }
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _isSendingCredentials = false;
        });
        _showErrorDialog("Error: $e");
      }
    }
  }

  void _showSuccessDialog(String ipAddress) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text("Setup Complete!"),
        content: Text(
          "Robot connected to WiFi!\n\n"
          "IP Address: $ipAddress\n\n"
          "The app will now use this IP address to communicate with the robot via WiFi.",
        ),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
              Navigator.of(context).pop(); // Go back to settings
            },
            child: const Text("Done"),
          ),
        ],
      ),
    );
  }

  void _showErrorDialog(String message) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text("Error"),
        content: Text(message),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: const Text("OK"),
          ),
        ],
      ),
    );
  }

  void _showInfoDialog(String title, String message) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text(title),
        content: Text(message),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: const Text("OK"),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Bluetooth WiFi Setup'),
        backgroundColor: Colors.blue,
        foregroundColor: Colors.white,
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // Instructions
            Card(
              color: Colors.blue[50],
              child: const Padding(
                padding: EdgeInsets.all(16.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      "Setup Instructions:",
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                    SizedBox(height: 8),
                    Text("1. Make sure TurtleBot BLE is advertising"),
                    Text("2. Click 'Scan for Devices' below"),
                    Text("3. Select your robot from the list"),
                    Text("4. Enter WiFi credentials and connect"),
                  ],
                ),
              ),
            ),

            const SizedBox(height: 20),

            // Connection Status Indicator
            if (_isConnected && _currentStatus != 'idle')
              Card(
                color: _currentStatus == 'connecting'
                    ? Colors.orange[50]
                    : _currentStatus == 'connected'
                    ? Colors.green[50]
                    : Colors.red[50],
                child: Padding(
                  padding: const EdgeInsets.all(16.0),
                  child: Row(
                    children: [
                      if (_currentStatus == 'connecting')
                        const SizedBox(
                          width: 20,
                          height: 20,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        )
                      else if (_currentStatus == 'connected')
                        const Icon(Icons.check_circle, color: Colors.green)
                      else
                        const Icon(Icons.error, color: Colors.red),
                      const SizedBox(width: 12),
                      Expanded(
                        child: Text(
                          _currentStatus == 'connecting'
                              ? "Robot is connecting to WiFi..."
                              : _currentStatus == 'connected'
                              ? "Robot connected to WiFi!"
                              : "WiFi connection failed",
                          style: const TextStyle(fontWeight: FontWeight.bold),
                        ),
                      ),
                    ],
                  ),
                ),
              ),

            const SizedBox(height: 20),

            // Current saved IP
            if (_receivedIpAddress != null)
              Card(
                color: Colors.green[50],
                child: Padding(
                  padding: const EdgeInsets.all(16.0),
                  child: Row(
                    children: [
                      const Icon(Icons.check_circle, color: Colors.green),
                      const SizedBox(width: 12),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            const Text(
                              "Saved IP Address:",
                              style: TextStyle(fontWeight: FontWeight.bold),
                            ),
                            Text(_receivedIpAddress!),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
              ),

            const SizedBox(height: 20),

            // Step 1: Scan for devices
            Text(
              "Step 1: Scan for Devices",
              style: Theme.of(context).textTheme.titleLarge,
            ),
            const SizedBox(height: 10),
            Card(
              color: Colors.blue[50],
              child: Padding(
                padding: const EdgeInsets.all(12.0),
                child: Row(
                  children: [
                    const Icon(Icons.info_outline, color: Colors.blue),
                    const SizedBox(width: 12),
                    Expanded(
                      child: Text(
                        "Make sure TurtleBot BLE service is running",
                        style: TextStyle(fontSize: 13, color: Colors.blue[900]),
                      ),
                    ),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 10),
            ElevatedButton.icon(
              onPressed: _isScanning ? null : _scanForDevices,
              icon: _isScanning
                  ? const SizedBox(
                      width: 20,
                      height: 20,
                      child: CircularProgressIndicator(strokeWidth: 2),
                    )
                  : const Icon(Icons.bluetooth_searching),
              label: Text(_isScanning ? "Scanning..." : "Scan for Devices"),
              style: ElevatedButton.styleFrom(
                minimumSize: const Size(double.infinity, 48),
              ),
            ),

            const SizedBox(height: 10),

            // Device list
            if (_devices.isNotEmpty) ...[
              const Text("Discovered devices:"),
              const SizedBox(height: 8),
              ..._devices.map((device) {
                final deviceId = device['address'] ?? '';
                final deviceName = device['name'] ?? 'Unknown Device';
                final isSelected = _selectedDeviceId == deviceId;
                return Card(
                  color: isSelected ? Colors.blue[50] : null,
                  child: ListTile(
                    leading: Icon(
                      Icons.bluetooth,
                      color: isSelected ? Colors.blue : null,
                    ),
                    title: Text(deviceName),
                    subtitle: Text(deviceId),
                    trailing: _isConnecting && isSelected
                        ? const CircularProgressIndicator()
                        : _isConnected && isSelected
                        ? const Icon(Icons.check_circle, color: Colors.green)
                        : null,
                    onTap: _isConnected
                        ? null
                        : () => _connectToDevice(deviceId, deviceName),
                  ),
                );
              }).toList(),
            ],

            const SizedBox(height: 30),

            // Step 2: WiFi Credentials
            if (_isConnected) ...[
              Text(
                "Step 2: Enter WiFi Credentials",
                style: Theme.of(context).textTheme.titleLarge,
              ),
              const SizedBox(height: 10),
              TextField(
                controller: _ssidController,
                decoration: InputDecoration(
                  labelText: "WiFi SSID",
                  hintText: "Enter WiFi network name",
                  border: OutlineInputBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                  prefixIcon: const Icon(Icons.wifi),
                ),
              ),
              const SizedBox(height: 10),
              TextField(
                controller: _passwordController,
                obscureText: true,
                decoration: InputDecoration(
                  labelText: "WiFi Password",
                  hintText: "Enter WiFi password",
                  border: OutlineInputBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                  prefixIcon: const Icon(Icons.lock),
                ),
              ),
              const SizedBox(height: 20),
              ElevatedButton.icon(
                onPressed: _isSendingCredentials ? null : _sendWifiCredentials,
                icon: _isSendingCredentials
                    ? const SizedBox(
                        width: 20,
                        height: 20,
                        child: CircularProgressIndicator(strokeWidth: 2),
                      )
                    : const Icon(Icons.send),
                label: Text(
                  _isSendingCredentials ? "Sending..." : "Send to Robot",
                ),
                style: ElevatedButton.styleFrom(
                  minimumSize: const Size(double.infinity, 48),
                  backgroundColor: Colors.green,
                  foregroundColor: Colors.white,
                ),
              ),
            ],
          ],
        ),
      ),
    );
  }
}
