import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'native_bluetooth_service.dart';

class RobotService {
  // Singleton setup
  static final RobotService _instance = RobotService._internal();
  factory RobotService() => _instance;
  RobotService._internal();

  WebSocketChannel? _channel;
  String _ipAddress = '172.20.10.3'; // Default IP
  String _port = '8765'; // Default Port
  
  final NativeBluetoothService _bluetoothService = NativeBluetoothService();

  // ============================================================================
  // BLUETOOTH SETUP FLOW - Load saved IP address from Bluetooth setup
  // ============================================================================
  Future<void> initialize() async {
    // Try to load previously saved IP address from Bluetooth setup
    final savedIp = await _bluetoothService.loadSavedIpAddress();
    if (savedIp != null && savedIp.isNotEmpty) {
      _ipAddress = savedIp;
      debugPrint("📱 Using saved IP address from Bluetooth setup: $_ipAddress");
    }
  }
  // ============================================================================

  // Update connection details
  void setConnection(String ip, String port) {
    _ipAddress = ip;
    _port = port;
    // If we are already connected, we should probably disconnect
    // and maybe even reconnect with the new settings.
    disconnect();
  }

  // Connect to the robot
  Future<void> connect() async {
    // Initialize first to load any saved IP
    await initialize();
    
    // Prevent reconnecting if already connected
    if (_channel != null) {
      try {
        // Check if channel is still alive
        await _channel!.ready;
        debugPrint('Already connected.');
        return;
      } catch (e) {
        // Channel is dead, need to reconnect
        _channel = null;
      }
    }

    try {
      final uri = Uri.parse('ws://$_ipAddress:$_port');
      _channel = WebSocketChannel.connect(uri);
      await _channel!.ready;
      debugPrint('Successfully connected to ws://$_ipAddress:$_port');
    } catch (e) {
      debugPrint("Error connecting to robot: $e");
      _channel = null; // Ensure channel is null on error
      rethrow; // Re-throw the exception to be handled by the caller
    }
  }

  // Disconnect from the robot
  void disconnect() {
    if (_channel != null) {
      _channel!.sink.close();
      _channel = null;
      debugPrint('Disconnected from robot.');
    }
  }

  // Send a command to the robot
  void sendCommand(String movement) {
    if (_channel != null) {
      debugPrint("Sending command: $movement");
      _channel!.sink.add(jsonEncode({'movement': movement}));
    } else {
      debugPrint('Cannot send command: Not connected to the robot.');
    }
  }
  
  String get currentIpAddress => _ipAddress;
  
  Future<String?> getRobotIp() async {
    await initialize();
    return _ipAddress;
  }
}
