import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

class RobotService {
  // Singleton setup
  static final RobotService _instance = RobotService._internal();
  factory RobotService() => _instance;
  RobotService._internal();

  WebSocketChannel? _channel;
  String _ipAddress = '172.20.10.3'; // Default IP
  String _port = '8765'; // Default Port

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
    // Prevent reconnecting if already connected
    if (_channel != null && _channel!.sink.done == null) {
      debugPrint('Already connected.');
      return;
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
}
