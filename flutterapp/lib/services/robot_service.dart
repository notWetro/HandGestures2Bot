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

  Future<void> initialize() async {
    final savedIp = await _bluetoothService.loadSavedIpAddress();
    if (savedIp != null && savedIp.isNotEmpty) {
      _ipAddress = savedIp;
    }
  }

  void setConnection(String ip, String port) {
    _ipAddress = ip;
    _port = port;
    disconnect();
  }

  Future<void> connect() async {
    await initialize();

    if (_channel != null) {
      try {
        await _channel!.ready;
        debugPrint('Already connected.');
        return;
      } catch (e) {
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
      _channel = null; 
      rethrow; 
    }
  }

  void disconnect() {
    if (_channel != null) {
      _channel!.sink.close();
      _channel = null;
    }
  }

  void sendCommand(String movement) {
    if (_channel != null) {
      _channel!.sink.add(jsonEncode({'movement': movement}));
    }
  }

  String get currentIpAddress => _ipAddress;

  Future<String?> getRobotIp() async {
    await initialize();
    return _ipAddress;
  }
}
