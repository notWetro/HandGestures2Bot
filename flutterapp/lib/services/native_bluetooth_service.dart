import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';

class NativeBluetoothService {
  static final NativeBluetoothService _instance = NativeBluetoothService._internal();
  factory NativeBluetoothService() => _instance;
  NativeBluetoothService._internal();

  static const MethodChannel _channel = MethodChannel('bluetooth_channel');
  
  final _ipAddressController = StreamController<String>.broadcast();
  final _statusController = StreamController<Map<String, dynamic>>.broadcast();
  
  Stream<String> get onIpAddressReceived => _ipAddressController.stream;
  Stream<Map<String, dynamic>> get onStatusUpdate => _statusController.stream;

  // Start BLE scan for devices
  Future<List<Map<String, String>>> startScanning() async {
    debugPrint("🔵 BLE: Starting scan...");
    
    try {
      final result = await _channel.invokeMethod('startScanning');
      debugPrint("🔵 BLE: Scan result: $result");
      
      if (result is List) {
        return result.map((device) => Map<String, String>.from(device)).toList();
      }
      
      return [];
    } catch (e) {
      debugPrint("❌ BLE: Scan error: $e");
      return [];
    }
  }
  
  // Stop BLE scan
  Future<void> stopScanning() async {
    try {
      await _channel.invokeMethod('stopScanning');
      debugPrint("🔵 BLE: Scan stopped");
    } catch (e) {
      debugPrint("❌ BLE: Stop scan error: $e");
    }
  }

  // Get connected Bluetooth devices from iOS system
  Future<List<Map<String, String>>> getConnectedDevices() async {
    debugPrint("🔵 BLE: Getting connected devices...");
    
    try {
      final result = await _channel.invokeMethod('getConnectedDevices');
      debugPrint("🔵 BLE: Result: $result");
      
      // Parse the result - it should be a list of devices
      if (result is Map) {
        // For now, return empty list - we'll implement proper parsing
        return [];
      }
      
      return [];
    } catch (e) {
      debugPrint("❌ BLE: Error getting devices: $e");
      return [];
    }
  }

  // Connect to a specific Bluetooth device by ID
  Future<bool> connectToDevice(String deviceId) async {
    debugPrint("🔵 BLE: Connecting to device: $deviceId");
    
    try {
      final result = await _channel.invokeMethod('connectToDevice', deviceId);
      debugPrint("✅ BLE: Connected: $result");
      return result == true;
    } catch (e) {
      debugPrint("❌ BLE: Connection error: $e");
      return false;
    }
  }

  // Send WiFi credentials to the connected device
  Future<bool> sendWiFiCredentials(String ssid, String password) async {
    debugPrint("🔵 BLE: Sending WiFi credentials...");
    debugPrint("🔵 BLE: SSID: $ssid");
    
    try {
      final result = await _channel.invokeMethod('sendWiFiCredentials', {
        'ssid': ssid,
        'password': password,
      });
      
      debugPrint("✅ BLE: Credentials sent: $result");
      return result == true;
    } catch (e) {
      debugPrint("❌ BLE: Error sending credentials: $e");
      return false;
    }
  }

  // Disconnect from the device
  Future<void> disconnect() async {
    try {
      await _channel.invokeMethod('disconnect');
      debugPrint("🔵 BLE: Disconnected");
    } catch (e) {
      debugPrint("❌ BLE: Disconnect error: $e");
    }
  }

  // Save IP address to local storage
  Future<void> saveIpAddress(String ipAddress) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString('robot_ip_address', ipAddress);
      debugPrint("💾 Saved IP address to local storage: $ipAddress");
    } catch (e) {
      debugPrint("❌ Error saving IP address: $e");
    }
  }

  // Load IP address from local storage
  Future<String?> loadSavedIpAddress() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final ipAddress = prefs.getString('robot_ip_address');
      if (ipAddress != null) {
        debugPrint("💾 Loaded saved IP address: $ipAddress");
      }
      return ipAddress;
    } catch (e) {
      debugPrint("❌ Error loading IP address: $e");
      return null;
    }
  }

  void dispose() {
    _ipAddressController.close();
    _statusController.close();
  }
}
