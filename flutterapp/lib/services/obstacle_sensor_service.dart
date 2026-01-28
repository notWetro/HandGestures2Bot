import 'dart:async';
import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';
import '../models/obstacle_status.dart';

class ObstacleSensorService {
  WebSocketChannel? _channel;
  final _obstacleStatusController =
      StreamController<ObstacleStatus>.broadcast();
  final _connectionStatusController = StreamController<bool>.broadcast();

  String? _robotIp;
  int _port = 8765;
  bool _isConnected = false;

  Stream<ObstacleStatus> get obstacleStatusStream =>
      _obstacleStatusController.stream;
  Stream<bool> get connectionStatusStream => _connectionStatusController.stream;
  bool get isConnected => _isConnected;

  void connect(String robotIp, {int port = 8765}) {
    _robotIp = robotIp;
    _port = port;
    _connectWebSocket();
  }

  void _connectWebSocket() {
    try {
      final uri = Uri.parse('ws://$_robotIp:$_port');
      _channel = WebSocketChannel.connect(uri);

      _isConnected = true;
      _connectionStatusController.add(true);

      _channel!.stream.listen(
        (message) {
          _handleMessage(message);
        },
        onError: (error) {
          _handleError(error);
        },
        onDone: () {
          _handleDisconnect();
        },
      );
    } catch (e) {
      _handleError(e);
    }
  }

  void _handleMessage(dynamic message) {
    try {
      final json = jsonDecode(message as String);

      if (json['type'] == 'obstacle_status') {
        final obstacleStatus = ObstacleStatus.fromJson(json);
        _obstacleStatusController.add(obstacleStatus);
      }
    } catch (e) {
      // Parsing error
    }
  }

  void _handleError(dynamic error) {
    print('WebSocket error: $error');
    _isConnected = false;
    _connectionStatusController.add(false);
  }

  void _handleDisconnect() {
    _isConnected = false;
    _connectionStatusController.add(false);

    // Auto-reconnect after 3 seconds 
    Future.delayed(const Duration(seconds: 3), () {
      if (_robotIp != null && !_isConnected) {
        _connectWebSocket();
      }
    });
  }

  void disconnect() {
    _channel?.sink.close();
    _isConnected = false;
    _connectionStatusController.add(false);
    _robotIp = null;
  }

  void dispose() {
    _channel?.sink.close();
    _obstacleStatusController.close();
    _connectionStatusController.close();
  }
}
