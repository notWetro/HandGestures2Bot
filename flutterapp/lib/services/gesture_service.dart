import 'dart:async';
import 'dart:io' show Platform;
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import '../services/dance_service.dart';

class GestureService {
  // Method channel for bi-directional communication
  static const MethodChannel _channel = MethodChannel('gesture_channel');
  static const MethodChannel _adminChannel =
    MethodChannel('gesture_admin_channel');

  final _gestureController = StreamController<String>.broadcast();
  final _saveSuccessController = StreamController<String>.broadcast();

  static const String _androidViewType = 'my_camera_view';
  static const String _iosViewType = 'my_camera_view';

  GestureService() {
    _channel.setMethodCallHandler(_handleMethodCall);
  }

  Stream<String> get onGesture => _gestureController.stream;

  Stream<String> get onSaveSuccess => _saveSuccessController.stream;

  Future<void> _handleMethodCall(MethodCall call) async {
    switch (call.method) {
      case 'onGesture':
        _gestureController.add(call.arguments as String);
        break;
      case 'onSaveSuccess':
        _saveSuccessController.add(call.arguments as String);
        break;
      default:
        debugPrint('GestureService: Unhandled method call: ${call.method}');
    }
  }

  Future<void> saveGesture(String name) async {
    try {
      await _channel.invokeMethod('saveGesture', name);
    } on PlatformException catch (e) {
      debugPrint("Failed to save gesture: '${e.message}'.");
    }
  }

  Future<List<String>> getGestureList() async {
    try {
      final List<dynamic>? result = await _channel.invokeMethod(
        'getGestureList',
      );
      return result?.cast<String>() ?? [];
    } on PlatformException catch (e) {
      return [];
    }
  }

  /// Builds and returns native camera view widget.
  Widget buildCameraView() {
    if (Platform.isAndroid) {
      return const AndroidView(
        viewType: _androidViewType,
        layoutDirection: TextDirection.ltr,
        creationParamsCodec: StandardMessageCodec(),
      );
    } else if (Platform.isIOS) {
      return const UiKitView(
        viewType: _iosViewType,
        layoutDirection: TextDirection.ltr,
        creationParamsCodec: StandardMessageCodec(),
      );
    }
    return const Center(child: Text("Platform not supported"));
  }

  void dispose() {
    _gestureController.close();
    _saveSuccessController.close();
  }

  Future<void> deleteGesture(String name) async {
    debugPrint("FLUTTER deleteGesture() called with name='$name'");
    try {
      await _adminChannel.invokeMethod('deleteGesture', name);
      debugPrint("FLUTTER deleteGesture() success for '$name'");
    } catch (e) {
      debugPrint("FLUTTER deleteGesture() error: $e");
      rethrow;
    }
  }
}
