import 'dart:async';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutterapp/services/gesture_service.dart';
import 'package:flutterapp/services/robot_service.dart';
import 'package:flutterapp/services/dance_service.dart';
import 'package:flutterapp/services/obstacle_sensor_service.dart';
import 'package:flutterapp/models/dance_move.dart';
import 'package:flutterapp/models/obstacle_status.dart';
import 'package:flutterapp/services/dance_music_controller.dart';

class CameraView extends StatefulWidget {
  const CameraView({super.key});

  @override
  State<CameraView> createState() => _CameraViewState();
}

class _CameraViewState extends State<CameraView> {
  bool isPermissionGranted = Platform.isAndroid ? false : true;
  static const cameraPermission = MethodChannel('camera_permission');

  // Services
  final GestureService _gestureService = GestureService();
  final RobotService _robotService = RobotService();
  final DanceService _danceService = DanceService();
  final ObstacleSensorService _obstacleSensorService = ObstacleSensorService();
  final DanceMusicController _musicController = DanceMusicController();

  // State
  StreamSubscription? _gestureSubscription;
  StreamSubscription? _obstacleSubscription;
  String detectedGesture = "Waiting...";
  String robotCommand = "stop";
  List<String> commandHistory = [];
  final ScrollController _scrollController = ScrollController();
  bool _isDancePlaying = false;
  ObstacleStatus? _currentObstacleStatus;

  @override
  void initState() {
    super.initState();
    if (Platform.isAndroid) {
      _getCameraPermissionAndroid();
    } else {
      _initializeServices();
    }
  }

  Future<void> _initializeServices() async {
    // Connect to obstacle sensor
    try {
      final robotIp = await _robotService.getRobotIp();
      if (robotIp != null && robotIp.isNotEmpty) {
        _obstacleSensorService.connect(robotIp);
        _obstacleSubscription = _obstacleSensorService.obstacleStatusStream
            .listen((status) {
              if (mounted) {
                setState(() {
                  _currentObstacleStatus = status;
                });
              }
            });
      }
    } catch (e) {
      debugPrint("Failed to connect to obstacle sensor: $e");
    }

    // Start listening to the gesture service's stream FIRST
    _gestureSubscription = _gestureService.onGesture.listen((gesture) async {
      debugPrint("📸 [GESTURE] received: '$gesture'");
      debugPrint("📸 [STATE] _isDancePlaying = $_isDancePlaying");

      if (mounted) {
        // Check if this gesture is assigned to a dance
        final dance = await _danceService.getDanceByGesture(gesture);

        debugPrint(
          "🎵 [LOOKUP] dance = ${dance?.name}, assignedGesture = ${dance?.assignedGesture}",
        );

        if (dance != null && !_isDancePlaying) {
          debugPrint("🔥 [TRIGGER] starting dance '${dance.name}'");
          // Play dance sequence
          setState(() {
            detectedGesture = gesture;
            commandHistory.insert(0, 'Dance: ${dance.name}');
            if (commandHistory.length > 50) {
              commandHistory.removeLast();
            }
          });
          debugPrint(
            "Detected Gesture: $gesture -> Playing Dance: ${dance.name}",
          );
          await _playDance(dance);
        } else if (!_isDancePlaying) {
          debugPrint("➡️ [COMMAND] normal command for gesture '$gesture'");

          // Normal gesture command
          String newCommand = _mapGestureToCommand(gesture);
          setState(() {
            detectedGesture = gesture;
            robotCommand = newCommand;
            // Add command to history (keep last 50)
            commandHistory.insert(0, newCommand);
            if (commandHistory.length > 50) {
              commandHistory.removeLast();
            }
          });
          debugPrint("Detected Gesture: $gesture -> Command: $newCommand");
          _robotService.sendCommand(robotCommand);
        }
      }
    });

    try {
      await _robotService.connect();
    } catch (e) {
      debugPrint("Failed to connect to robot on view init: $e");
    }
  }

  Future<void> _playDance(DanceMove dance) async {
    debugPrint("🎬 [PLAY] ENTER _playDance for '${dance.name}'");
    debugPrint("🎬 [PLAY] steps count = ${dance.steps.length}");
    debugPrint("🎬 [PLAY] musicPath = ${dance.musicPath}");

    _isDancePlaying = true;

    // Start music
    if (dance.musicPath != null && dance.musicPath!.isNotEmpty) {
      await _musicController.play(dance.musicPath!);
      debugPrint("🎵 [MUSIC] started");
    }

    for (final step in dance.steps) {
      debugPrint(
        "🤖 [STEP] sending command '${step.movement}' for ${step.durationMs}ms",
      );

      if (!mounted || !_isDancePlaying) break;

      _robotService.sendCommand(step.movement);

      setState(() {
        robotCommand = step.movement;
      });

      await Future.delayed(Duration(milliseconds: step.durationMs));
    }

    // Stop at the end
    debugPrint("🛑 [PLAY] stopping robot & music");
    _robotService.sendCommand('stop');
    setState(() {
      robotCommand = 'stop';
    });

    // Stop music
    await _musicController.stop();
    debugPrint("🏁 [PLAY] FINISHED dance '${dance.name}'");
    _isDancePlaying = false;
  }

  String _mapGestureToCommand(String gesture) {
    switch (gesture.toLowerCase()) {
      case 'move forward':
        return 'forward';
      case 'move backward':
        return 'backward';
      case 'stop robot':
        return 'stop';
      case 'turn left':
        return 'left';
      case 'turn right':
        return 'right';
      default:
        return 'stop';
    }
  }

  @override
  void dispose() {
    // Clean up to prevent memory leaks
    _isDancePlaying = false;
    _gestureSubscription?.cancel();
    _obstacleSubscription?.cancel();
    _scrollController.dispose();
    _gestureService.dispose();
    _robotService.disconnect();
    _obstacleSensorService.dispose();
    _musicController.stop();
    super.dispose();
  }

  // PERMISSION LOGIC FOR ANDROID
  Future<void> _getCameraPermissionAndroid() async {
    try {
      final bool result = await cameraPermission.invokeMethod(
        'getCameraPermission',
      );
      if (mounted) {
        setState(() {
          isPermissionGranted = result;
        });
        if (result) {
          _initializeServices();
        }
      }
    } on PlatformException catch (e) {
      // Permission error
    }
  }

  @override
  Widget build(BuildContext context) {
    if (!isPermissionGranted) {
      return const SafeArea(
        child: Center(child: Text("Waiting for camera permission...")),
      );
    }

    return SingleChildScrollView(
      padding: const EdgeInsets.all(24.0),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const SizedBox(height: 20),
          const Text(
            "Position your hand in the camera view:",
            style: TextStyle(fontSize: 16, color: Colors.grey),
          ),
          const SizedBox(height: 20),
          Center(
            child: Stack(
              children: [
                Container(
                  height: 350,
                  width: double.infinity,
                  decoration: BoxDecoration(
                    border: Border.all(color: Colors.blue, width: 2),
                    borderRadius: BorderRadius.circular(24),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.black.withOpacity(0.1),
                        blurRadius: 10,
                        offset: const Offset(0, 5),
                      ),
                    ],
                  ),
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(22),
                    child: _gestureService.buildCameraView(),
                  ),
                ),
                if (_currentObstacleStatus?.blocked == true)
                  Positioned.fill(
                    child: Container(
                      decoration: BoxDecoration(
                        borderRadius: BorderRadius.circular(22),
                      ),
                      child: _buildObstacleWarnings(),
                    ),
                  ),
              ],
            ),
          ),
          const SizedBox(height: 20),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
            decoration: BoxDecoration(
              color: Colors.blue.withOpacity(0.1),
              borderRadius: BorderRadius.circular(16),
              border: Border.all(color: Colors.blue.withOpacity(0.3), width: 1),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  children: [
                    const Icon(Icons.pan_tool, color: Colors.blue),
                    const SizedBox(width: 12),
                    Expanded(
                      child: Text(
                        detectedGesture,
                        style: const TextStyle(
                          fontSize: 20,
                          fontWeight: FontWeight.bold,
                          color: Colors.blue,
                        ),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 16),
                const Divider(),
                const SizedBox(height: 8),
                Row(
                  children: [
                    const Icon(Icons.history, color: Colors.green, size: 20),
                    const SizedBox(width: 8),
                    const Text(
                      "Command History:",
                      style: TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.bold,
                        color: Colors.green,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 8),
                if (commandHistory.isEmpty)
                  const Padding(
                    padding: EdgeInsets.symmetric(vertical: 8.0),
                    child: Text(
                      "No commands yet...",
                      style: TextStyle(
                        fontSize: 14,
                        color: Colors.grey,
                        fontStyle: FontStyle.italic,
                      ),
                    ),
                  )
                else
                  Container(
                    constraints: const BoxConstraints(maxHeight: 150),
                    child: Scrollbar(
                      controller: _scrollController,
                      thumbVisibility: true,
                      child: ListView.builder(
                        controller: _scrollController,
                        shrinkWrap: true,
                        itemCount: commandHistory.length,
                        itemBuilder: (context, index) {
                          String command = commandHistory[index];
                          return Padding(
                            padding: const EdgeInsets.symmetric(vertical: 4.0),
                            child: Row(
                              children: [
                                Container(
                                  width: 24,
                                  height: 24,
                                  decoration: BoxDecoration(
                                    color: index == 0
                                        ? Colors.green
                                        : Colors.grey.withOpacity(0.3),
                                    shape: BoxShape.circle,
                                  ),
                                  child: Center(
                                    child: Text(
                                      "${index + 1}",
                                      style: TextStyle(
                                        fontSize: 12,
                                        fontWeight: FontWeight.bold,
                                        color: index == 0
                                            ? Colors.white
                                            : Colors.black54,
                                      ),
                                    ),
                                  ),
                                ),
                                const SizedBox(width: 12),
                                Text(
                                  command,
                                  style: TextStyle(
                                    fontSize: 16,
                                    fontWeight: index == 0
                                        ? FontWeight.bold
                                        : FontWeight.normal,
                                    color: index == 0
                                        ? Colors.green
                                        : Colors.black87,
                                  ),
                                ),
                              ],
                            ),
                          );
                        },
                      ),
                    ),
                  ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildObstacleWarnings() {
    if (_currentObstacleStatus == null) return const SizedBox.shrink();

    return Stack(
      children: [
        if (_currentObstacleStatus!.isSectorBlocked('center'))
          Positioned(
            top: 16,
            left: 0,
            right: 0,
            child: Center(
              child: _buildWarningBadge(
                'OBJECT IN FRONT',
                _currentObstacleStatus!.getDistance('center'),
              ),
            ),
          ),
        if (_currentObstacleStatus!.isSectorBlocked('behind'))
          Positioned(
            top: 16,
            left: 0,
            right: 0,
            child: Center(
              child: _buildWarningBadge(
                'OBJECT BEHIND',
                _currentObstacleStatus!.getDistance('behind'),
              ),
            ),
          ),
      ],
    );
  }

  Widget _buildWarningBadge(String text, double? distance) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
      decoration: BoxDecoration(
        color: Colors.red,
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: Colors.red.withOpacity(0.5),
            blurRadius: 8,
            spreadRadius: 2,
          ),
        ],
      ),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Icon(Icons.warning, color: Colors.white, size: 24),
              const SizedBox(width: 8),
              Text(
                text,
                style: const TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: 16,
                ),
              ),
            ],
          ),
          if (distance != null)
            Text(
              '${distance.toInt()} cm',
              style: const TextStyle(color: Colors.white, fontSize: 14),
            ),
        ],
      ),
    );
  }
}
