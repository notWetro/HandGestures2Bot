import 'package:audioplayers/audioplayers.dart';

class DanceMusicController {
  static final DanceMusicController _instance =
      DanceMusicController._internal();
  factory DanceMusicController() => _instance;
  DanceMusicController._internal();

  final AudioPlayer _player = AudioPlayer();

  Future<void> play(String path) async {
    await _player.stop();
    await _player.play(DeviceFileSource(path));
  }

  Future<void> stop() async {
    await _player.stop();
  }
}
