import 'package:speech_to_text/speech_to_text.dart';

class VoiceService {
  final SpeechToText _speech = SpeechToText();
  bool _isAvailable = false;

  Future<bool> initialize() async {
    _isAvailable = await _speech.initialize(
      onError: (e) => print("mic error: $e"),
      onStatus: (status) => print("mic status: $status"),
    );
    return _isAvailable;
  }

  Future<void> listen({required Function(String) onResult}) async {
    if (!_isAvailable) return;

    var systemLocale = await _speech.systemLocale();
    String localeId = 'en_US';

    await _speech.listen(
      onResult: (result) {
        if (result.finalResult) {
          onResult(result.recognizedWords);
        }
      },
      localeId: localeId,

      listenMode: ListenMode.deviceDefault,
      cancelOnError: true,
    );
  }

  Future<void> stop() async {
    await _speech.stop();
  }
}
