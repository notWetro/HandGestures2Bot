import 'package:flutter_tts/flutter_tts.dart';

class TtsService {
  final FlutterTts _tts = FlutterTts();

  Future<void> initialize() async {
    // Wait for the engine to load
    await _tts.awaitSpeakCompletion(true);
    
    // Set Language 
    await _tts.setLanguage("en-US");
    
    // Adjust Voice Properties
    await _tts.setPitch(1.0); 
    await _tts.setSpeechRate(0.5);
  }

  Future<void> speak(String text) async {
    if (text.isNotEmpty) {
      await _tts.speak(text);
    }
  }

  Future<void> stop() async {
    await _tts.stop();
  }
}