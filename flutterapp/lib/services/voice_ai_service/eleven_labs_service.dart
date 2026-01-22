import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:audioplayers/audioplayers.dart';
import 'package:flutter/foundation.dart';

class ElevenLabsService {
  static const String _apiKey = "sk_18c0abf2b73bea48df7a2a7f3f27a395b2ad4f0ec1928575"; 
  static const String _baseUrl = "https://api.elevenlabs.io/v1/text-to-speech";
  
  final AudioPlayer _audioPlayer = AudioPlayer();

  Future<void> speak(String text, String voiceId) async {
    if (text.isEmpty) return;

    try {
      final url = Uri.parse("$_baseUrl/$voiceId");
      final response = await http.post(
        url,
        headers: {
          "xi-api-key": _apiKey,
          "Content-Type": "application/json",
          "Accept": "audio/mpeg",
        },
        body: jsonEncode({
          "text": text,
          "model_id": "eleven_multilingual_v2",
          "voice_settings": {
            "stability": 0.5,
            "similarity_boost": 0.75,
          }
        }),
      );

      if (response.statusCode == 200) {
        final bytes = response.bodyBytes;
        await _audioPlayer.play(BytesSource(bytes));
      } else {
        debugPrint("ElevenLabs Error: ${response.statusCode} - ${response.body}");
      }
    } catch (e) {
      debugPrint("ElevenLabs Exception: $e");
    }
  }
}