library eleven_labs_service;

import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:audioplayers/audioplayers.dart';
import 'package:flutter/foundation.dart';

/// A service responsible for interacting with the ElevenLabs Text-to-Speech API.
///
/// This class handles the HTTP POST request to ElevenLabs and plays the 
/// resulting audio stream using [AudioPlayer].
class ElevenLabsService {
  /// The API key for ElevenLabs authentication.
  /// 
  static const String apiKey = "sk_d63b1fb3f34df3f3a2631a94b62135b39235c8e2f8d467a3"; 
  
  /// The base URL for the ElevenLabs v1 TTS endpoint.
  static const String baseUrl = "https://api.elevenlabs.io/v1/text-to-speech";
  
  /// Internal player instance used to output the generated speech.
  final AudioPlayer audioPlayer = AudioPlayer();

  /// Converts [text] into speech using the specified [voiceId].
  ///
  /// This method performs the following steps:
  /// - Sends a request to ElevenLabs using the `eleven_multilingual_v2` model.
  /// - Validates the [response.statusCode].
  /// - If successful (200 OK), plays the audio immediately.
  /// 
  /// [text] - The string content to be spoken. If empty, the function returns early.
  /// [voiceId] - The unique identifier for the ElevenLabs voice.
  ///
  /// Throws an error log to the console if the request fails.
  Future<void> speak(String text, String voiceId) async {
    if (text.isEmpty) return;

    try {
      final url = Uri.parse("$baseUrl/$voiceId");
      final response = await http.post(
        url,
        headers: {
          "xi-api-key": apiKey,
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
        await audioPlayer.play(BytesSource(bytes));
      } else {
        debugPrint("ElevenLabs Error: ${response.statusCode} - ${response.body}");
      }
    } catch (e) {
      debugPrint("ElevenLabs Exception: $e");
    }
  }
}