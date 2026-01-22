import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter_gemma/flutter_gemma.dart';
import 'package:flutterapp/services/voice_ai_service/ai_service.dart';
import 'package:flutterapp/services/voice_ai_service/voice_service.dart';
import 'package:flutterapp/services/voice_ai_service/tts_service.dart';
import 'package:flutterapp/services/voice_ai_service/eleven_labs_service.dart';
import 'package:flutterapp/services/robot_service.dart';

class VoiceView extends StatefulWidget {
  const VoiceView({super.key});

  @override
  State<VoiceView> createState() => _VoiceViewState();
}

class _VoiceViewState extends State<VoiceView> {
  // Services
  final AiService _aiService = AiService();
  final VoiceService _voiceService = VoiceService();
  final TtsService _ttsService = TtsService();
  final ElevenLabsService _elevenLabsService = ElevenLabsService();
  final RobotService _robotService = RobotService();

  // State
  String _status = "Booting up...";
  String _userSpeech = "Press Mic to Speak";
  String _aiResponse = "{}";
  bool _isListening = false;
  bool _canSpeak = false;

  // ElevenLabs Voices (Name : ID)
  final Map<String, String> _voices = {
    'Micky Mouse (male)': 'mdzEgLpu0FjTwYs5oot0', 
    'Micky Mouse (female)': 'eppqEXVumQ3CfdndcIBd',
    'Evil': 'PSkrmGGNwoOIKXqzUWs9',
  };
  String _selectedVoiceName = 'Micky Mouse (male)';

  @override
  void initState() {
    super.initState();
    _initializeSystem();
  }

  Future<void> _initializeSystem() async {
    if (mounted) setState(() => _status = "Initializing AI...");

    try {
      await FlutterGemma.initialize();
    } catch (e) {
      debugPrint("Gemma init warning: $e");
    }

    bool aiReady = false;
    try {
      await _aiService.initialize();
      aiReady = true;
    } catch (e) {
      debugPrint("AI Service Init Error: $e");
      if (mounted) setState(() => _aiResponse = "AI Model Error: Check logs");
    }

    if (mounted) setState(() => _status = "Starting Voice Services...");
    await _ttsService.initialize();

    final micReady = await _voiceService.initialize();

<<<<<<< HEAD
    if (mounted)
      setState(() => _status = "Connecting to Robot... (!! TO IMPLEMENT !!)");
=======
    if (mounted) setState(() => _status = "Connecting to Robot... ");
>>>>>>> c5d28231397c684aab004f2c3928f3ba4947fe1f
    try {
      await _robotService.connect().timeout(const Duration(seconds: 2));
    } catch (e) {
      // Ignore error
    }

    if (mounted) {
      setState(() {
        if (!aiReady) {
          _status = "AI MODEL MISSING";
          _canSpeak = false;
        } else {
          _status = micReady ? "SYSTEM READY" : "MIC ERROR";
          _canSpeak = micReady;
        }
      });
    }

    _ttsService.speak("System Online");
  }

  @override
  void dispose() {
    _voiceService.stop();
    _ttsService.stop();
    super.dispose();
  }

  Future<void> _toggleListening() async {
    if (_isListening) {
      await _voiceService.stop();
      if (mounted) setState(() => _isListening = false);
    } else {
      if (mounted) {
        setState(() {
          _isListening = true;
          _userSpeech = "Listening...";
          _aiResponse = "...";
        });
      }

      await _voiceService.listen(
        onResult: (text) async {
          if (!mounted) return;

<<<<<<< HEAD
=======
        setState(() {
          _userSpeech = text;
          _isListening = false;
          _status = "🤔 Processing...";
        });

        // Get JSON String from AI
        final jsonString = await _aiService.sendCommand(text, personaName: _selectedVoiceName);
        
        if (mounted) {
>>>>>>> c5d28231397c684aab004f2c3928f3ba4947fe1f
          setState(() {
            _userSpeech = text;
            _isListening = false;
            _status = "Processing...";
          });

          // Get JSON String from AI
          final jsonString = await _aiService.sendCommand(text);

          if (mounted) {
            setState(() {
              _aiResponse = jsonString;
              _status = "COMMAND EXECUTED";
            });
          }

          // Speak the Action
          _speakAction(jsonString);
        },
      );
    }
  }

  // Logic: Convert JSON -> Voice & Robot Command
  void _speakAction(String jsonString) {
    try {
      // Find the JSON part
      final startIndex = jsonString.indexOf('{');
      final endIndex = jsonString.lastIndexOf('}');

      if (startIndex != -1 && endIndex != -1) {
        final cleanJson = jsonString.substring(startIndex, endIndex + 1);
        final data = jsonDecode(cleanJson);

        // Check for the "response" field and speak it
        if (data.containsKey('response')) {
          String aiSpeech = data['response'];
          debugPrint("AI Says: $aiSpeech");
<<<<<<< HEAD
          _ttsService.speak(aiSpeech);
        }

=======
          _elevenLabsService.speak(aiSpeech, _voices[_selectedVoiceName]!);
        } 
        
>>>>>>> c5d28231397c684aab004f2c3928f3ba4947fe1f
        // Check for "direction" and move robot
        if (data.containsKey('direction')) {
          String direction = data['direction'];
          debugPrint("Robot Command: $direction");
          _robotService.sendCommand(direction);

          // Fallback speech if no response field
          if (!data.containsKey('response')) {
            _ttsService.speak("Moving $direction");
          }
        }
      }
    } catch (e) {
      debugPrint("JSON Parse Error: $e");
      _ttsService.speak("I'm confused.");
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(
        title: const Text("Voice Control"),
        backgroundColor: Colors.grey[900],
        foregroundColor: Colors.white,
      ),
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
<<<<<<< HEAD
            Text(
              _status,
              style: const TextStyle(
                color: Colors.greenAccent,
                fontWeight: FontWeight.bold,
=======
            Text(_status, style: const TextStyle(color: Colors.greenAccent, fontWeight: FontWeight.bold)),
            const SizedBox(height: 10),
            // Voice Selector Dropdown
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 12),
              decoration: BoxDecoration(
                color: Colors.grey[800],
                borderRadius: BorderRadius.circular(8),
              ),
              child: DropdownButtonHideUnderline(
                child: DropdownButton<String>(
                  value: _selectedVoiceName,
                  dropdownColor: Colors.grey[800],
                  style: const TextStyle(color: Colors.white),
                  icon: const Icon(Icons.arrow_drop_down, color: Colors.white),
                  items: _voices.keys.map((String name) {
                    return DropdownMenuItem<String>(
                      value: name,
                      child: Text("$name (ElevenLabs)"),
                    );
                  }).toList(),
                  onChanged: (String? newValue) {
                    if (newValue != null) {
                      setState(() => _selectedVoiceName = newValue);
                    }
                  },
                ),
>>>>>>> c5d28231397c684aab004f2c3928f3ba4947fe1f
              ),
            ),
            const Spacer(),
            _buildChatBubble("USER", _userSpeech, Colors.blue),
            const SizedBox(height: 20),
            _buildChatBubble("ROBOT", _aiResponse, Colors.green),
            const Spacer(),
            GestureDetector(
              onTap: _canSpeak ? _toggleListening : null,
              child: AnimatedContainer(
                duration: const Duration(milliseconds: 200),
                height: 80,
                width: 80,
                decoration: BoxDecoration(
                  color: _canSpeak
                      ? (_isListening ? Colors.red : Colors.green)
                      : Colors.grey,
                  shape: BoxShape.circle,
                  boxShadow: [
                    BoxShadow(
                      color:
                          (_canSpeak
                                  ? (_isListening ? Colors.red : Colors.green)
                                  : Colors.grey)
                              .withOpacity(0.5),
                      blurRadius: 20,
                      spreadRadius: 5,
                    ),
                  ],
                ),
                child: Icon(
                  _isListening ? Icons.stop : Icons.mic,
                  color: Colors.white,
                  size: 40,
                ),
              ),
            ),
            const SizedBox(height: 30),
            if (_canSpeak)
              const Text("Tap to Speak", style: TextStyle(color: Colors.grey)),
          ],
        ),
      ),
    );
  }

  Widget _buildChatBubble(String label, String text, Color color) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(label, style: TextStyle(color: color, fontSize: 12)),
        const SizedBox(height: 5),
        Container(
          width: double.infinity,
          padding: const EdgeInsets.all(15),
          decoration: BoxDecoration(
            color: Colors.grey[900],
            borderRadius: BorderRadius.circular(10),
            border: Border.all(color: color.withOpacity(0.5)),
          ),
          child: Text(
            text,
            style: const TextStyle(
              color: Colors.white,
              fontFamily: 'monospace',
            ),
          ),
        ),
      ],
    );
  }
}
