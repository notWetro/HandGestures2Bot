import 'package:flutter_gemma/flutter_gemma.dart';

class AiService {
  InferenceModel? _brain;

  static const String _systemPrompt = '''
  You are a Robot.

  INSTRUCTIONS:
  1. Analyze the user's voice command.
  2. Output a JSON object with two fields:
     - "direction": The movement (forward, backward, left, right, stop).
     - "response": A short (max 5 words), somewhat humorous reply to the user.
  ''';

  Future<void> initialize() async {
    await FlutterGemma.installModel(
      modelType: ModelType.gemmaIt,
    ).fromAsset('assets/llm/gemma3-1B-it-int4.task').install();

    _brain = await FlutterGemma.getActiveModel(
      maxTokens: 4096,
      preferredBackend: PreferredBackend.cpu,
    );
  }

  Future<String> sendCommand(String text, {String? personaName}) async {
    if (_brain == null) return '{"error": "AI not ready"}';

    try {
      final session = await _brain!.createChat(
        temperature: 0.8,
        randomSeed: DateTime.now().millisecondsSinceEpoch,
        topK: 40,
      );

      String currentPrompt = _systemPrompt;
      if (personaName != null) {
        currentPrompt += "\n  3. The 'response' text MUST be in the style/personality of $personaName.";
      }

      // Combine personality + user command
      final fullPrompt = '$currentPrompt\nUser: "$text"\nAI:';
      
      // Send to brain
      await session.addQueryChunk(Message.text(text: fullPrompt, isUser: true));
      final result = await session.generateChatResponse();

      if (result is TextResponse) {
        final text = result.token;
        final startIndex = text.indexOf('{');
        final endIndex = text.lastIndexOf('}');
        if (startIndex != -1 && endIndex != -1) {
          return text.substring(startIndex, endIndex + 1);
        }
        return text;
      }
      return '{"error": "Unknown response type"}';
    } catch (e) {
      print("AI Error: $e");

      if (e.toString().contains("maxTokens")) {
        await initialize();
        return '{"direction": "stop", "response": "Brain rebooting..."}';
      }
      return '{"error": "$e"}';
    }
  }
}
