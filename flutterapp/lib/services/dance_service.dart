import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import '../models/dance_move.dart';

class DanceService {
  static final DanceService _instance = DanceService._internal();
  factory DanceService() => _instance;
  DanceService._internal();

  static const MethodChannel _channel = MethodChannel('dance_channel');

  Future<List<DanceMove>> loadDanceMoves() async {
    try {
      final String? jsonString = await _channel.invokeMethod('loadDanceMoves');

      if (jsonString == null || jsonString.isEmpty) {
        return [];
      }

      final List<dynamic> jsonList = jsonDecode(jsonString);
      return jsonList.map((json) => DanceMove.fromJson(json)).toList();
    } on PlatformException catch (e) {
      debugPrint('Error loading dance moves: ${e.message}');
      return [];
    } catch (e) {
      debugPrint('Error loading dance moves: $e');
      return [];
    }
  }

  Future<void> saveDanceMoves(List<DanceMove> dances) async {
    try {
      final String jsonString = jsonEncode(
        dances.map((d) => d.toJson()).toList(),
      );
      await _channel.invokeMethod('saveDanceMoves', jsonString);
    } on PlatformException catch (e) {
      debugPrint('Error saving dance moves: ${e.message}');
    } catch (e) {
      debugPrint('Error saving dance moves: $e');
    }
  }

  Future<void> saveDanceMove(DanceMove dance) async {
    final dances = await loadDanceMoves();
    final index = dances.indexWhere((d) => d.id == dance.id);

    if (index >= 0) {
      dances[index] = dance;
    } else {
      dances.add(dance);
    }

    await saveDanceMoves(dances);
  }

  Future<void> deleteDanceMove(String id) async {
    final dances = await loadDanceMoves();
    dances.removeWhere((d) => d.id == id);
    await saveDanceMoves(dances);
  }

  Future<DanceMove?> getDanceByGesture(String gesture) async {
  final dances = await loadDanceMoves();

  debugPrint("🧪 [DANCES COUNT] ${dances.length}");
  for (final d in dances) {
    debugPrint(
      "🧾 Dance='${d.name}', assignedGesture='${d.assignedGesture}'",
    );
  }

  final normalized = gesture.trim().toLowerCase();

  try {
    return dances.firstWhere(
      (d) =>
          d.assignedGesture != null &&
          d.assignedGesture!.trim().toLowerCase() == normalized,
    );
  } catch (e) {
    debugPrint("❌ [NO MATCH] for '$gesture'");
    return null;
  }
}

}
