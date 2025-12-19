import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import '../models/dance_move.dart';

class DanceService {
  static final DanceService _instance = DanceService._internal();
  factory DanceService() => _instance;
  DanceService._internal();

  static const String _storageKey = 'saved_dance_moves';

  Future<List<DanceMove>> loadDanceMoves() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final String? jsonString = prefs.getString(_storageKey);
      
      if (jsonString == null || jsonString.isEmpty) {
        return [];
      }

      final List<dynamic> jsonList = jsonDecode(jsonString);
      return jsonList.map((json) => DanceMove.fromJson(json)).toList();
    } catch (e) {
      debugPrint('Error loading dance moves: $e');
      return [];
    }
  }

  Future<void> saveDanceMoves(List<DanceMove> dances) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final String jsonString = jsonEncode(dances.map((d) => d.toJson()).toList());
      await prefs.setString(_storageKey, jsonString);
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
    try {
      return dances.firstWhere((d) => d.assignedGesture == gesture);
    } catch (e) {
      return null;
    }
  }
}
