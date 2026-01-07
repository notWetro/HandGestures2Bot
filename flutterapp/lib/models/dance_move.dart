class DanceMove {
  final String id;
  final String name;
  final List<DanceStep> steps;
  String? assignedGesture;
  String? musicPath;
  List<double>? triggerGesture;

  DanceMove({
    required this.id,
    required this.name,
    required this.steps,
    this.assignedGesture,
    this.musicPath,
    this.triggerGesture,
  });

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'name': name,
      'steps': steps.map((s) => s.toJson()).toList(),
      'assignedGesture': assignedGesture,
      'musicPath': musicPath,
      'triggerGesture': triggerGesture,
    };
  }

  factory DanceMove.fromJson(Map<String, dynamic> json) {
    return DanceMove(
      id: json['id'] as String,
      name: json['name'] as String,
      steps: (json['steps'] as List)
          .map((s) => DanceStep.fromJson(s as Map<String, dynamic>))
          .toList(),
      assignedGesture: json['assignedGesture'] as String?,
      musicPath: json['musicPath'] as String?,
      triggerGesture: json['triggerGesture'] != null
          ? List<double>.from(json['triggerGesture'])
          : null,
    );
  }

  DanceMove copyWith({
    String? id,
    String? name,
    List<DanceStep>? steps,
    String? assignedGesture,
    String? musicPath,
    List<double>? triggerGesture,
  }) {
    return DanceMove(
      id: id ?? this.id,
      name: name ?? this.name,
      steps: steps ?? this.steps,
      assignedGesture: assignedGesture ?? this.assignedGesture,
      musicPath: musicPath ?? this.musicPath,
      triggerGesture: triggerGesture ?? this.triggerGesture,
    );
  }
}

class DanceStep {
  final String movement; // 'forward', 'left', 'right', 'stop'
  final int durationMs;

  DanceStep({
    required this.movement,
    required this.durationMs,
  });

  Map<String, dynamic> toJson() {
    return {
      'movement': movement,
      'durationMs': durationMs,
    };
  }

  factory DanceStep.fromJson(Map<String, dynamic> json) {
    return DanceStep(
      movement: json['movement'] as String,
      durationMs: json['durationMs'] as int,
    );
  }
}
