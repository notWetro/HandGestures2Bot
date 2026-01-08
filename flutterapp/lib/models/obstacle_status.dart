class ObstacleStatus {
  final bool blocked;
  final List<String> sectors;
  final Map<String, double> distances;
  final double safetyDistanceCm;

  ObstacleStatus({
    required this.blocked,
    required this.sectors,
    required this.distances,
    required this.safetyDistanceCm,
  });

  factory ObstacleStatus.fromJson(Map<String, dynamic> json) {
    return ObstacleStatus(
      blocked: json['blocked'] as bool,
      sectors: List<String>.from(json['sectors'] as List),
      distances: Map<String, double>.from(
        (json['distances'] as Map).map(
          (key, value) => MapEntry(key.toString(), (value as num).toDouble()),
        ),
      ),
      safetyDistanceCm: (json['safety_distance_cm'] as num).toDouble(),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'blocked': blocked,
      'sectors': sectors,
      'distances': distances,
      'safety_distance_cm': safetyDistanceCm,
    };
  }

  bool isSectorBlocked(String sector) {
    return sectors.contains(sector);
  }

  double? getDistance(String sector) {
    return distances[sector];
  }
}
