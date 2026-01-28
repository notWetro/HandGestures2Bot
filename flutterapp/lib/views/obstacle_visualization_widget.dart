import 'package:flutter/material.dart';
import '../models/obstacle_status.dart';

class ObstacleVisualizationWidget extends StatelessWidget {
  final ObstacleStatus obstacleStatus;

  const ObstacleVisualizationWidget({
    super.key,
    required this.obstacleStatus,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: obstacleStatus.blocked ? Colors.red.shade50 : Colors.green.shade50,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(
          color: obstacleStatus.blocked ? Colors.red : Colors.green,
          width: 2,
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(
                obstacleStatus.blocked ? Icons.warning : Icons.check_circle,
                color: obstacleStatus.blocked ? Colors.red : Colors.green,
                size: 32,
              ),
              const SizedBox(width: 12),
              Text(
                obstacleStatus.blocked ? 'BLOCKIERT' : 'Freie Fahrt',
                style: TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                  color: obstacleStatus.blocked ? Colors.red : Colors.green,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          _buildSensorRow(),
          const SizedBox(height: 8),
          Text(
            'Sicherheitsabstand: ${obstacleStatus.safetyDistanceCm.toInt()} cm',
            style: const TextStyle(fontSize: 12, color: Colors.grey),
          ),
        ],
      ),
    );
  }

  Widget _buildSensorRow() {
    return Column(
      children: [
        _buildSectorIndicator('center', 'Vorne'),
        const SizedBox(height: 16),
        _buildSectorIndicator('behind', 'Hinten'),
      ],
    );
  }

  Widget _buildSectorIndicator(String sector, String label) {
    final distance = obstacleStatus.getDistance(sector) ?? 999;
    final isBlocked = obstacleStatus.isSectorBlocked(sector);
    
    return Column(
      children: [
        Container(
          width: 80,
          height: 80,
          decoration: BoxDecoration(
            color: isBlocked ? Colors.red.shade300 : Colors.green.shade300,
            shape: BoxShape.circle,
            boxShadow: [
              BoxShadow(
                color: (isBlocked ? Colors.red : Colors.green).withOpacity(0.3),
                blurRadius: 8,
                spreadRadius: 2,
              ),
            ],
          ),
          child: Center(
            child: Text(
              '${distance.toInt()}\ncm',
              textAlign: TextAlign.center,
              style: const TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.bold,
                color: Colors.white,
              ),
            ),
          ),
        ),
        const SizedBox(height: 8),
        Text(
          label,
          style: TextStyle(
            fontSize: 14,
            fontWeight: isBlocked ? FontWeight.bold : FontWeight.normal,
            color: isBlocked ? Colors.red : Colors.black87,
          ),
        ),
      ],
    );
  }
}