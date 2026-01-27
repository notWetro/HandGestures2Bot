import 'package:flutter/material.dart';
import 'package:flutterapp/services/gesture_service.dart';
import 'package:flutterapp/services/dance_service.dart';
import 'package:flutterapp/models/dance_move.dart';
import 'package:flutterapp/views/settings_pages/save_gesture_view.dart';
import 'package:flutterapp/views/settings_pages/create_dance_view.dart';

class HandGesturesPage extends StatefulWidget {
  const HandGesturesPage({super.key});

  @override
  State<HandGesturesPage> createState() => _HandGesturesPageState();
}

class _HandGesturesPageState extends State<HandGesturesPage> {
  // The Fixed List of Commands the Robot understands
  final List<String> _robotActions = [
    'Stop Robot',
    'Move Forward',
    'Move Backward',
    'Turn Left',
    'Turn Right',
  ];

  // Service to handle native communication
  final GestureService _gestureService = GestureService();
  final DanceService _danceService = DanceService();

  // State variables
  List<String> _savedGestures = [];
  List<DanceMove> _dances = [];
  bool _isLoading = true;

  @override
  void initState() {
    super.initState();
    _fetchGestures();
    _fetchDances();
  }

  // Ask the service for the list of gestures
  Future<void> _fetchGestures() async {
    try {
      // Use the service instead of a direct channel call
      final List<String> result = await _gestureService.getGestureList();
      if (mounted) {
        setState(() {
          _savedGestures = result;
          _isLoading = false;
        });
      }
    } catch (e) {
      debugPrint("Error fetching gestures: $e");
      if (mounted) {
        setState(() {
          _isLoading = false;
        });
      }
    }
  }

  // Fetch dance moves
  Future<void> _fetchDances() async {
    try {
      final dances = await _danceService.loadDanceMoves();
      if (mounted) {
        setState(() {
          _dances = dances;
        });
      }
    } catch (e) {
      debugPrint("Error fetching dances: $e");
    }
  }

  // Open camera to record a SPECIFIC action
  void _recordAction(String actionName) async {
    await Navigator.push(
      context,
      MaterialPageRoute(
        builder: (context) => SaveGestureView(
          // We pass the Action Name as the Gesture Name.
          // This forces the app to save it as "Turn Left" etc.
          currentGesture: actionName,
          actionName: actionName,
        ),
      ),
    );

    // Refresh list to show the green checkmark
    _fetchGestures();
  }

  // Create or edit a dance
  void _createOrEditDance([DanceMove? existingDance]) async {
    final result = await Navigator.push<DanceMove>(
      context,
      MaterialPageRoute(
        builder: (context) => CreateDanceView(existingDance: existingDance),
      ),
    );

    if (result != null) {
      await _danceService.saveDanceMove(result);
      _fetchDances();
    }
  }

  // Delete a dance
  void _deleteDance(DanceMove dance) async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Delete Dance'),
        content: Text('Are you sure you want to delete "${dance.name}"?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text('Cancel'),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, true),
            style: TextButton.styleFrom(foregroundColor: Colors.red),
            child: const Text('Delete'),
          ),
        ],
      ),
    );

    if (confirmed != true) return;

  // Gesture mit löschen
  if (dance.assignedGesture != null) {
    await _gestureService.deleteGesture(dance.assignedGesture!);
  }

  // Dance löschen
  await _danceService.deleteDanceMove(dance.id);

  _fetchDances();
  _fetchGestures();
  }

  // Assign gesture to dance
  void _assignGestureToDance(DanceMove dance) async {
    final gesture = await showDialog<String>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Assign Gesture'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Text('Select a gesture to trigger this dance:'),
            const SizedBox(height: 16),
            ..._savedGestures.map((gesture) {
              return ListTile(
                title: Text(gesture),
                trailing: dance.assignedGesture == gesture
                    ? const Icon(Icons.check, color: Colors.green)
                    : null,
                onTap: () => Navigator.pop(context, gesture),
              );
            }).toList(),
          ],
        ),
        actions: [
          if (dance.assignedGesture != null)
            TextButton(
              onPressed: () => Navigator.pop(context, ''),
              child: const Text('Remove Assignment'),
            ),
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Cancel'),
          ),
        ],
      ),
    );

    if (gesture != null) {
      final updatedDance = dance.copyWith(
        assignedGesture: gesture.isEmpty ? null : gesture,
      );
      await _danceService.saveDanceMove(updatedDance);
      _fetchDances();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Robot Commands'),
        backgroundColor: Colors.orange,
        foregroundColor: Colors.white,
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Command Configuration',
              style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 10),
            const Text(
              'Teach the app how to recognize these specific commands.',
              style: TextStyle(fontSize: 16, color: Colors.grey),
            ),
            const SizedBox(height: 20),
            Expanded(
              child: _isLoading
                  ? const Center(child: CircularProgressIndicator())
                  : SingleChildScrollView(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          ListView.builder(
                            shrinkWrap: true,
                            physics: const NeverScrollableScrollPhysics(),
                            itemCount: _robotActions.length,
                            itemBuilder: (context, index) {
                              final action = _robotActions[index];

                              // Check if this action is already learned
                              final isConfigured = _savedGestures.contains(
                                action,
                              );

                              return Card(
                                elevation: isConfigured ? 2 : 0,
                                color: isConfigured
                                    ? Colors.white
                                    : Colors.grey[100],
                                margin: const EdgeInsets.only(bottom: 12),
                                shape: RoundedRectangleBorder(
                                  side: BorderSide(
                                    color: isConfigured
                                        ? Colors.green.shade200
                                        : Colors.transparent,
                                    width: 1,
                                  ),
                                  borderRadius: BorderRadius.circular(12),
                                ),
                                child: ListTile(
                                  leading: CircleAvatar(
                                    backgroundColor: isConfigured
                                        ? Colors.green.shade100
                                        : Colors.grey.shade300,
                                    child: Icon(
                                      isConfigured
                                          ? Icons.check
                                          : Icons.question_mark,
                                      color: isConfigured
                                          ? Colors.green
                                          : Colors.grey,
                                    ),
                                  ),
                                  title: Text(
                                    action,
                                    style: TextStyle(
                                      fontSize: 18,
                                      fontWeight: FontWeight.bold,
                                      color: isConfigured
                                          ? Colors.black
                                          : Colors.grey[700],
                                    ),
                                  ),
                                  subtitle: Text(
                                    isConfigured
                                        ? "Gesture Recorded"
                                        : "Not Configured (Empty)",
                                    style: TextStyle(
                                      color: isConfigured
                                          ? Colors.green
                                          : Colors.redAccent,
                                      fontWeight: FontWeight.w500,
                                    ),
                                  ),
                                  trailing: ElevatedButton.icon(
                                    style: ElevatedButton.styleFrom(
                                      backgroundColor: isConfigured
                                          ? Colors.white
                                          : Colors.blue,
                                      foregroundColor: isConfigured
                                          ? Colors.blue
                                          : Colors.white,
                                      elevation: isConfigured ? 0 : 2,
                                      side: isConfigured
                                          ? const BorderSide(color: Colors.blue)
                                          : null,
                                    ),
                                    icon: const Icon(
                                      Icons.camera_alt,
                                      size: 18,
                                    ),
                                    label: Text(
                                      isConfigured ? "Edit" : "Record",
                                    ),
                                    onPressed: () => _recordAction(action),
                                  ),
                                ),
                              );
                            },
                          ),
                          const SizedBox(height: 32),
                          const Divider(thickness: 2),
                          const SizedBox(height: 16),
                          Row(
                            children: [
                              const Text(
                                'Dance Moves',
                                style: TextStyle(
                                  fontSize: 24,
                                  fontWeight: FontWeight.bold,
                                ),
                              ),
                              const Spacer(),
                              ElevatedButton.icon(
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: Colors.purple,
                                  foregroundColor: Colors.white,
                                ),
                                icon: const Icon(Icons.add),
                                label: const Text('Add Dance'),
                                onPressed: () => _createOrEditDance(),
                              ),
                            ],
                          ),
                          const SizedBox(height: 10),
                          const Text(
                            'Create dance sequences that can be triggered by gestures.',
                            style: TextStyle(fontSize: 16, color: Colors.grey),
                          ),
                          const SizedBox(height: 20),
                          if (_dances.isEmpty)
                            Center(
                              child: Padding(
                                padding: const EdgeInsets.all(32.0),
                                child: Column(
                                  children: [
                                    Icon(
                                      Icons.music_note,
                                      size: 64,
                                      color: Colors.grey[400],
                                    ),
                                    const SizedBox(height: 16),
                                    Text(
                                      'No dance moves created yet',
                                      style: TextStyle(
                                        fontSize: 16,
                                        color: Colors.grey[600],
                                      ),
                                    ),
                                  ],
                                ),
                              ),
                            )
                          else
                            ListView.builder(
                              shrinkWrap: true,
                              physics: const NeverScrollableScrollPhysics(),
                              itemCount: _dances.length,
                              itemBuilder: (context, index) {
                                final dance = _dances[index];
                                final hasGesture =
                                    dance.assignedGesture != null;

                                return Card(
                                  elevation: 2,
                                  margin: const EdgeInsets.only(bottom: 12),
                                  shape: RoundedRectangleBorder(
                                    side: BorderSide(
                                      color: hasGesture
                                          ? Colors.purple.shade200
                                          : Colors.grey.shade300,
                                      width: 1,
                                    ),
                                    borderRadius: BorderRadius.circular(12),
                                  ),
                                  child: ListTile(
                                    leading: CircleAvatar(
                                      backgroundColor: Colors.purple.shade100,
                                      child: const Icon(
                                        Icons.music_note,
                                        color: Colors.purple,
                                      ),
                                    ),
                                    title: Text(
                                      dance.name,
                                      style: const TextStyle(
                                        fontSize: 18,
                                        fontWeight: FontWeight.bold,
                                      ),
                                    ),
                                    subtitle: Column(
                                      crossAxisAlignment:
                                          CrossAxisAlignment.start,
                                      children: [
                                        Text('${dance.steps.length} steps'),
                                        if (hasGesture)
                                          Text(
                                            'Gesture: ${dance.assignedGesture}',
                                            style: const TextStyle(
                                              color: Colors.purple,
                                              fontWeight: FontWeight.w500,
                                            ),
                                          )
                                        else
                                          const Text(
                                            'No gesture assigned',
                                            style: TextStyle(
                                              color: Colors.orange,
                                              fontStyle: FontStyle.italic,
                                            ),
                                          ),
                                      ],
                                    ),
                                    trailing: PopupMenuButton<String>(
                                      onSelected: (value) {
                                        switch (value) {
                                          case 'edit':
                                            _createOrEditDance(dance);
                                            break;
                                          case 'assign':
                                            _assignGestureToDance(dance);
                                            break;
                                          case 'delete':
                                            _deleteDance(dance);
                                            break;
                                        }
                                      },
                                      itemBuilder: (context) => [
                                        const PopupMenuItem(
                                          value: 'edit',
                                          child: Row(
                                            children: [
                                              Icon(Icons.edit, size: 20),
                                              SizedBox(width: 8),
                                              Text('Edit'),
                                            ],
                                          ),
                                        ),
                                        const PopupMenuItem(
                                          value: 'assign',
                                          child: Row(
                                            children: [
                                              Icon(Icons.gesture, size: 20),
                                              SizedBox(width: 8),
                                              Text('Assign Gesture'),
                                            ],
                                          ),
                                        ),
                                        const PopupMenuItem(
                                          value: 'delete',
                                          child: Row(
                                            children: [
                                              Icon(
                                                Icons.delete,
                                                size: 20,
                                                color: Colors.red,
                                              ),
                                              SizedBox(width: 8),
                                              Text(
                                                'Delete',
                                                style: TextStyle(
                                                  color: Colors.red,
                                                ),
                                              ),
                                            ],
                                          ),
                                        ),
                                      ],
                                    ),
                                  ),
                                );
                              },
                            ),
                        ],
                      ),
                    ),
            ),
          ],
        ),
      ),
    );
  }
}
