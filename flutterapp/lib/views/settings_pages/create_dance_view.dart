import 'package:flutter/material.dart';
import '../../models/dance_move.dart';
import 'package:file_picker/file_picker.dart'; 

class CreateDanceView extends StatefulWidget {
  final DanceMove? existingDance;

  const CreateDanceView({super.key, this.existingDance});

  @override
  State<CreateDanceView> createState() => _CreateDanceViewState();
}

class _CreateDanceViewState extends State<CreateDanceView> {
  final TextEditingController _nameController = TextEditingController();
  final List<DanceStep> _steps = [];
  String? _selectedMusicPath;
  String? _selectedMusicName;

  
  final List<String> _availableMovements = [
    'forward',
    'left', 
    'right',
    'stop',
  ];

  @override
  void initState() {
    super.initState();
    if (widget.existingDance != null) {
      _nameController.text = widget.existingDance!.name;
      _steps.addAll(widget.existingDance!.steps);
    }
  }

  @override
  void dispose() {
    _nameController.dispose();
    super.dispose();
  }

  void _addStep(String movement) {
    setState(() {
      _steps.add(DanceStep(movement: movement, durationMs: 1000));
    });
  }

  void _removeStep(int index) {
    setState(() {
      _steps.removeAt(index);
    });
  }

  void _updateStepDuration(int index, int newDuration) {
    setState(() {
      _steps[index] = DanceStep(
        movement: _steps[index].movement,
        durationMs: newDuration,
      );
    });
  }

  void _moveStepUp(int index) {
    if (index > 0) {
      setState(() {
        final step = _steps.removeAt(index);
        _steps.insert(index - 1, step);
      });
    }
  }

  void _moveStepDown(int index) {
    if (index < _steps.length - 1) {
      setState(() {
        final step = _steps.removeAt(index);
        _steps.insert(index + 1, step);
      });
    }
  }

  Future<void> pickMp3() async {
  try {
    debugPrint("📂 Öffne MP3 Picker");

    final result = await FilePicker.platform.pickFiles(
      type: FileType.custom,
      allowedExtensions: ['mp3'],
      allowMultiple: false,
    );

    if (result == null) {
      debugPrint("❌ Picker abgebrochen");
      return;
    }

    final file = result.files.single;

    setState(() {
      _selectedMusicPath = file.path;
      _selectedMusicName = file.name;
    });

    debugPrint("✅ MP3 gewählt: ${file.name}");
    debugPrint("📍 Pfad: ${file.path}");
  } catch (e) {
    debugPrint("🔥 Picker Fehler: $e");
  }
}


  void _saveDance() {
    if (_nameController.text.trim().isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('Please enter a dance name')),
      );
      return;
    }

    if (_steps.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('Please add at least one step')),
      );
      return;
    }

    final dance = DanceMove(
      id: widget.existingDance?.id ?? DateTime.now().millisecondsSinceEpoch.toString(),
      name: _nameController.text.trim(),
      steps: _steps,
      assignedGesture: widget.existingDance?.assignedGesture,
      musicPath: _selectedMusicPath,
    );

    Navigator.pop(context, dance);
  }

  IconData _getIconForMovement(String movement) {
    switch (movement) {
      case 'forward':
        return Icons.arrow_upward;
      case 'left':
        return Icons.arrow_back;
      case 'right':
        return Icons.arrow_forward;
      case 'stop':
        return Icons.stop;
      default:
        return Icons.help;
    }
  }

  Color _getColorForMovement(String movement) {
    switch (movement) {
      case 'forward':
        return Colors.blue;
      case 'left':
        return Colors.orange;
      case 'right':
        return Colors.green;
      case 'stop':
        return Colors.red;
      default:
        return Colors.grey;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.existingDance != null ? 'Edit Dance' : 'Create Dance'),
        backgroundColor: Colors.purple,
        foregroundColor: Colors.white,
        actions: [
          IconButton(
            icon: const Icon(Icons.save),
            onPressed: _saveDance,
          ),
        ],
      ),
      body: Column(
        children: [
          Padding(
            padding: const EdgeInsets.all(16.0),
            child: TextField(
              controller: _nameController,
              decoration: InputDecoration(
                labelText: 'Dance Name',
                border: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(12),
                ),
                prefixIcon: const Icon(Icons.music_note),
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16.0),
            child: GestureDetector(
              onTap: () async {
                await pickMp3();
              },
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
                decoration: BoxDecoration(
                  borderRadius: BorderRadius.circular(12),
                  border: Border.all(color: Colors.grey),
                  color: Colors.white,
                ),
                child: Row(
                  children: [
                    const Icon(Icons.music_note, color: Colors.purple),
                    const SizedBox(width: 12),
                    Expanded(
                      child: Text(
                        _selectedMusicName ?? 'Select MP3 from device',
                        style: TextStyle(
                          color: _selectedMusicName == null
                              ? Colors.grey
                              : Colors.black,
                        ),
                        overflow: TextOverflow.ellipsis,
                      ),
                    ),
                    const Icon(Icons.folder_open),
                  ],
                ),
              ),
            ),
          ),
          const SizedBox(height: 12),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16.0),
            child: Row(
              children: [
                const Text(
                  'Movement Blocks:',
                  style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
                ),
              ],
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(16.0),
            child: Wrap(
              spacing: 8,
              runSpacing: 8,
              children: _availableMovements.map((movement) {
                return ElevatedButton.icon(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: _getColorForMovement(movement),
                    foregroundColor: Colors.white,
                    padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
                  ),
                  icon: Icon(_getIconForMovement(movement)),
                  label: Text(movement.toUpperCase()),
                  onPressed: () => _addStep(movement),
                );
              }).toList(),
            ),
          ),
          const Divider(thickness: 2),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 8.0),
            child: Row(
              children: [
                const Text(
                  'Dance Sequence:',
                  style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
                ),
                const Spacer(),
                Text(
                  '${_steps.length} steps',
                  style: TextStyle(fontSize: 14, color: Colors.grey[600]),
                ),
              ],
            ),
          ),
          Expanded(
            child: _steps.isEmpty
                ? Center(
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Icon(Icons.queue_music, size: 64, color: Colors.grey[400]),
                        const SizedBox(height: 16),
                        Text(
                          'No steps added yet',
                          style: TextStyle(fontSize: 16, color: Colors.grey[600]),
                        ),
                        const SizedBox(height: 8),
                        Text(
                          'Add movement blocks above to build your dance',
                          style: TextStyle(fontSize: 14, color: Colors.grey[500]),
                        ),
                      ],
                    ),
                  )
                : ReorderableListView.builder(
                    padding: const EdgeInsets.symmetric(horizontal: 16.0),
                    itemCount: _steps.length,
                    onReorder: (oldIndex, newIndex) {
                      setState(() {
                        if (newIndex > oldIndex) {
                          newIndex -= 1;
                        }
                        final step = _steps.removeAt(oldIndex);
                        _steps.insert(newIndex, step);
                      });
                    },
                    itemBuilder: (context, index) {
                      final step = _steps[index];
                      return Card(
                        key: ValueKey(index),
                        margin: const EdgeInsets.only(bottom: 8),
                        elevation: 2,
                        child: Padding(
                          padding: const EdgeInsets.all(12.0),
                          child: Row(
                            children: [
                              Container(
                                width: 40,
                                height: 40,
                                decoration: BoxDecoration(
                                  color: _getColorForMovement(step.movement),
                                  borderRadius: BorderRadius.circular(8),
                                ),
                                child: Icon(
                                  _getIconForMovement(step.movement),
                                  color: Colors.white,
                                ),
                              ),
                              const SizedBox(width: 12),
                              Expanded(
                                child: Column(
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: [
                                    Text(
                                      step.movement.toUpperCase(),
                                      style: const TextStyle(
                                        fontSize: 16,
                                        fontWeight: FontWeight.bold,
                                      ),
                                    ),
                                    const SizedBox(height: 4),
                                    Row(
                                      children: [
                                        const Text('Duration: '),
                                        Expanded(
                                          child: Slider(
                                            value: step.durationMs.toDouble(),
                                            min: 500,
                                            max: 5000,
                                            divisions: 9,
                                            label: '${(step.durationMs / 1000).toStringAsFixed(1)}s',
                                            onChanged: (value) {
                                              _updateStepDuration(index, value.toInt());
                                            },
                                          ),
                                        ),
                                        Text(
                                          '${(step.durationMs / 1000).toStringAsFixed(1)}s',
                                          style: const TextStyle(fontWeight: FontWeight.bold),
                                        ),
                                      ],
                                    ),
                                  ],
                                ),
                              ),
                              IconButton(
                                icon: const Icon(Icons.delete, color: Colors.red),
                                onPressed: () => _removeStep(index),
                              ),
                            ],
                          ),
                        ),
                      );
                    },
                  ),
          ),
        ],
      ),
    );
  }
}
