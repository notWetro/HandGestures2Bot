import 'package:flutter/material.dart';

class SaveGestureView extends StatefulWidget {
  final String actionName;
  final String currentGesture;

  const SaveGestureView({
    super.key,
    required this.actionName,
    required this.currentGesture,
  });

  @override
  State<SaveGestureView> createState() => _SaveGestureViewState();
}

class _SaveGestureViewState extends State<SaveGestureView> {
  String? selectedGesture;
  final List<Map<String, String>> availableGestures = [
    {'name': 'Fist', 'icon': '✊', 'description': 'Closed fist'},
    {'name': 'Open Hand', 'icon': '✋', 'description': 'Open palm'},
    {'name': 'Peace Sign', 'icon': '✌️', 'description': 'Two fingers up'},
    {'name': 'Thumbs Up', 'icon': '👍', 'description': 'Thumb pointing up'},
    {'name': 'Point', 'icon': '👉', 'description': 'Index finger pointing'},
    {
      'name': 'OK Sign',
      'icon': '👌',
      'description': 'Thumb and index finger circle',
    },
    {'name': 'Rock On', 'icon': '🤟', 'description': 'Index and pinky up'},
  ];

  @override
  void initState() {
    super.initState();
    selectedGesture = widget.currentGesture;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Edit ${widget.actionName}'),
        backgroundColor: Colors.orange,
        foregroundColor: Colors.white,
        actions: [
          TextButton(
            onPressed: () {
              if (selectedGesture != null) {
                Navigator.pop(context, selectedGesture);
              }
            },
            child: const Text(
              'Save',
              style: TextStyle(
                color: Colors.white,
                fontWeight: FontWeight.bold,
              ),
            ),
          ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Select gesture for "${widget.actionName}"',
              style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 10),
            Text(
              'Current gesture: ${widget.currentGesture}',
              style: const TextStyle(fontSize: 16, color: Colors.grey),
            ),
            const SizedBox(height: 20),

            const Text(
              'Available Gestures:',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.w600),
            ),
            const SizedBox(height: 16),

            Expanded(
              child: ListView.builder(
                itemCount: availableGestures.length,
                itemBuilder: (context, index) {
                  final gesture = availableGestures[index];
                  final isSelected = selectedGesture == gesture['name'];

                  return Card(
                    color: isSelected ? Colors.orange.shade50 : null,
                    child: ListTile(
                      leading: Text(
                        gesture['icon']!,
                        style: const TextStyle(fontSize: 32),
                      ),
                      title: Text(
                        gesture['name']!,
                        style: TextStyle(
                          fontWeight: isSelected
                              ? FontWeight.bold
                              : FontWeight.normal,
                        ),
                      ),
                      subtitle: Text(gesture['description']!),
                      trailing: isSelected
                          ? const Icon(Icons.check_circle, color: Colors.orange)
                          : null,
                      onTap: () {
                        setState(() {
                          selectedGesture = gesture['name'];
                        });
                      },
                    ),
                  );
                },
              ),
            ),

            const SizedBox(height: 16),
            Container(
              width: double.infinity,
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: Colors.blue.shade50,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.blue.shade200),
              ),
              child: Column(
                children: [
                  const Icon(Icons.info_outline, color: Colors.blue),
                  const SizedBox(height: 8),
                  const Text(
                    'Tip: Test the gesture',
                    style: TextStyle(fontWeight: FontWeight.bold),
                  ),
                  const SizedBox(height: 4),
                  Text(
                    'Make sure you can comfortably perform the selected gesture before saving.',
                    textAlign: TextAlign.center,
                    style: TextStyle(color: Colors.grey.shade700),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
