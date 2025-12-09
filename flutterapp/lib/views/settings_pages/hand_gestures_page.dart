import 'package:flutter/material.dart';

class HandGesturesPage extends StatefulWidget {
  const HandGesturesPage({super.key});

  @override
  State<HandGesturesPage> createState() => _HandGesturesPageState();
}

class _HandGesturesPageState extends State<HandGesturesPage> {
  final List<Map<String, String>> _gestures = [
    {'name': 'Fist', 'action': 'Stop Robot', 'icon': ''},
    {'name': 'Open Hand', 'action': 'Move Forward', 'icon': ''},
    {'name': 'Peace Sign', 'action': 'Turn Right', 'icon': ''},
    {'name': 'Thumbs Up', 'action': 'Speed Up', 'icon': ''},
    {'name': 'Point', 'action': 'Turn Left', 'icon': ''},
  ];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Hand Gestures'),
        backgroundColor: Colors.orange,
        foregroundColor: Colors.white,
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Gesture Configuration',
              style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 10),
            const Text(
              'Configure what actions each gesture performs:',
              style: TextStyle(fontSize: 16, color: Colors.grey),
            ),
            const SizedBox(height: 20),

            Expanded(
              child: ListView.builder(
                itemCount: _gestures.length,
                itemBuilder: (context, index) {
                  final gesture = _gestures[index];
                  return Card(
                    child: ListTile(
                      leading: Text(
                        gesture['icon']!,
                        style: const TextStyle(fontSize: 24),
                      ),
                      title: Text(gesture['name']!),
                      subtitle: Text('Action: ${gesture['action']!}'),
                      trailing: IconButton(
                        icon: const Icon(Icons.edit),
                        onPressed: () {
                          // TODO: Edit gesture action
                          ScaffoldMessenger.of(context).showSnackBar(
                            SnackBar(
                              content: Text('Edit ${gesture['name']} gesture'),
                            ),
                          );
                        },
                      ),
                    ),
                  );
                },
              ),
            ),

            const SizedBox(height: 16),
            ElevatedButton(
              onPressed: () {
                // TODO: Calibrate gestures
                ScaffoldMessenger.of(context).showSnackBar(
                  const SnackBar(
                    content: Text('Starting gesture calibration...'),
                  ),
                );
              },
              child: const Text('Calibrate Gestures'),
            ),
          ],
        ),
      ),
    );
  }
}
