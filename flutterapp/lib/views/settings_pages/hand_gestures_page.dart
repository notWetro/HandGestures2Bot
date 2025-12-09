import 'package:flutter/material.dart';
import 'package:flutterapp/views/settings_pages/save_gesture_view.dart';

class HandGesturesPage extends StatefulWidget {
  const HandGesturesPage({super.key});

  @override
  State<HandGesturesPage> createState() => _HandGesturesPageState();
}

class _HandGesturesPageState extends State<HandGesturesPage> {
  final List<Map<String, String>> _actions = [
    {'action': 'Stop Robot', 'gesture': 'Fist', 'icon': ''},
    {'action': 'Move Forward', 'gesture': 'Open Hand', 'icon': ''},
    {'action': 'Turn Right', 'gesture': 'Peace Sign', 'icon': ''},
    {'action': 'Speed Up', 'gesture': 'Thumbs Up', 'icon': ''},
    {'action': 'Turn Left', 'gesture': 'Point', 'icon': ''},
  ];

  void _editGesture(int index) async {
    final action = _actions[index];
    final result = await Navigator.push<String>(
      context,
      MaterialPageRoute(
        builder: (context) => SaveGestureView(
          actionName: action['action']!,
          currentGesture: action['gesture']!,
        ),
      ),
    );

    if (result != null) {
      setState(() {
        _actions[index]['gesture'] = result;
        // Update icon based on gesture name
        switch (result) {
          case 'Fist':
            _actions[index]['icon'] = '';
            break;
          case 'Open Hand':
            _actions[index]['icon'] = '';
            break;
          case 'Peace Sign':
            _actions[index]['icon'] = '';
            break;
          case 'Thumbs Up':
            _actions[index]['icon'] = '';
            break;
          case 'Point':
            _actions[index]['icon'] = '';
            break;
          case 'OK Sign':
            _actions[index]['icon'] = '';
            break;
          case 'Rock On':
            _actions[index]['icon'] = '';
            break;
          default:
            _actions[index]['icon'] = '';
        }
      });

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Updated ${action['action']} to use ${result} gesture'),
        ),
      );
    }
  }

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
              'Action Configuration',
              style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 10),
            const Text(
              'Configure what gesture triggers each robot action:',
              style: TextStyle(fontSize: 16, color: Colors.grey),
            ),
            const SizedBox(height: 20),

            Expanded(
              child: ListView.builder(
                itemCount: _actions.length,
                itemBuilder: (context, index) {
                  final action = _actions[index];
                  return Card(
                    child: ListTile(
                      leading: Text(
                        action['icon']!,
                        style: const TextStyle(fontSize: 32),
                      ),
                      title: Text(
                        action['action']!,
                        style: const TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                      subtitle: Text(
                        action['gesture']!,
                        style: const TextStyle(
                          fontSize: 14,
                          color: Colors.grey,
                        ),
                      ),
                      trailing: IconButton(
                        icon: const Icon(Icons.edit, color: Colors.orange),
                        onPressed: () => _editGesture(index),
                      ),
                    ),
                  );
                },
              ),
            ),

            const SizedBox(height: 16),
            SizedBox(
              width: double.infinity,
              child: ElevatedButton(
                onPressed: () {
                  // TODO: Calibrate gestures
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('Starting gesture calibration...'),
                    ),
                  );
                },
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.orange,
                  foregroundColor: Colors.white,
                  padding: const EdgeInsets.symmetric(vertical: 16),
                ),
                child: const Text(
                  'Calibrate All Gestures',
                  style: TextStyle(fontSize: 16),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
