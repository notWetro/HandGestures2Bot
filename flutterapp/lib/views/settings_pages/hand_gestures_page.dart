import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutterapp/views/settings_pages/save_gesture_view.dart';

class HandGesturesPage extends StatefulWidget {
  const HandGesturesPage({super.key});

  @override
  State<HandGesturesPage> createState() => _HandGesturesPageState();
}

class _HandGesturesPageState extends State<HandGesturesPage> {
  // 1. The Fixed List of Commands the Robot understands
  final List<String> _robotActions = [
    'Stop Robot',
    'Move Forward',
    'Move Backward',
    'Turn Left',
    'Turn Right',
  ];

  // 2. List of gestures currently saved in Android memory
  List<String> _savedGestures = [];
  bool _isLoading = true;

  static const gestureChannel = MethodChannel('gesture_channel');

  @override
  void initState() {
    super.initState();
    _fetchGestures();
  }

  // Ask Android what it knows
  Future<void> _fetchGestures() async {
    try {
      final List<dynamic> result = await gestureChannel.invokeMethod('getGestureList');
      if (mounted) {
        setState(() {
          _savedGestures = result.cast<String>();
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
                  : ListView.builder(
                      itemCount: _robotActions.length,
                      itemBuilder: (context, index) {
                        final action = _robotActions[index];
                        
                        // Check if this action is already learned
                        final isConfigured = _savedGestures.contains(action);

                        return Card(
                          elevation: isConfigured ? 2 : 0,
                          color: isConfigured ? Colors.white : Colors.grey[100],
                          margin: const EdgeInsets.only(bottom: 12),
                          shape: RoundedRectangleBorder(
                            side: BorderSide(
                              color: isConfigured ? Colors.green.shade200 : Colors.transparent,
                              width: 1,
                            ),
                            borderRadius: BorderRadius.circular(12),
                          ),
                          child: ListTile(
                            leading: CircleAvatar(
                              backgroundColor: isConfigured ? Colors.green.shade100 : Colors.grey.shade300,
                              child: Icon(
                                isConfigured ? Icons.check : Icons.question_mark,
                                color: isConfigured ? Colors.green : Colors.grey,
                              ),
                            ),
                            title: Text(
                              action,
                              style: TextStyle(
                                fontSize: 18,
                                fontWeight: FontWeight.bold,
                                color: isConfigured ? Colors.black : Colors.grey[700],
                              ),
                            ),
                            subtitle: Text(
                              isConfigured ? "Gesture Recorded" : "Not Configured (Empty)",
                              style: TextStyle(
                                color: isConfigured ? Colors.green : Colors.redAccent,
                                fontWeight: FontWeight.w500,
                              ),
                            ),
                            trailing: ElevatedButton.icon(
                              style: ElevatedButton.styleFrom(
                                backgroundColor: isConfigured ? Colors.white : Colors.blue,
                                foregroundColor: isConfigured ? Colors.blue : Colors.white,
                                elevation: isConfigured ? 0 : 2,
                                side: isConfigured ? const BorderSide(color: Colors.blue) : null,
                              ),
                              icon: const Icon(Icons.camera_alt, size: 18),
                              label: Text(isConfigured ? "Edit" : "Record"),
                              onPressed: () => _recordAction(action),
                            ),
                          ),
                        );
                      },
                    ),
            ),
          ],
        ),
      ),
    );
  }
}