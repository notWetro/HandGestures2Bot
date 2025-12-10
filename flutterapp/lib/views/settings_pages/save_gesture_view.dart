import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
// Ensure this imports your actual Native View wrapper
import 'package:flutterapp/my_camera_view.dart';

class SaveGestureView extends StatefulWidget {
  // Optional parameter to pre-fill the text box
  final String? currentGesture;
  final String? actionName; // Optional: To show "Recording for: Stop Robot"

  const SaveGestureView({
    super.key, 
    this.currentGesture,
    this.actionName,
  });

  @override
  State<SaveGestureView> createState() => _SaveGestureViewState();
}

class _SaveGestureViewState extends State<SaveGestureView> {
  late TextEditingController _textController;
  
  // The Channel to talk to Android (Must match the name in Kotlin!)
  static const gestureChannel = MethodChannel('gesture_channel');

  @override
  void initState() {
    super.initState();
    // Pre-fill the text box if a name was passed
    _textController = TextEditingController(text: widget.currentGesture ?? "");
  }

  @override
  void dispose() {
    _textController.dispose();
    super.dispose();
  }

  Future<void> _saveGesture() async {
    final String name = _textController.text.trim();
    
    if (name.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('Please enter a gesture name first!')),
      );
      return;
    }

    try {
      // Send command to Android: "Next time you see a hand, save it as [name]"
      // The result might return "true" if successful, but we rely on the Android side logic.
      await gestureChannel.invokeMethod('saveGesture', name);
      
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Memorizing "$name"... Hold steady!'),
            backgroundColor: Colors.green,
          ),
        );
        
        // Wait a brief moment for the user to read the message, then close
        // Passing back the name allows the previous screen to update its list if needed
        Future.delayed(const Duration(milliseconds: 500), () {
            if (mounted) {
                Navigator.pop(context, name); 
            }
        });
      }
    } catch (e) {
      debugPrint("Error saving gesture: $e");
      if (mounted) {
         ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('Error: $e')),
        );
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      // Prevents the camera from getting squished when keyboard opens
      resizeToAvoidBottomInset: false, 
      appBar: AppBar(
        title: Text(widget.actionName != null ? "Record: ${widget.actionName}" : "Save Gesture"),
        backgroundColor: Colors.orange,
        foregroundColor: Colors.white,
      ),
      body: Padding(
        padding: const EdgeInsets.all(24.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // --- 1. INSTRUCTIONS ---
            const Text(
              "Position your hand in the camera box below.",
              style: TextStyle(fontSize: 16, color: Colors.grey),
            ),
            const SizedBox(height: 20),

            // --- 2. THE REAL CAMERA VIEW ---
            Center(
              child: Container(
                height: 300, 
                width: double.infinity,
                decoration: BoxDecoration(
                  border: Border.all(color: Colors.orange, width: 2),
                  borderRadius: BorderRadius.circular(24),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.black.withOpacity(0.1),
                      blurRadius: 10,
                      offset: const Offset(0, 5),
                    ),
                  ],
                ),
                child: ClipRRect(
                  borderRadius: BorderRadius.circular(22),
                  // This loads the Native Kotlin Camera
                  child: const MyCameraView(), 
                ),
              ),
            ),
            const SizedBox(height: 30),

            // --- 3. INPUT FIELD ---
            const Text(
              "Gesture Name:",
              style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 10),
            TextField(
              controller: _textController,
              decoration: InputDecoration(
                hintText: 'e.g. "Fist", "Point_Left"',
                border: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(12),
                ),
                filled: true,
                fillColor: Colors.grey[100],
                prefixIcon: const Icon(Icons.label),
              ),
            ),

            const Spacer(),

            // --- 4. SAVE BUTTON ---
            SizedBox(
              width: double.infinity,
              height: 55,
              child: ElevatedButton.icon(
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.blue,
                  foregroundColor: Colors.white,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(16),
                  ),
                  elevation: 4,
                ),
                onPressed: _saveGesture,
                icon: const Icon(Icons.save),
                label: const Text(
                  "Memorize & Save",
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ),
            ),
            const SizedBox(height: 20),
          ],
        ),
      ),
    );
  }
}