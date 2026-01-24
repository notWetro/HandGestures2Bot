import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutterapp/services/gesture_service.dart';

class SaveGestureView extends StatefulWidget {
  final String? currentGesture;
  final String? actionName;

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
  final GestureService _gestureService = GestureService();
  StreamSubscription<String>? _saveSubscription;

  @override
  void initState() {
    super.initState();
    _textController = TextEditingController(text: widget.currentGesture ?? "");

    // Listen for the native side to confirm the save was successful
    _saveSubscription = _gestureService.onSaveSuccess.listen((message) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Gesture "$message" saved successfully!'),
            backgroundColor: Colors.green,
          ),
        );
        // Pop the screen, returning the new gesture name
        Navigator.pop(context, message);
      }
    });
  }

  @override
  void dispose() {
    _textController.dispose();
    _saveSubscription?.cancel();
    _gestureService.dispose();
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

    // Give immediate feedback
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('Memorizing "$name"... Hold steady!')),
    );

    // Tell the service to trigger the save on the native side
    await _gestureService.saveGesture(name);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      resizeToAvoidBottomInset: false,
      appBar: AppBar(
        title: Text(widget.actionName != null
            ? "Record: ${widget.actionName}"
            : "Save Gesture"),
        backgroundColor: Colors.orange,
        foregroundColor: Colors.white,
      ),
      body: Padding(
        padding: const EdgeInsets.all(24.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              "Position your hand in the camera box below.",
              style: TextStyle(fontSize: 16, color: Colors.grey),
            ),
            const SizedBox(height: 20),
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
                  // Use the service to build the native camera view
                  child: _gestureService.buildCameraView(),
                ),
              ),
            ),
            const SizedBox(height: 30),
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