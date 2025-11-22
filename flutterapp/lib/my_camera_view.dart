import 'dart:io';
import 'package:flutter/services.dart';
import 'package:flutter/material.dart';

class MyCameraView extends StatefulWidget {
  const MyCameraView({super.key});

  @override
  State<MyCameraView> createState() => _MyCameraViewState();
}

class _MyCameraViewState extends State<MyCameraView> {
  final Map<String, dynamic> creationParams = <String, dynamic>{};

  @override
  Widget build(BuildContext context) {

    if (Platform.isAndroid) {
      return AndroidView(
      viewType: 'my_camera_view', 
      layoutDirection: TextDirection.ltr,
      creationParams: creationParams,
      creationParamsCodec: const StandardMessageCodec(),
    );
    }else if (Platform.isIOS) {
      // TODO für OKAN
      return const Center(
        child: Text('Camera view is only available on Android.'),
      );
    } 
    else {
      return const Center(
        child: Text('Camera view is only available on Android or IOS.'),
      );
    }

    
  }
}