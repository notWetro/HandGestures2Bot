package com.example.flutterapp

import android.content.Context
import io.flutter.plugin.common.BinaryMessenger
import io.flutter.plugin.common.StandardMessageCodec
import io.flutter.plugin.platform.PlatformView
import io.flutter.plugin.platform.PlatformViewFactory

// Receives MainActivity AND Messenger
class MyCameraViewFactory(
    private val activity: MainActivity, 
    private val messenger: BinaryMessenger
) : PlatformViewFactory(StandardMessageCodec.INSTANCE) {

    override fun create(context: Context, viewId: Int, args: Any?): PlatformView {
        // Passes Activity and Messenger to the View
        return MyCameraView(activity, messenger, viewId)
    }
}