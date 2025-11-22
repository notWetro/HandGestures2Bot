package com.example.flutterapp

import android.content.Context
import io.flutter.plugin.common.StandardMessageCodec
import io.flutter.plugin.platform.PlatformView
import io.flutter.plugin.platform.PlatformViewFactory

// Creates instances of the MyCameraView class
class MyCameraViewFactory(private val activity: MainActivity) :
    PlatformViewFactory(StandardMessageCodec.INSTANCE) {

    override fun create(context: Context, viewId: Int, args: Any?): PlatformView {
        // Instantiate and return the actual view class, passing the MainActivity context
        return MyCameraView(activity, viewId)
    }
}