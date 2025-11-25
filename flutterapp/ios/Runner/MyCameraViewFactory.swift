//
//  MyCameraViewFactory.swift
//  Runner
//
//  Created by Okan Demirbilek on 24.11.25.
//

import Flutter
import UIKit

class MyCameraViewFactory: NSObject, FlutterPlatformViewFactory {

    func create(
        withFrame frame: CGRect,
        viewIdentifier viewId: Int64,
        arguments args: Any?
    ) -> FlutterPlatformView {
        return MyCameraPlatformView(frame)
    }
}

class MyCameraPlatformView: NSObject, FlutterPlatformView {
    private let cameraView: MyCameraView

    init(_ frame: CGRect) {
        cameraView = MyCameraView(frame: frame)
    }

    func view() -> UIView {
        return cameraView
    }
}
