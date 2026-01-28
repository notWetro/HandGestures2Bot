//
//  SwiftCameraViewFactory.swift
//  
//
//  Created by Okan Demirbilek on 02.12.25.
//

import Flutter
import UIKit

class SwiftCameraViewFactory: NSObject, FlutterPlatformViewFactory {

    private let messenger: FlutterBinaryMessenger

    init(messenger: FlutterBinaryMessenger) {
        self.messenger = messenger
        super.init()
    }

    func create(
        withFrame frame: CGRect,
        viewIdentifier viewId: Int64,
        arguments args: Any?
    ) -> FlutterPlatformView {

        return SwiftCameraPlatformView(
            frame: frame,
            viewId: viewId,
            messenger: messenger
        )
    }
}
