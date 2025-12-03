//
//  SwiftCameraViewFactory.swift
//  Runner
//
//  Created by Okan Demirbilek on 02.12.25.
//

import Flutter
import UIKit

class SwiftCameraViewFactory: NSObject, FlutterPlatformViewFactory {

    func create(withFrame frame: CGRect,
                viewIdentifier viewId: Int64,
                arguments args: Any?) -> FlutterPlatformView {

        return SwiftCameraPlatformView()
    }
}
