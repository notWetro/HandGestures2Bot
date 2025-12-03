//
//  SwiftCameraPlatformView.swift
//  Runner
//
//  Created by Okan Demirbilek on 02.12.25.
//

import Flutter
import UIKit

class SwiftCameraPlatformView: NSObject, FlutterPlatformView {

    private let controller: CameraViewController

    override init() {
        self.controller = CameraViewController()
        super.init()
    }

    func view() -> UIView {
        return controller.view
    }
}
