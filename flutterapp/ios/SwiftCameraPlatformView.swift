//
//  SwiftCameraPlatformView.swift
//  
//
//  Created by Okan Demirbilek on 02.12.25.
//

import Flutter
import UIKit

class SwiftCameraPlatformView: NSObject, FlutterPlatformView {

    private let containerView: UIView
    private let cameraVC = CameraViewController()
    private let channel: FlutterMethodChannel

    init(frame: CGRect, viewId: Int64, messenger: FlutterBinaryMessenger) {

        self.containerView = UIView(frame: frame)

        self.channel = FlutterMethodChannel(
            name: "gesture_channel",
            binaryMessenger: messenger
        )

        super.init()

        cameraVC.methodChannel = channel

        // camera ONLY inside this platform view
        embedCameraONLYInsideContainer()

        channel.setMethodCallHandler(handleMethodCall)
    }

    func view() -> UIView {
        return containerView
    }

    private func embedCameraONLYInsideContainer() {

        guard let parent = UIApplication.shared
            .connectedScenes
            .compactMap({ $0 as? UIWindowScene })
            .flatMap({ $0.windows })
            .first(where: { $0.isKeyWindow })?
            .rootViewController
        else { return }

        parent.addChild(cameraVC)
        cameraVC.view.frame = containerView.bounds
        cameraVC.view.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        containerView.addSubview(cameraVC.view)
        cameraVC.didMove(toParent: parent)
    }

    private func handleMethodCall(call: FlutterMethodCall, result: FlutterResult) {
        switch call.method {

        case "saveGesture":
            if let name = call.arguments as? String,
               let lm = cameraVC.overlay.latestLandmarks {
                cameraVC.saveGesture(name: name, landmarks: lm)
            }
            result(nil)

        case "getGestureList":
            result(cameraVC.recognizer.getSavedNames())

        default:
            result(FlutterMethodNotImplemented)
        }
    }
}
