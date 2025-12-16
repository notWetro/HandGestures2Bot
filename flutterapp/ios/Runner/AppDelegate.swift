import Flutter
import UIKit

@main
@objc class AppDelegate: FlutterAppDelegate {

  override func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey : Any]?
  ) -> Bool {

    let controller = window?.rootViewController as! FlutterViewController

    // GLOBAL channel that always responds
    let gestureChannel = FlutterMethodChannel(
        name: "gesture_channel",
        binaryMessenger: controller.binaryMessenger
    )

    gestureChannel.setMethodCallHandler { call, result in
        switch call.method {

        case "getGestureList":
            let names = GestureStore.shared.getSavedNames()
            print("iOS → sending gesture list: \(names)")
            result(names)

        case "saveGesture":
            result(FlutterError(code: "NO_CAMERA",
                    message: "saveGesture must come from camera view",
                    details: nil))

        default:
            result(FlutterMethodNotImplemented)
        }
    }

    // Register iOS camera View
    let factory = SwiftCameraViewFactory(messenger: controller.binaryMessenger)
    registrar(forPlugin: "my_camera_view")?
        .register(factory, withId: "my_camera_view")

    return super.application(application, didFinishLaunchingWithOptions: launchOptions)
  }
}

