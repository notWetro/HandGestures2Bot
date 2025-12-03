import Flutter
import UIKit

@main
@objc class AppDelegate: FlutterAppDelegate {
  override func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
  ) -> Bool {

    // 1. Flutter Plugins laden (NICHT entfernen!)
    GeneratedPluginRegistrant.register(with: self)

    // 2. Deine iOS PlatformView für die Kamera registrieren
    let factory = SwiftCameraViewFactory()
    self.registrar(forPlugin: "my_camera_view")?
        .register(factory, withId: "my_camera_view")

    // 3. Rest wie gehabt
    return super.application(application, didFinishLaunchingWithOptions: launchOptions)
  }
}

