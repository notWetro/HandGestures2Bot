import Flutter
import UIKit

@main
@objc class AppDelegate: FlutterAppDelegate {
  
  private var bluetoothService: BluetoothProvisioningService?

  override func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey : Any]?
  ) -> Bool {
    
    // Test Zeile
    GeneratedPluginRegistrant.register(with: self)


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

    // Dance moves channel
    let danceChannel = FlutterMethodChannel(
        name: "dance_channel",
        binaryMessenger: controller.binaryMessenger
    )

    danceChannel.setMethodCallHandler { call, result in
        switch call.method {

        case "loadDanceMoves":
            let jsonString = DanceStore.shared.loadDanceMoves()
            result(jsonString)

        case "saveDanceMoves":
            if let jsonString = call.arguments as? String {
                DanceStore.shared.saveDanceMoves(jsonString: jsonString)
                result(nil)
            } else {
                result(FlutterError(code: "INVALID_ARGS",
                        message: "Expected JSON string",
                        details: nil))
            }

        default:
            result(FlutterMethodNotImplemented)
        }
    }

    // Bluetooth provisioning channel
    self.bluetoothService = BluetoothProvisioningService()
    let bluetoothChannel = FlutterMethodChannel(
        name: "bluetooth_channel",
        binaryMessenger: controller.binaryMessenger
    )

    // Setup callbacks for IP and status updates
    self.bluetoothService?.setIpCallback { ipAddress in
        DispatchQueue.main.async {
            bluetoothChannel.invokeMethod("onIpAddressReceived", arguments: ipAddress)
            print("🔵 BLE: Notifying Flutter about IP: \(ipAddress)")
        }
    }
    
    self.bluetoothService?.setStatusCallback { status in
        DispatchQueue.main.async {
            bluetoothChannel.invokeMethod("onStatusUpdate", arguments: ["status": status])
            print("🔵 BLE: Notifying Flutter about status: \(status)")
        }
    }

    bluetoothChannel.setMethodCallHandler { call, result in
        switch call.method {

        case "getConnectedDevices":
            self.bluetoothService?.getConnectedDevices { devices in
                result(devices)
            }
        
        case "startScanning":
            self.bluetoothService?.startScanning { devices in
                // Send results back to Flutter via EventChannel or callback
                result(devices)
            }
            
        case "stopScanning":
            self.bluetoothService?.stopScanning()
            result(nil)

        case "connectToDevice":
            guard let deviceId = call.arguments as? String else {
                result(FlutterError(code: "INVALID_ARGS",
                        message: "Expected device ID string",
                        details: nil))
                return
            }
            self.bluetoothService?.connectToDevice(deviceId: deviceId) { success, error in
                if success {
                    result(true)
                } else {
                    result(FlutterError(code: "CONNECT_ERROR",
                            message: error ?? "Connection failed",
                            details: nil))
                }
            }

        case "sendWiFiCredentials":
            guard let args = call.arguments as? [String: String],
                  let ssid = args["ssid"],
                  let password = args["password"] else {
                result(FlutterError(code: "INVALID_ARGS",
                        message: "Expected ssid and password",
                        details: nil))
                return
            }
            self.bluetoothService?.sendWiFiCredentials(ssid: ssid, password: password) { success, error in
                if success {
                    result(true)
                } else {
                    result(FlutterError(code: "SEND_ERROR",
                            message: error ?? "Send failed",
                            details: nil))
                }
            }

        case "disconnect":
            self.bluetoothService?.disconnect()
            result(nil)

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
