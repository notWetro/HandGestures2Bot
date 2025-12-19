//
//  BluetoothProvisioningService.swift
//  Runner
//
//  Native BLE implementation for WiFi provisioning
//

import Foundation
import CoreBluetooth

class BluetoothProvisioningService: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    
    // BLE Service and Characteristic UUIDs
    private let serviceUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef0")
    private let ssidCharUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef1")
    private let passwordCharUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef2")
    private let statusCharUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef3")
    
    private var centralManager: CBCentralManager!
    private var connectedPeripheral: CBPeripheral?
    private var ssidCharacteristic: CBCharacteristic?
    private var passwordCharacteristic: CBCharacteristic?
    private var statusCharacteristic: CBCharacteristic?
    
    private var statusCallback: ((String) -> Void)?
    private var ipCallback: ((String) -> Void)?
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    // MARK: - Public API
    
    func getConnectedDevices(callback: @escaping ([String: String]) -> Void) {
        print("🔵 BLE Native: Getting connected devices...")
        
        // Get already connected peripherals
        let connectedPeripherals = centralManager.retrieveConnectedPeripherals(withServices: [serviceUUID])
        
        var devices: [[String: String]] = []
        for peripheral in connectedPeripherals {
            devices.append([
                "name": peripheral.name ?? "Unknown",
                "id": peripheral.identifier.uuidString
            ])
            print("🔵 BLE Native: Found connected device: \(peripheral.name ?? "Unknown")")
        }
        
        callback(["devices": devices.description])
    }
    
    func connectToDevice(deviceId: String, callback: @escaping (Bool, String?) -> Void) {
        print("🔵 BLE Native: Connecting to device: \(deviceId)")
        
        guard let uuid = UUID(uuidString: deviceId) else {
            callback(false, "Invalid device ID")
            return
        }
        
        let peripherals = centralManager.retrievePeripherals(withIdentifiers: [uuid])
        guard let peripheral = peripherals.first else {
            callback(false, "Device not found")
            return
        }
        
        connectedPeripheral = peripheral
        peripheral.delegate = self
        centralManager.connect(peripheral, options: nil)
        
        // Callback will be called in didConnect delegate
    }
    
    func sendWiFiCredentials(ssid: String, password: String, callback: @escaping (Bool, String?) -> Void) {
        guard let peripheral = connectedPeripheral,
              let ssidChar = ssidCharacteristic,
              let passwordChar = passwordCharacteristic else {
            callback(false, "Not connected to device")
            return
        }
        
        print("🔵 BLE Native: Sending WiFi credentials...")
        print("🔵 BLE Native: SSID: \(ssid)")
        
        // Write SSID
        if let ssidData = ssid.data(using: .utf8) {
            peripheral.writeValue(ssidData, for: ssidChar, type: .withResponse)
        }
        
        // Small delay
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            // Write password
            if let passwordData = password.data(using: .utf8) {
                peripheral.writeValue(passwordData, for: passwordChar, type: .withResponse)
            }
            callback(true, nil)
        }
    }
    
    func disconnect() {
        if let peripheral = connectedPeripheral {
            centralManager.cancelPeripheralConnection(peripheral)
            connectedPeripheral = nil
        }
    }
    
    // MARK: - CBCentralManagerDelegate
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            print("🔵 BLE Native: Bluetooth is powered on")
        case .poweredOff:
            print("⚠️ BLE Native: Bluetooth is powered off")
        case .unauthorized:
            print("⚠️ BLE Native: Bluetooth is unauthorized")
        case .unsupported:
            print("⚠️ BLE Native: Bluetooth is unsupported")
        default:
            print("⚠️ BLE Native: Bluetooth state: \(central.state.rawValue)")
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("✅ BLE Native: Connected to \(peripheral.name ?? "device")")
        peripheral.discoverServices([serviceUUID])
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        print("❌ BLE Native: Failed to connect: \(error?.localizedDescription ?? "unknown")")
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print("🔵 BLE Native: Disconnected from device")
        connectedPeripheral = nil
    }
    
    // MARK: - CBPeripheralDelegate
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        
        for service in services {
            if service.uuid == serviceUUID {
                print("✅ BLE Native: Found provisioning service")
                peripheral.discoverCharacteristics([ssidCharUUID, passwordCharUUID, statusCharUUID], for: service)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let characteristics = service.characteristics else { return }
        
        for characteristic in characteristics {
            switch characteristic.uuid {
            case ssidCharUUID:
                ssidCharacteristic = characteristic
                print("✅ BLE Native: Found SSID characteristic")
            case passwordCharUUID:
                passwordCharacteristic = characteristic
                print("✅ BLE Native: Found Password characteristic")
            case statusCharUUID:
                statusCharacteristic = characteristic
                print("✅ BLE Native: Found Status characteristic")
                // Subscribe to status notifications
                peripheral.setNotifyValue(true, for: characteristic)
            default:
                break
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        if characteristic.uuid == statusCharUUID {
            guard let data = characteristic.value,
                  let jsonString = String(data: data, encoding: .utf8) else { return }
            
            print("🔵 BLE Native: Received status: \(jsonString)")
            
            // Parse JSON and extract IP if available
            if let jsonData = jsonString.data(using: .utf8),
               let json = try? JSONSerialization.jsonObject(with: jsonData) as? [String: Any] {
                if let ip = json["ip"] as? String {
                    ipCallback?(ip)
                }
                if let status = json["status"] as? String {
                    statusCallback?(status)
                }
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
        if let error = error {
            print("❌ BLE Native: Write error: \(error.localizedDescription)")
        } else {
            print("✅ BLE Native: Write successful for \(characteristic.uuid)")
        }
    }
}
