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

    private var connectCallback: ((Bool, String?) -> Void)?

    private func completeConnect(_ success: Bool, _ message: String?) {
        guard let cb = connectCallback else { return }
        connectCallback = nil
        DispatchQueue.main.async {
            cb(success, message)
        }
    }
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    // MARK: - Callback Setup
    
    func setIpCallback(_ callback: @escaping (String) -> Void) {
        ipCallback = callback
    }
    
    func setStatusCallback(_ callback: @escaping (String) -> Void) {
        statusCallback = callback
    }
    
    // MARK: - Public API
    
    private var scanCallback: (([[String: String]]) -> Void)?
    private var isScanning: Bool = false
    
    func startScanning(callback: @escaping ([[String: String]]) -> Void) {
        print("🔵 BLE Native: Starting scan...")
        if isScanning {
            // Avoid overwriting the pending callback; just return what we have so far.
            var devices: [[String: String]] = []
            for p in discoveredPeripherals {
                devices.append([
                    "name": p.name ?? "Unknown",
                    "id": p.identifier.uuidString
                ])
            }
            callback(devices)
            return
        }
        // IMPORTANT: MethodChannel results must be returned exactly once.
        // We therefore collect peripherals during the scan window and return
        // the full list when scanning stops.
        discoveredPeripherals.removeAll()
        scanCallback = callback
        isScanning = true
        
        // Scan for devices advertising our service
        centralManager.scanForPeripherals(withServices: [serviceUUID], options: nil)
        
        // Stop scan after 10 seconds
        DispatchQueue.main.asyncAfter(deadline: .now() + 10) { [weak self] in
            self?.stopScanning()
        }
    }
    
    func stopScanning() {
        centralManager.stopScan()
        print("🔵 BLE Native: Scan stopped")

        guard isScanning else { return }
        isScanning = false

        // Return discovered devices once
        if let callback = scanCallback {
            var devices: [[String: String]] = []
            for p in discoveredPeripherals {
                devices.append([
                    "name": p.name ?? "Unknown",
                    "id": p.identifier.uuidString
                ])
            }
            scanCallback = nil
            callback(devices)
        }
    }
    
    func getConnectedDevices(callback: @escaping ([[String: String]]) -> Void) {
        print("🔵 BLE Native: Getting connected devices...")
        
        var devices: [[String: String]] = []
        
        // Get already connected peripherals
        let connectedPeripherals = centralManager.retrieveConnectedPeripherals(withServices: [serviceUUID])
        
        for peripheral in connectedPeripherals {
            devices.append([
                "name": peripheral.name ?? "Unknown",
                "id": peripheral.identifier.uuidString
            ])
            print("🔵 BLE Native: Found connected device: \(peripheral.name ?? "Unknown")")
        }
        
        callback(devices)
    }
    
    func connectToDevice(deviceId: String, callback: @escaping (Bool, String?) -> Void) {
        print("🔵 BLE Native: Connecting to device: \(deviceId)")

        // Keep only the latest attempt; ensure we complete it exactly once.
        connectCallback = callback

        guard centralManager.state == .poweredOn else {
            completeConnect(false, "Bluetooth is not powered on")
            return
        }
        
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
        
        // Callback will be completed once characteristics are discovered (or on failure)
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
    
    private var discoveredPeripherals: [CBPeripheral] = []
    
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
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        print("🔵 BLE Native: Discovered \(peripheral.name ?? "Unknown") - \(peripheral.identifier)")
        
        if !discoveredPeripherals.contains(where: { $0.identifier == peripheral.identifier }) {
            discoveredPeripherals.append(peripheral)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("✅ BLE Native: Connected to \(peripheral.name ?? "device")")
        peripheral.discoverServices([serviceUUID])
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        print("❌ BLE Native: Failed to connect: \(error?.localizedDescription ?? "unknown")")
        completeConnect(false, error?.localizedDescription ?? "Failed to connect")
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print("🔵 BLE Native: Disconnected from device")
        connectedPeripheral = nil
    }
    
    // MARK: - CBPeripheralDelegate
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let error = error {
            print("❌ BLE Native: Service discovery error: \(error.localizedDescription)")
            completeConnect(false, error.localizedDescription)
            return
        }
        guard let services = peripheral.services else {
            completeConnect(false, "No services discovered")
            return
        }

        var foundProvisioningService = false
        
        for service in services {
            if service.uuid == serviceUUID {
                foundProvisioningService = true
                print("✅ BLE Native: Found provisioning service")
                peripheral.discoverCharacteristics([ssidCharUUID, passwordCharUUID, statusCharUUID], for: service)
            }
        }

        if !foundProvisioningService {
            completeConnect(false, "Provisioning service not found")
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let error = error {
            print("❌ BLE Native: Characteristic discovery error: \(error.localizedDescription)")
            completeConnect(false, error.localizedDescription)
            return
        }
        guard let characteristics = service.characteristics else {
            completeConnect(false, "No characteristics discovered")
            return
        }
        
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

        // Consider the device "ready" when we have all characteristics we need.
        if ssidCharacteristic != nil, passwordCharacteristic != nil, statusCharacteristic != nil {
            completeConnect(true, nil)
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
                    print("🔵 BLE Native: Extracted IP: \(ip)")
                    ipCallback?(ip)
                }
                if let status = json["status"] as? String {
                    print("🔵 BLE Native: Extracted status: \(status)")
                    statusCallback?(status)
                }
                if let errorMsg = json["error"] as? String {
                    print("❌ BLE Native: Error from robot: \(errorMsg)")
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
