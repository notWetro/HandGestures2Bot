# Robot Bluetooth WiFi Setup - Implementation Guide

This document describes how to implement the Bluetooth WiFi setup on the robot side.

## Overview

The Flutter app will:
1. Scan for Bluetooth devices
2. Connect to the robot via Bluetooth
3. Send WiFi credentials (SSID and password) to the robot
4. Wait for the robot to connect to WiFi
5. Receive the robot's IP address via Bluetooth
6. Save the IP address and use it for WebSocket communication

## Required Bluetooth Implementation on Robot

### 1. Bluetooth Service and Characteristics

The robot MUST implement a Bluetooth service with the following UUIDs:

#### Service UUID
```
12345678-1234-5678-1234-56789abcdef0
```

#### Characteristic 1: WiFi Configuration (WRITE)
```
UUID: 12345678-1234-5678-1234-56789abcdef1
Properties: WRITE
```
**Purpose:** Receives WiFi credentials from the app

**Data Format:** JSON string
```json
{
  "ssid": "YourWiFiName",
  "password": "YourWiFiPassword"
}
```

**What the robot should do:**
1. Receive the JSON data
2. Parse the JSON to extract `ssid` and `password`
3. Connect to the WiFi network using these credentials
4. Get the assigned IP address
5. Send the IP address back via the IP Address Characteristic

#### Characteristic 2: IP Address (READ + NOTIFY)
```
UUID: 12345678-1234-5678-1234-56789abcdef2
Properties: READ, NOTIFY
```
**Purpose:** Sends the robot's IP address back to the app

**Data Format:** String (UTF-8 encoded)
```
Example: "192.168.1.100"
```

**What the robot should do:**
1. After connecting to WiFi successfully
2. Get the IP address from the network interface
3. Convert it to a string
4. Send it via this characteristic (using NOTIFY)
5. The app will receive this and save it for WebSocket communication

## Implementation Steps for Robot

### Python Example (using `bleak` for Bluetooth)

```python
import asyncio
import json
from bleak import BleakServer, BleakGATTCharacteristic, BleakGATTServiceCollection
from bleak.backends.characteristic import GattCharacteristicsFlags
import subprocess

# UUIDs - MUST match the Flutter app
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
WIFI_CONFIG_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
IP_ADDRESS_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"

# Store characteristics
wifi_config_char = None
ip_address_char = None

# Callback when WiFi credentials are received
def wifi_config_write_callback(characteristic: BleakGATTCharacteristic, data: bytearray):
    print(f"📱 Received WiFi credentials: {len(data)} bytes")
    
    try:
        # Parse JSON data
        credentials = json.loads(data.decode('utf-8'))
        ssid = credentials['ssid']
        password = credentials['password']
        
        print(f"📱 SSID: {ssid}")
        print(f"📱 Connecting to WiFi...")
        
        # Connect to WiFi (Linux example using nmcli)
        subprocess.run([
            'nmcli', 'device', 'wifi', 'connect', ssid, 
            'password', password
        ])
        
        # Wait a moment for connection
        asyncio.sleep(3)
        
        # Get IP address
        ip_address = get_ip_address()
        print(f"📱 Connected! IP Address: {ip_address}")
        
        # Send IP address back to app
        if ip_address_char:
            ip_address_char.value = ip_address.encode('utf-8')
            # Trigger notification to app
            asyncio.create_task(server.notify(IP_ADDRESS_CHAR_UUID, ip_address.encode('utf-8')))
            
    except Exception as e:
        print(f"❌ Error: {e}")

def get_ip_address():
    """Get the current IP address of the robot"""
    # Linux example
    result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
    ip = result.stdout.strip().split()[0]
    return ip

async def main():
    global wifi_config_char, ip_address_char
    
    # Create GATT service
    service_collection = BleakGATTServiceCollection()
    
    # Add service
    service = service_collection.add_service(SERVICE_UUID)
    
    # Add WiFi Config characteristic (WRITE)
    wifi_config_char = service.add_characteristic(
        WIFI_CONFIG_CHAR_UUID,
        GattCharacteristicsFlags.write,
        None,
        wifi_config_write_callback
    )
    
    # Add IP Address characteristic (READ + NOTIFY)
    ip_address_char = service.add_characteristic(
        IP_ADDRESS_CHAR_UUID,
        GattCharacteristicsFlags.read | GattCharacteristicsFlags.notify,
        b"0.0.0.0"
    )
    
    # Start BLE server
    server = BleakServer(service_collection)
    
    print("🔵 Bluetooth server starting...")
    await server.start()
    print("🔵 Bluetooth server running. Waiting for connections...")
    
    # Keep running
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
```

### Alternative: Using Arduino/ESP32

```cpp
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// UUIDs - MUST match the Flutter app
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define WIFI_CONFIG_CHAR_UUID "12345678-1234-5678-1234-56789abcdef1"
#define IP_ADDRESS_CHAR_UUID "12345678-1234-5678-1234-56789abcdef2"

BLECharacteristic *ipAddressCharacteristic;

class WiFiConfigCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        
        if (value.length() > 0) {
            Serial.println("📱 Received WiFi credentials");
            
            // Parse JSON
            StaticJsonDocument<200> doc;
            deserializeJson(doc, value.c_str());
            
            const char* ssid = doc["ssid"];
            const char* password = doc["password"];
            
            Serial.print("📱 SSID: ");
            Serial.println(ssid);
            Serial.println("📱 Connecting to WiFi...");
            
            // Connect to WiFi
            WiFi.begin(ssid, password);
            
            // Wait for connection
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                delay(500);
                Serial.print(".");
                attempts++;
            }
            
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\n✅ Connected to WiFi!");
                
                // Get IP address
                String ipAddress = WiFi.localIP().toString();
                Serial.print("📱 IP Address: ");
                Serial.println(ipAddress);
                
                // Send IP address back to app
                ipAddressCharacteristic->setValue(ipAddress.c_str());
                ipAddressCharacteristic->notify();
            } else {
                Serial.println("\n❌ Failed to connect to WiFi");
            }
        }
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("🔵 Starting Bluetooth server...");
    
    // Initialize BLE
    BLEDevice::init("RobotBLE");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // WiFi Config characteristic (WRITE)
    BLECharacteristic *wifiConfigCharacteristic = pService->createCharacteristic(
        WIFI_CONFIG_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    wifiConfigCharacteristic->setCallbacks(new WiFiConfigCallbacks());
    
    // IP Address characteristic (READ + NOTIFY)
    ipAddressCharacteristic = pService->createCharacteristic(
        IP_ADDRESS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    ipAddressCharacteristic->addDescriptor(new BLE2902());
    ipAddressCharacteristic->setValue("0.0.0.0");
    
    // Start service
    pService->start();
    
    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
    
    Serial.println("🔵 Bluetooth server running!");
}

void loop() {
    delay(1000);
}
```

## Testing Flow

1. **Robot**: Start Bluetooth server
2. **App**: Scan for Bluetooth devices
3. **App**: Connect to robot
4. **App**: Send WiFi credentials via WiFi Config characteristic
5. **Robot**: Receive credentials, connect to WiFi
6. **Robot**: Get IP address and send via IP Address characteristic (NOTIFY)
7. **App**: Receive IP address, save it
8. **Robot**: Keep WiFi connection, start WebSocket server on saved IP
9. **App**: Disconnect Bluetooth, connect via WebSocket using saved IP

## Important Notes

- The UUIDs MUST match exactly between the app and robot
- The robot must implement NOTIFY on the IP Address characteristic
- After setup, the robot should save the WiFi credentials for auto-reconnect on reboot
- The WebSocket server should continue running on the configured IP address
- The app will automatically use the saved IP address for future connections

## Debugging

Enable these debug logs on the robot:
- ✅ Bluetooth server started
- ✅ Device connected
- ✅ WiFi credentials received
- ✅ Connecting to WiFi
- ✅ WiFi connected, IP: xxx.xxx.xxx.xxx
- ✅ IP address sent to app

## Flutter App Debug Output

You'll see these logs in the Flutter console:
```
🔵 BLUETOOTH: Starting device scan...
🔵 BLUETOOTH: Found device: RobotBLE
🔵 BLUETOOTH: Connecting to RobotBLE...
🔵 BLUETOOTH: Connected successfully!
🔵 BLUETOOTH: Discovering services...
🔵 BLUETOOTH: Found robot service!
🔵 BLUETOOTH: WiFi Config characteristic found!
🔵 BLUETOOTH: IP Address characteristic found!
🔵 BLUETOOTH: Sending WiFi credentials...
✅ BLUETOOTH: WiFi credentials sent successfully!
🔵 BLUETOOTH: Waiting for robot to connect to WiFi and send IP address...
🔵 BLUETOOTH: Received IP address from robot: 192.168.1.100
💾 Saved IP address to local storage: 192.168.1.100
📱 Using saved IP address from Bluetooth setup: 192.168.1.100
```
