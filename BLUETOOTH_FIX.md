# Bluetooth WiFi Setup - Fixes Applied

## Problem
Die Flutter App konnte keine IP-Adresse vom TurtleBot empfangen, obwohl die Bluetooth-Verbindung funktionierte.

## Ursache
Die iOS native Implementierung hatte zwar Callbacks für IP und Status definiert (`ipCallback`, `statusCallback`), aber diese waren **nicht** mit dem Flutter MethodChannel verbunden. Die Benachrichtigungen vom Roboter wurden empfangen, aber nie an die Flutter-Ebene weitergeleitet.

## Lösung

### 1. iOS AppDelegate.swift
**Änderung:** Callbacks vom `BluetoothProvisioningService` werden jetzt an Flutter weitergeleitet.

```swift
// Setup callbacks for IP and status updates
bluetoothService.setIpCallback { ipAddress in
    DispatchQueue.main.async {
        bluetoothChannel.invokeMethod("onIpAddressReceived", arguments: ipAddress)
        print("🔵 BLE: Notifying Flutter about IP: \(ipAddress)")
    }
}

bluetoothService.setStatusCallback { status in
    DispatchQueue.main.async {
        bluetoothChannel.invokeMethod("onStatusUpdate", arguments: ["status": status])
        print("🔵 BLE: Notifying Flutter about status: \(status)")
    }
}
```

### 2. iOS BluetoothProvisioningService.swift
**Änderung:** Public Methoden hinzugefügt, um Callbacks zu setzen.

```swift
func setIpCallback(_ callback: @escaping (String) -> Void) {
    ipCallback = callback
}

func setStatusCallback(_ callback: @escaping (String) -> Void) {
    statusCallback = callback
}
```

### 3. Flutter NativeBluetoothService.dart
**Änderung:** MethodCallHandler eingerichtet, um Callbacks von iOS zu empfangen.

```dart
void _setupMethodCallHandler() {
  _channel.setMethodCallHandler((call) async {
    switch (call.method) {
      case 'onIpAddressReceived':
        final ipAddress = call.arguments as String;
        debugPrint('🔵 BLE: Received IP from native: $ipAddress');
        _ipAddressController.add(ipAddress);
        break;
      
      case 'onStatusUpdate':
        final status = Map<String, dynamic>.from(call.arguments);
        debugPrint('🔵 BLE: Received status from native: $status');
        _statusController.add(status);
        break;
    }
  });
}
```

## Datenfluss (jetzt komplett)

```
TurtleBot (Python)
    ↓
[BLE Status Characteristic Notify]
    ↓ JSON: {"status": "connected", "ip": "192.168.x.x"}
    ↓
iOS BluetoothProvisioningService
    ↓ didUpdateValueFor characteristic
    ↓ Parse JSON → ipCallback?(ip)
    ↓
iOS AppDelegate
    ↓ bluetoothChannel.invokeMethod("onIpAddressReceived", ...)
    ↓
Flutter NativeBluetoothService
    ↓ _channel.setMethodCallHandler
    ↓ _ipAddressController.add(ipAddress)
    ↓
Flutter BluetoothSetupPage
    ↓ _ipSubscription.listen
    ↓ saveIpAddress(ipAddress)
    ↓ _showSuccessDialog(ipAddress)
```

## Wie es funktioniert

1. **App verbindet sich mit TurtleBot via BLE**
   - Scannt nach Service UUID: `12345678-1234-5678-1234-56789abcdef0`
   - Verbindet sich und entdeckt Charakteristiken

2. **App sendet WiFi-Credentials**
   - SSID → Characteristic `...def1` (Write)
   - Password → Characteristic `...def2` (Write)

3. **TurtleBot verbindet sich mit WiFi**
   - Python Script verwendet `wpa_cli` oder `nmcli`
   - Ermittelt eigene IP-Adresse

4. **TurtleBot sendet Status zurück**
   - Status Characteristic `...def3` (Read + Notify)
   - Sendet JSON: `{"status": "connected", "ip": "192.168.x.x"}`

5. **App empfängt IP-Adresse** ✅ (JETZT FIXED!)
   - iOS empfängt Notification
   - Leitet an Flutter weiter
   - Speichert IP in SharedPreferences
   - Zeigt Success-Dialog

6. **App verwendet IP für WebSocket**
   - Lädt gespeicherte IP beim Start
   - Verbindet sich über WiFi statt Bluetooth

## Testing

### Auf dem TurtleBot:
```bash
# Start BLE Provisioning Service
sudo systemctl start bluetooth-provisioning
sudo systemctl status bluetooth-provisioning

# Check logs
sudo journalctl -u bluetooth-provisioning -f
```

### In der Flutter App:
1. Öffne Settings → Bluetooth WiFi Setup
2. Klicke "Scan for Devices"
3. Wähle "TurtleBot3-Provisioning"
4. Gib WiFi-Credentials ein
5. Klicke "Send to Robot"
6. **Warte auf IP-Adresse** (sollte jetzt erscheinen!)

### Erwartete Logs:

**iOS Console:**
```
🔵 BLE Native: Connected to TurtleBot3-Provisioning
✅ BLE Native: Found provisioning service
✅ BLE Native: Found SSID characteristic
✅ BLE Native: Found Password characteristic
✅ BLE Native: Found Status characteristic
✅ BLE Native: Write successful for ...def1
✅ BLE Native: Write successful for ...def2
🔵 BLE Native: Received status: {"status":"connecting"}
🔵 BLE Native: Extracted status: connecting
🔵 BLE: Notifying Flutter about status: connecting
🔵 BLE Native: Received status: {"status":"connected","ip":"192.168.1.100"}
🔵 BLE Native: Extracted IP: 192.168.1.100
🔵 BLE Native: Extracted status: connected
🔵 BLE: Notifying Flutter about IP: 192.168.1.100
🔵 BLE: Notifying Flutter about status: connected
```

**Flutter Console:**
```
🔵 BLE: Received status from native: {status: connecting}
🔵 BLE: Received IP from native: 192.168.1.100
🔵 BLE: Received status from native: {status: connected}
💾 Saved IP address to local storage: 192.168.1.100
```

**TurtleBot Logs:**
```
[INFO] BLE Server started and advertising!
[INFO] SSID received: YourWiFiName
[INFO] Password received (length=...)
[INFO] Connecting to Wi-Fi: YourWiFiName
[INFO] Provisioning complete: YourWiFiName -> 192.168.1.100
```

## Wichtige UUIDs (müssen übereinstimmen!)

| Characteristic | UUID | Type | Roboter | App |
|---|---|---|---|---|
| Service | `12345678-1234-5678-1234-56789abcdef0` | - | ✅ | ✅ |
| SSID | `12345678-1234-5678-1234-56789abcdef1` | Write | ✅ | ✅ |
| Password | `12345678-1234-5678-1234-56789abcdef2` | Write | ✅ | ✅ |
| Status | `12345678-1234-5678-1234-56789abcdef3` | Read+Notify | ✅ | ✅ |

## Nächste Schritte

1. ✅ iOS Implementierung fixed
2. ⚠️ Android Implementierung fehlt noch (MainActivity.kt)
3. ⚠️ Testen mit echtem TurtleBot

## Android TODO

Für Android muss eine ähnliche Implementierung in `MainActivity.kt` erstellt werden:
- BLE Central Manager
- Scan für Service UUID
- Connect zu Device
- Write SSID/Password
- Subscribe zu Status Notifications
- Parse JSON und sende Callbacks an Flutter

Die Flutter-Seite ist bereits vorbereitet und sollte mit Android funktionieren, sobald die native Implementierung fertig ist.
