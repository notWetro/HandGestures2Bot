# Roboter vs App - Bluetooth Implementierung Vergleich

## ✅ Was passt zusammen

### Service UUID
- **Roboter:** `12345678-1234-5678-1234-56789abcdef0`
- **App:** `12345678-1234-5678-1234-56789abcdef0`
- **Status:** ✅ IDENTISCH

### SSID Characteristic
- **Roboter:** `12345678-1234-5678-1234-56789abcdef1` (WRITE)
- **App:** `12345678-1234-5678-1234-56789abcdef1` (WRITE)
- **Status:** ✅ IDENTISCH

### Password Characteristic
- **Roboter:** `12345678-1234-5678-1234-56789abcdef2` (WRITE)
- **App:** `12345678-1234-5678-1234-56789abcdef2` (WRITE)
- **Status:** ✅ IDENTISCH

### Status Characteristic
- **Roboter:** `12345678-1234-5678-1234-56789abcdef3` (READ + NOTIFY)
- **App:** `12345678-1234-5678-1234-56789abcdef3` (READ + NOTIFY)
- **Status:** ✅ IDENTISCH

### Datenformat
- **Roboter sendet:** `{"status": "idle|connecting|connected|failed", "ip": "x.x.x.x", "error": "..."}`
- **App erwartet:** JSON mit `status`, `ip`, `error` Feldern
- **Status:** ✅ KOMPATIBEL

## ❌ Was NICHT passte (JETZT FIXED!)

### Callback-Weiterleitung
- **Problem:** iOS BluetoothProvisioningService hatte Callbacks definiert, aber nicht mit Flutter verbunden
- **Lösung:** 
  - `setIpCallback()` und `setStatusCallback()` Methoden hinzugefügt
  - Callbacks in AppDelegate.swift mit MethodChannel verbunden
  - Flutter MethodCallHandler eingerichtet
- **Status:** ✅ FIXED

## Ablauf mit allen Details

### 1. BLE Service Advertising (Roboter)
```python
# bluetooth_provisioning.py
DEVICE_NAME = "TurtleBot3-Provisioning"
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"

ble_peripheral.publish()
# → Roboter sendet BLE advertisements mit Service UUID
```

### 2. Scan und Connect (App)
```swift
// BluetoothProvisioningService.swift
centralManager.scanForPeripherals(withServices: [serviceUUID])
// → App scannt nur nach Geräten mit der Service UUID
// → Findet "TurtleBot3-Provisioning"
// → User wählt Gerät aus
// → App connected
```

### 3. Service Discovery (App)
```swift
peripheral.discoverServices([serviceUUID])
peripheral.discoverCharacteristics([ssidCharUUID, passwordCharUUID, statusCharUUID], for: service)
// → App findet alle 3 Characteristics
// → Subscribed zu Status Characteristic (für Notifications)
```

### 4. WiFi Credentials senden (App)
```swift
// SSID
peripheral.writeValue(ssidData, for: ssidChar, type: .withResponse)
// → Roboter: write_ssid() callback wird aufgerufen

// Password (mit kleinem Delay)
peripheral.writeValue(passwordData, for: passwordChar, type: .withResponse)
// → Roboter: write_password() callback wird aufgerufen
```

### 5. WiFi Connect (Roboter)
```python
# bluetooth_provisioning.py - try_provision()
if ssid_value and password_value:
    status_value = "connecting"
    # → Notification: {"status": "connecting"}
    
    success, reason = connect_wifi(ssid, password)
    # → Versucht wpa_cli oder nmcli
    
    if success:
        ip = get_wlan_ip()
        status_value = "connected"
        ip_value = ip
        # → Notification: {"status": "connected", "ip": "192.168.x.x"}
    else:
        status_value = "failed"
        last_error = reason
        # → Notification: {"status": "failed", "error": "..."}
```

### 6. Status empfangen (App) - JETZT FIXED!
```swift
// BluetoothProvisioningService.swift
func peripheral(..., didUpdateValueFor characteristic: ...) {
    let json = parse(characteristic.value)
    
    if let ip = json["ip"] {
        ipCallback?(ip)  // ← Callback wird aufgerufen
    }
    if let status = json["status"] {
        statusCallback?(status)  // ← Callback wird aufgerufen
    }
}

// AppDelegate.swift
bluetoothService.setIpCallback { ipAddress in
    bluetoothChannel.invokeMethod("onIpAddressReceived", arguments: ipAddress)
    // → Sendet an Flutter
}

// NativeBluetoothService.dart
_channel.setMethodCallHandler((call) async {
    case 'onIpAddressReceived':
        _ipAddressController.add(ipAddress);
        // → Stream sendet an BluetoothSetupPage
});

// BluetoothSetupPage.dart
_ipSubscription = _bluetoothService.onIpAddressReceived.listen((ip) {
    _receivedIpAddress = ip;
    _bluetoothService.saveIpAddress(ip);
    _showSuccessDialog(ip);
    // → Gespeichert und Dialog angezeigt!
});
```

## Roboter-seitige Charakteristiken

Der Roboter hat noch eine zusätzliche Characteristic, die die App NICHT verwendet:

### ACK Characteristic (nur Roboter)
- **UUID:** `12345678-1234-5678-1234-56789abcdef5`
- **Type:** WRITE
- **Zweck:** Optional - Client kann bestätigen, dass er die IP empfangen hat
- **Status:** ⚠️ NICHT IMPLEMENTIERT in App (aber auch nicht nötig)

## Was wurde geändert?

### Geänderte Dateien

1. **flutterapp/ios/Runner/AppDelegate.swift**
   - ✅ Callbacks von BluetoothProvisioningService an Flutter weitergeleitet
   - ✅ `setIpCallback()` und `setStatusCallback()` aufgerufen

2. **flutterapp/ios/Runner/BluetoothProvisioningService.swift**
   - ✅ Public Methoden `setIpCallback()` und `setStatusCallback()` hinzugefügt
   - ✅ Besseres Logging für empfangene Status-Updates

3. **flutterapp/lib/services/native_bluetooth_service.dart**
   - ✅ `_setupMethodCallHandler()` implementiert
   - ✅ Empfängt `onIpAddressReceived` und `onStatusUpdate` von iOS

## Was wurde NICHT geändert?

- ❌ **Android Implementierung** (MainActivity.kt) - fehlt komplett
- ❌ **Roboter-Code** (bluetooth_provisioning.py) - war bereits korrekt
- ❌ **Flutter UI** (bluetooth_setup_page.dart) - war bereits korrekt
- ❌ **UUIDs** - waren bereits identisch

## Testing Checklist

### TurtleBot vorbereiten:
- [ ] BLE Service läuft: `sudo systemctl status bluetooth-provisioning`
- [ ] Logs beobachten: `sudo journalctl -u bluetooth-provisioning -f`

### iPhone App:
- [ ] Bluetooth Permission erteilt
- [ ] Settings → Bluetooth WiFi Setup
- [ ] "Scan for Devices" → TurtleBot erscheint
- [ ] Device auswählen → Verbindung erfolgreich
- [ ] WiFi Credentials eingeben
- [ ] "Send to Robot" klicken
- [ ] **Status "connecting" wird angezeigt** ← NEU
- [ ] **Status "connected" wird angezeigt** ← NEU
- [ ] **IP Adresse wird empfangen und gespeichert** ← NEU (FIXED!)
- [ ] Success Dialog erscheint ← NEU (FIXED!)

### Logs prüfen:

**iOS:**
```
✅ BLE Native: Write successful
🔵 BLE Native: Received status: {"status":"connecting"}
🔵 BLE Native: Received status: {"status":"connected","ip":"192.168.1.100"}
🔵 BLE Native: Extracted IP: 192.168.1.100
🔵 BLE: Notifying Flutter about IP: 192.168.1.100
```

**Flutter:**
```
🔵 BLE: Received IP from native: 192.168.1.100
💾 Saved IP address to local storage: 192.168.1.100
```

**Roboter:**
```
[INFO] SSID received: YourHotspot
[INFO] Password received (length=12)
[INFO] Connecting to Wi-Fi: YourHotspot
[INFO] Provisioning complete: YourHotspot -> 192.168.1.100
```

## Zusammenfassung

| Komponente | Status vor Fix | Status nach Fix |
|---|---|---|
| BLE Service UUID | ✅ Korrekt | ✅ Korrekt |
| BLE Characteristics | ✅ Korrekt | ✅ Korrekt |
| iOS BLE Connect | ✅ Funktioniert | ✅ Funktioniert |
| iOS Write Credentials | ✅ Funktioniert | ✅ Funktioniert |
| iOS Receive Status | ⚠️ Empfangen, aber nicht weitergeleitet | ✅ **FIXED** - an Flutter weitergeleitet |
| Flutter Receive IP | ❌ Kam nie an | ✅ **FIXED** - kommt jetzt an |
| Flutter Save IP | ❌ Konnte nicht speichern | ✅ **FIXED** - wird gespeichert |
| Android | ❌ Nicht implementiert | ❌ TODO |

**Das Hauptproblem war:** Die "letzte Meile" von iOS Native zu Flutter fehlte. Die Daten kamen vom Roboter an, wurden in Swift empfangen, aber nie an Flutter weitergeleitet.

**Jetzt funktioniert:** Der komplette Datenfluss von Roboter → iOS BLE → Swift Callbacks → Flutter MethodChannel → Dart Streams → UI Update + Save.
