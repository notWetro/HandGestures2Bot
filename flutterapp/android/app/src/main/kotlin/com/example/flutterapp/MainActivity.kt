package com.example.flutterapp

import android.Manifest
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodChannel
import org.json.JSONObject
import java.util.*

class MainActivity: FlutterActivity() {

    private val DANCE_CHANNEL = "dance_channel"
    private val CAMERA_CHANNEL = "camera_permission"
    private val BLUETOOTH_CHANNEL = "bluetooth_channel"
    private val CAMERA_PERMISSION_CODE = 101
    private val BLUETOOTH_PERMISSION_CODE = 102
    private var pendingResult: MethodChannel.Result? = null

    private var bluetoothGatt: BluetoothGatt? = null
    private var bluetoothMethodChannel: MethodChannel? = null
    private val handler = Handler(Looper.getMainLooper())
    private var currentScanCallback: ScanCallback? = null

    // UUIDs must match the Robot's Python implementation
    private val SERVICE_UUID = UUID.fromString("12345678-1234-5678-1234-56789abcdef0")
    private val SSID_UUID = UUID.fromString("12345678-1234-5678-1234-56789abcdef1")
    private val PASS_UUID = UUID.fromString("12345678-1234-5678-1234-56789abcdef2")
    private val STATUS_UUID = UUID.fromString("12345678-1234-5678-1234-56789abcdef3")

    lateinit var gestureRecognizer: GestureRecognizer


    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)

        gestureRecognizer = GestureRecognizer(this)


        // REGISTER FACTORY with 'this' (Activity) and 'binaryMessenger'
        flutterEngine
            .platformViewsController
            .registry
            .registerViewFactory(
                "my_camera_view",
                MyCameraViewFactory(this, flutterEngine.dartExecutor.binaryMessenger)
            )

        // Camera Permission Channel
        MethodChannel(flutterEngine.dartExecutor.binaryMessenger, CAMERA_CHANNEL).setMethodCallHandler { call, result ->
            if (call.method == "getCameraPermission") {
                if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
                    result.success(true)
                } else {
                    pendingResult = result
                    ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), CAMERA_PERMISSION_CODE)
                }
            } else {
                result.notImplemented()
            }
        }

        //Dance_Channel
        MethodChannel(
            flutterEngine.dartExecutor.binaryMessenger,
            DANCE_CHANNEL
        ).setMethodCallHandler { call, result ->

            val store = DanceStore.getInstance(this)

            when (call.method) {
                "saveDanceMoves" -> {
                    val json = call.arguments as? String
                    if (json != null) {
                        store.saveDanceMoves(json)
                        result.success(true)
                    } else {
                        result.error("ARG_ERROR", "JSON is null", null)
                    }
                }

                "loadDanceMoves" -> {
                    val json = store.loadDanceMoves()
                    result.success(json)
                }

                else -> result.notImplemented()
            }
        }

        // Gesture Channel
        MethodChannel(
            flutterEngine.dartExecutor.binaryMessenger,
            "gesture_channel"
        ).setMethodCallHandler { call, result ->

            when (call.method) {

                "deleteGesture" -> {
                    val name = call.arguments as? String
                    if (name != null) {
                        gestureRecognizer.deleteGesture(name)
                        Log.d("GESTURE", "Deleted gesture in realtime: $name")
                        result.success(true)
                    } else {
                        result.error("ARG_ERROR", "Gesture name is null", null)
                    }
                }

                else -> result.notImplemented()
            }
        }



        // Bluetooth Provisioning Channel
        bluetoothMethodChannel = MethodChannel(flutterEngine.dartExecutor.binaryMessenger, BLUETOOTH_CHANNEL)
        bluetoothMethodChannel?.setMethodCallHandler { call, result ->
            when (call.method) {
                "startScanning" -> startScan(result)
                "connectToDevice" -> {
                    val address = call.arguments as? String
                    if (address != null) connectToDevice(address, result)
                    else result.error("ARG_ERROR", "Address is null", null)
                }
                "sendWiFiCredentials" -> {
                    val ssid = call.argument<String>("ssid")
                    val password = call.argument<String>("password")
                    if (ssid != null && password != null) {
                        sendWifiCredentials(ssid, password, result)
                    } else {
                        result.error("ARG_ERROR", "SSID or Password is null", null)
                    }
                }
                "disconnect" -> disconnect(result)
                "stopScanning" -> stopScanning(result)
                "getConnectedDevices" -> getConnectedDevices(result)
                else -> result.notImplemented()
            }
        }
    }

    private fun startScan(result: MethodChannel.Result) {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }

        val missingPermissions = permissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (missingPermissions.isNotEmpty()) {
            pendingResult = result
            ActivityCompat.requestPermissions(this, missingPermissions.toTypedArray(), BLUETOOTH_PERMISSION_CODE)
            return
        }

        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val scanner = bluetoothManager.adapter.bluetoothLeScanner

        currentScanCallback = object : ScanCallback() {
            override fun onScanResult(callbackType: Int, scanResult: ScanResult) {
                val device = scanResult.device
                val name = device.name

                // Only notify Flutter if the device is a TurtleBot
                if (name != null && name.contains("TurtleBot", ignoreCase = true)) {
                    handler.post {
                        bluetoothMethodChannel?.invokeMethod("onDeviceFound", mapOf(
                            "name" to name,
                            "address" to device.address
                        ))
                    }
                }
            }
        }

        scanner.startScan(currentScanCallback)
        handler.postDelayed({
            currentScanCallback?.let {
                scanner.stopScan(it)
                currentScanCallback = null
            }
        }, 10000)
        result.success(true)
    }

    private fun stopScanning(result: MethodChannel.Result) {
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val scanner = bluetoothManager.adapter.bluetoothLeScanner
        currentScanCallback?.let { scanner.stopScan(it) }
        currentScanCallback = null
        result.success(true)
    }

    private fun getConnectedDevices(result: MethodChannel.Result) {
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val devices = bluetoothManager.getConnectedDevices(BluetoothProfile.GATT)
        val deviceList = devices
            .filter { it.name?.contains("TurtleBot", ignoreCase = true) == true }
            .map { device ->
                mapOf("name" to (device.name ?: "TurtleBot"), "address" to device.address)
            }
        result.success(deviceList)
    }

    private fun connectToDevice(address: String, result: MethodChannel.Result) {
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val device = bluetoothManager.adapter.getRemoteDevice(address)
        bluetoothGatt = device.connectGatt(this, false, gattCallback)
        result.success(true)
    }

    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                gatt.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                handler.post {
                    bluetoothMethodChannel?.invokeMethod("onStatusUpdate", mapOf("status" to "disconnected"))
                }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val service = gatt.getService(SERVICE_UUID)
                val statusChar = service?.getCharacteristic(STATUS_UUID)
                if (statusChar != null) {
                    gatt.setCharacteristicNotification(statusChar, true)
                    val descriptor = statusChar.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))
                    descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                    gatt.writeDescriptor(descriptor)
                }
                handler.post {
                    bluetoothMethodChannel?.invokeMethod("onStatusUpdate", mapOf("status" to "connected"))
                }
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            if (characteristic.uuid == STATUS_UUID) {
                val data = String(characteristic.value, Charsets.UTF_8)
                handleStatusUpdate(data)
            }
        }
    }

    private fun handleStatusUpdate(jsonStr: String) {
        Log.d("BLE_PROVISIONING", "Received status update from robot: $jsonStr")
        try {
            val json = JSONObject(jsonStr)
            val status = json.optString("status")
            val ip = json.optString("ip")
            handler.post {
                if (ip.isNotEmpty()) {
                    Log.d("BLE_PROVISIONING", "Extracted IP Address: $ip")
                    bluetoothMethodChannel?.invokeMethod("onIpAddressReceived", ip)
                }
                bluetoothMethodChannel?.invokeMethod("onStatusUpdate", mapOf("status" to status))
            }
        } catch (e: Exception) {
            Log.e("BLE_PROVISIONING", "Error parsing status JSON: ${e.message}")
        }
    }

    private fun sendWifiCredentials(ssid: String, pass: String, result: MethodChannel.Result) {
        val gatt = bluetoothGatt ?: run {
            result.error("NOT_CONNECTED", "Not connected to device", null)
            return
        }
        val service = gatt.getService(SERVICE_UUID)
        val ssidChar = service?.getCharacteristic(SSID_UUID)
        val passChar = service?.getCharacteristic(PASS_UUID)

        if (ssidChar != null && passChar != null) {
            ssidChar.value = ssid.toByteArray()
            gatt.writeCharacteristic(ssidChar)
            handler.postDelayed({
                passChar.value = pass.toByteArray()
                gatt.writeCharacteristic(passChar)
            }, 600)
            result.success(true)
        } else {
            result.error("CHAR_NOT_FOUND", "Provisioning characteristics not found", null)
        }
    }

    private fun disconnect(result: MethodChannel.Result) {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
        result.success(true)
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        val allGranted = grantResults.isNotEmpty() && grantResults.all { it == PackageManager.PERMISSION_GRANTED }

        when (requestCode) {
            CAMERA_PERMISSION_CODE -> {
                pendingResult?.success(allGranted)
                pendingResult = null
            }
            BLUETOOTH_PERMISSION_CODE -> {
                if (allGranted) {
                    pendingResult?.let { startScan(it) }
                } else {
                    pendingResult?.error("PERMISSION_DENIED", "Bluetooth permissions are required", null)
                }
                pendingResult = null
            }
        }
    }
}