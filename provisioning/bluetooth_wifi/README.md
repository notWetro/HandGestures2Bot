# Bluetooth Wi-Fi Provisioning (System-Level)

Purpose: Provide first-time Wi-Fi setup over Bluetooth Classic without touching
ROS2 nodes. Runs as a boot-time service, independent from `ros2 run ...` flows.

## How it works
- Listens on Bluetooth RFCOMM channel 1.
- On phone connect, sends `{ "status": "connected" }`.
- Expects JSON `{ "ssid": "<wifi>", "password": "<pass>" }` (newline-terminated).
- Calls `nmcli dev wifi connect <ssid> password <pass> ifname wlan0`.
- On success, reads IP via `netifaces.ifaddresses("wlan0")[AF_INET][0]["addr"]` and
  replies `{ "status": "connected", "ssid": "...", "ip": "..." }`.
- On failure, replies `{ "status": "failed", "reason": "..." }`.
- Closes the Bluetooth connection after provisioning.

## Run manually (for testing)
```bash
sudo python3 bluetooth_provisioning.py
```

## Systemd service (recommended)
Install the script to a root-owned path, e.g. `/opt/handgestures/provisioning/bluetooth_wifi/`.
Use the unit file in `../systemd/bluetooth-provisioning.service` and enable it:
```bash
sudo cp bluetooth_provisioning.py /opt/handgestures/provisioning/bluetooth_wifi/
sudo cp ../systemd/bluetooth-provisioning.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now bluetooth-provisioning.service
```

## Notes
- Requires NetworkManager (`nmcli`).
- Does not delete or restart existing Wi-Fi connections; safe when a base Wi-Fi is active.
- Uses Bluetooth Classic (RFCOMM), not BLE. Pair from the phone before provisioning.
- Keep this outside `ros2_ws` to avoid coupling with ROS build/launch.
