# Remote Supervision — Local Network Browser Display

Access the Raspberry Pi's full desktop (RViz, terminals, everything) from any device on the same
local hotspot. No internet required. Supervisor only needs a browser.

**Stack:** x11vnc (shares the live X11 display) + noVNC (browser-based VNC client over WebSocket)

---

## One-Time Setup on the Raspberry Pi

### Step 1 — Disable Wayland

x11vnc only works on X11. Ubuntu 24.04 defaults to Wayland — disable it:

```bash
sudo nano /etc/gdm3/custom.conf
```

Set in the `[daemon]` section:
```ini
[daemon]
WaylandEnable=false
```

Reboot:
```bash
sudo reboot
```

---

### Step 2 — Install x11vnc and noVNC

```bash
sudo apt update
sudo apt install -y x11vnc novnc python3-websockify
```

---

### Step 3 — Set a VNC password

```bash
mkdir -p ~/.vnc
x11vnc -storepasswd ~/.vnc/passwd
```

Enter a password when prompted — you will use this in the browser.

---

### Step 4 — Create systemd service for x11vnc

```bash
sudo nano /etc/systemd/system/x11vnc.service
```

```ini
[Unit]
Description=x11vnc VNC Server
After=graphical.target

[Service]
Type=simple
User=slamrobot
ExecStart=/usr/bin/x11vnc -display :0 -auth guess -forever -noxdamage -repeat -rfbauth /home/slamrobot/.vnc/passwd -rfbport 5900 -shared
Restart=on-failure
RestartSec=3

[Install]
WantedBy=graphical.target
```

---

### Step 5 — Create systemd service for noVNC

```bash
sudo nano /etc/systemd/system/novnc.service
```

```ini
[Unit]
Description=noVNC WebSocket Proxy
After=x11vnc.service

[Service]
Type=simple
ExecStart=/usr/bin/websockify --web /usr/share/novnc/ 6080 localhost:5900
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

---

### Step 6 — Enable and start both services

```bash
sudo systemctl daemon-reload
sudo systemctl enable x11vnc novnc
sudo systemctl start x11vnc novnc
```

Verify both are running:
```bash
sudo systemctl status x11vnc
sudo systemctl status novnc
```

Both should show `active (running)`.

---

### Step 7 — Find the Pi's local IP

```bash
hostname -I
```

The first address (e.g. `192.168.x.x`) is the one the supervisor will use. You can also check:
```bash
ip addr show wlan0
```

---

## Connecting from the Supervisor Device

1. Connect supervisor device to the **same hotspot** as the Pi
2. Open any browser
3. Navigate to:
```
http://<PI_IP>:6080/vnc.html
```
Example: `http://192.168.1.45:6080/vnc.html`

4. Enter the VNC password from Step 3
5. Full Pi desktop is now visible — RViz, terminals, everything

Both services auto-start on every Pi reboot. Nothing extra needed after initial setup.

---

## Optional — Fix the Pi's IP so the URL never changes

If the hotspot assigns a different IP on each boot, set a static IP via NetworkManager:

```bash
# Replace values with your actual hotspot name and desired IP
sudo nmcli con mod "YourHotspotName" ipv4.addresses 192.168.1.100/24
sudo nmcli con mod "YourHotspotName" ipv4.gateway 192.168.1.1
sudo nmcli con mod "YourHotspotName" ipv4.method manual
sudo nmcli con up "YourHotspotName"
```

Your URL is then always: `http://192.168.1.100:6080/vnc.html`

To find the hotspot connection name:
```bash
nmcli con show
```

---

## Troubleshooting

| Problem | Check | Fix |
|---------|-------|-----|
| Browser shows blank / can't connect | `sudo systemctl status novnc` | Restart: `sudo systemctl restart novnc` |
| Black screen in browser | x11vnc started before login | Log into Pi desktop first, then `sudo systemctl restart x11vnc` |
| Wrong password | — | Re-run `x11vnc -storepasswd ~/.vnc/passwd` |
| IP changed | `hostname -I` on Pi | Set static IP (see above) or check new IP |
| Very laggy | Image quality setting in noVNC | Click the gear icon in the browser → lower quality or enable compression |
| Services not starting after reboot | Wayland re-enabled | Recheck `/etc/gdm3/custom.conf` — `WaylandEnable=false` must be set |
