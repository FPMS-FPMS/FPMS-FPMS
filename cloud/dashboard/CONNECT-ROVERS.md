# Connecting the Orange Pi 5B rovers

End state: each rover runs a systemd service that publishes telemetry to the
Mosquitto broker on the HQ laptop, and the dashboard renders it live — locally
and through the permanent public URL.

```
Orange Pi 5B  ──MQTT 1883──▶  Mosquitto on HQ laptop (192.168.0.27)
                                      │
                                      ▼
                          FPMS dashboard (:8000)
                                      │
                                      ▼
              https://fpms.aryan0419wadhawan.workers.dev
```

## Prerequisites

1. **Broker configured for the LAN.** Mosquitto ships loopback-only, so a rover
   cannot reach it out of the box. Run once, in an **Administrator** PowerShell:

   ```powershell
   & 'C:\Users\ruchi\FPMS-FPMS\cloud\dashboard\scripts\Setup-Mosquitto.ps1' -Password '<broker-password>'
   ```

   It prints `MODE=secure` (LAN + auth), `MODE=anonymous` (LAN, no auth), or
   `MODE=reverted`. Anything other than `secure` is worth a second look, and
   `reverted` means rovers still cannot connect.

2. **Rover on the same WiFi as the laptop.** The Pi's MQTT connection is plain
   LAN traffic — it does not go through Cloudflare. The tunnel is only for
   *viewing* the dashboard from elsewhere.

3. **Laptop IP.** Currently `192.168.0.27` (Wi-Fi). This is baked into the Pi's
   config at provisioning time, so re-provision if the laptop's IP changes.
   A DHCP reservation on the router avoids that entirely.

## Provisioning a rover

**From the dashboard:** Devices tab → find the Pi → provide username and
password → Provision.

**Or via the API:**

```bash
curl -X POST http://localhost:8000/api/discovery/provision \
  -H 'Content-Type: application/json' \
  -d '{"ip":"192.168.0.42","username":"orangepi","password":"<pi-password>","thing_name":"rover1"}'
```

Defaults to the LAN broker. Pass `"use_aws": true` to route through AWS IoT Core
with TLS instead, or `"broker_host": "..."` to override the target.

The response reports which transport was used:

```json
{"thing":"rover1","transport":"lan-mqtt","iot_endpoint":"192.168.0.27","mqtt_port":1883}
```

## What lands on the Pi

| Path | Purpose |
|---|---|
| `/etc/fpms/config.env` | thing name, broker host/port, credentials (chmod 600) |
| `/usr/local/bin/fpms-publisher` | Python publisher loop |
| `/etc/systemd/system/fpms-publisher.service` | `Restart=always`, starts at boot |

The publisher retries the initial connection every 5s, because a rover usually
boots before the HQ laptop is reachable. On connect it publishes a
`fpms/<thing>/events/online` event, then pose telemetry twice a second.

**The payload is still a placeholder.** Replace the loop body in
`build_provisioner_script()` (`backend/discovery.py`) with real reads from your
ldlidar / thermal / camera stack. Topics the dashboard already subscribes to:

```
fpms/<thing>/telemetry/lidar
fpms/<thing>/telemetry/camera     # base64 JPEG frames, event-based
fpms/<thing>/telemetry/thermal
fpms/<thing>/telemetry/pose
fpms/<thing>/events/#
```

## Verifying

On the Pi:

```bash
systemctl status fpms-publisher
journalctl -u fpms-publisher -f
```

On the laptop — confirm the broker accepted it:

```powershell
Get-Content "$env:LOCALAPPDATA\FPMS\launch.log" -Tail 20
```

Then open the dashboard; the rover's panels should leave the "waiting" state.

## Troubleshooting

| Symptom | Cause |
|---|---|
| Publisher logs `connect failed`, retries forever | Broker not on the LAN — check `MODE=secure`, and that the firewall rule for TCP 1883 exists |
| Connects then immediately drops | Wrong credentials. The Pi's `/etc/fpms/config.env` must match the broker's password file |
| Dashboard shows nothing but the Pi looks healthy | Thing name mismatch — panels are keyed by `<thing>` |
| Worked yesterday, dead today | Laptop's DHCP lease changed its IP; re-provision or set a reservation |

## Security note

The broker is firewalled to **private** networks only and never exposed through
the Cloudflare tunnel — only the dashboard UI is. Anyone with the broker
credentials and LAN access can publish telemetry, so treat that password as
sensitive; it's stored on every provisioned rover.
