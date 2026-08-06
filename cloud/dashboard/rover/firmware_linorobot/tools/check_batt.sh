#!/bin/bash
# Restart teleop with the BatteryState fix and confirm voltage reaches MQTT,
# which is what the dashboard actually reads.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
S systemctl restart fpms-teleop
sleep 18

set -a; . /etc/fpms/config.env; set +a
T="${FPMS_THING_NAME:-rover2}"
echo "=== teleop log ==="
journalctl -u fpms-teleop -n 5 --no-pager | cut -c1-140 | tail -5

echo "=== MQTT drive telemetry (what the dashboard receives) ==="
timeout 30 mosquitto_sub -h 127.0.0.1 -u "$FPMS_MQTT_USER" -P "$FPMS_MQTT_PASS" \
    -t "fpms/$T/telemetry/drive" -C 1 > /tmp/drv.json
python3 - <<'EOF'
import json
d = json.load(open("/tmp/drv.json"))
for k in ("battery_v", "battery_raw", "battery_stale", "battery_low",
          "battery_age_s", "link_ok"):
    print(f"  {k} = {d.get(k)}")
EOF
echo "=== BATT CHECK DONE ==="
