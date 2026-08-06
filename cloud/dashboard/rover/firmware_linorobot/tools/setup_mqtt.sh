#!/bin/bash
# Move the MQTT broker onto the Pi.
#
# WHY: the LiDAR currently travels Pi -> laptop broker -> Pi purely because the
# broker is off-board, putting a WiFi round trip on a Nav2-critical path. It was
# measured at 2.37 Hz with "MQTT lidar feed stale" warnings, against ~9.8 Hz when
# healthy. A local broker removes WiFi from that path entirely.
#
# A BRIDGE to the laptop broker is configured so the dashboard, which subscribes
# on the laptop, keeps working with no change at that end. Display traffic still
# crosses WiFi -- that is fine, it is not navigation-critical.
#
# Credentials are read out of the existing /etc/fpms/config.env and never leave
# the rover.
set -e
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }

LAPTOP_BROKER="${2:-192.168.137.1}"

echo "=== INSTALL ==="
S apt-get install -y mosquitto mosquitto-clients 2>&1 | tail -3

# shellcheck disable=SC1091
set -a; . /etc/fpms/config.env; set +a
MU="${FPMS_MQTT_USER:-fpms}"
MP="${FPMS_MQTT_PASS:-${FPMS_MQTT_PASSWORD:-}}"
echo "mqtt user resolved: ${MU} (password ${#MP} chars)"
if [ -z "$MP" ]; then
    echo "NO MQTT PASSWORD FOUND IN config.env - aborting before breaking auth"
    grep -oE '^FPMS_MQTT_[A-Z_]+' /etc/fpms/config.env || true
    exit 1
fi

echo "=== BACKUP CONFIG ==="
S cp /etc/fpms/config.env /etc/fpms/config.env.bak.$(date +%s)

echo "=== BROKER CONFIG ==="
S bash -c "printf '%s\n' \
'listener 1883' \
'allow_anonymous false' \
'password_file /etc/mosquitto/fpms.passwd' \
'' \
'# Bridge to the laptop broker so the dashboard keeps receiving telemetry.' \
'# Nav-critical traffic is served locally; only display traffic crosses WiFi.' \
'connection laptop-bridge' \
'address ${LAPTOP_BROKER}:1883' \
'topic # both 0' \
'remote_username ${MU}' \
'remote_password ${MP}' \
'bridge_attempt_unsubscribe false' \
'start_type automatic' \
'restart_timeout 5' \
'cleansession true' \
> /etc/mosquitto/conf.d/fpms.conf"
S chmod 600 /etc/mosquitto/conf.d/fpms.conf

echo "=== BROKER AUTH ==="
S bash -c "mosquitto_passwd -b -c /etc/mosquitto/fpms.passwd '${MU}' '${MP}'"
S chmod 600 /etc/mosquitto/fpms.passwd
S chown mosquitto:mosquitto /etc/mosquitto/fpms.passwd /etc/mosquitto/conf.d/fpms.conf

echo "=== RESTART BROKER ==="
S systemctl enable mosquitto 2>&1 | tail -1
S systemctl restart mosquitto
sleep 3
systemctl is-active mosquitto

echo "=== LOCAL PUBSUB SELFTEST ==="
timeout 8 mosquitto_sub -h 127.0.0.1 -u "$MU" -P "$MP" -t 'fpms/selftest' -C 1 &
SUBPID=$!
sleep 2
mosquitto_pub -h 127.0.0.1 -u "$MU" -P "$MP" -t 'fpms/selftest' -m 'ok'
wait $SUBPID && echo "LOCAL BROKER OK" || echo "LOCAL BROKER SELFTEST FAILED"

echo "=== POINT ROVER SERVICES AT LOCALHOST ==="
S sed -i 's/^FPMS_MQTT_HOST=.*/FPMS_MQTT_HOST=127.0.0.1/' /etc/fpms/config.env
grep '^FPMS_MQTT_HOST' /etc/fpms/config.env

S systemctl restart fpms-rover-agent
S systemctl restart fpms-lidar-ros
echo "=== SETTLING 40s ==="
sleep 40
journalctl -u fpms-lidar-ros -n 4 --no-pager | cut -c1-130 | tail -4
echo "=== MQTT DONE ==="
