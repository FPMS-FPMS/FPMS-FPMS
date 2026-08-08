#!/usr/bin/env bash
# Stage 00 - base system: identity, time, discovery, broker, tools.
set -euo pipefail
echo "--- 00-base-system"

export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq --no-install-recommends \
    ca-certificates curl wget gnupg lsb-release locales tzdata \
    sudo openssh-server \
    avahi-daemon avahi-utils libnss-mdns \
    systemd-timesyncd fake-hwclock \
    mosquitto mosquitto-clients \
    iw wireless-tools network-manager \
    usbutils v4l-utils udev \
    python3 python3-pip python3-venv \
    cloud-guest-utils e2fsprogs parted \
    jq git vim-tiny less htop

# --- locale and time --------------------------------------------------------
locale-gen en_US.UTF-8 >/dev/null
update-locale LANG=en_US.UTF-8

# UTC, fixed. Nothing in the FPMS code does timezone conversion - timestamps
# are unix seconds everywhere and the UI localises. A local timezone here just
# makes journal correlation across the rover and the laptop harder.
ln -sf /usr/share/zoneinfo/UTC /etc/localtime
echo "Etc/UTC" > /etc/timezone

# The board has NO RTC backup battery: it boots to the filesystem epoch, and
# without this every timestamp is 1970 until the network comes up. TLS to the
# cloud ingest cannot validate a certificate with a 1970 clock.
#
# fake-hwclock at least makes time monotonic across reboots when there is no
# NTP reachable, which on a competition floor is most of the time.
systemctl enable systemd-timesyncd fake-hwclock >/dev/null 2>&1 || true

# DELIBERATELY NOT ENABLED: systemd-time-wait-sync. Making any FPMS unit wait
# for a synchronised clock would delay STOP authority at boot, which
# contradicts the one ordering guarantee the stack actually makes.

# --- identity ---------------------------------------------------------------
echo "${FPMS_HOSTNAME}" > /etc/hostname
if ! grep -qE "^127\.0\.1\.1[[:space:]]+${FPMS_HOSTNAME}" /etc/hosts; then
    sed -i '/^127\.0\.1\.1[[:space:]]/d' /etc/hosts
    echo "127.0.1.1 ${FPMS_HOSTNAME}" >> /etc/hosts
fi

# avahi is what makes fpms-pi.local resolve, and it is assumed by the console,
# rosbridge, Foxglove and every deploy script in this project. NOTHING in the
# repository ever installed it - that gap is why "the rover's IP has changed
# more than seven times and every note that wrote one down was wrong the next
# day" was a recurring problem rather than a solved one.
systemctl enable avahi-daemon >/dev/null 2>&1 || true

# Do NOT disable IPv6. The Pi's mDNS has historically answered over IPv6
# link-local, and at least one resolver on the operator laptop depended on it.
sed -i 's/^hosts:.*/hosts: files mdns4_minimal [NOTFOUND=return] dns mdns4/' \
    /etc/nsswitch.conf 2>/dev/null || true

# --- the ubuntu user --------------------------------------------------------
#
# The name and home are NOT configurable in practice. Every unit hardcodes
# User=ubuntu and /home/ubuntu, and three Python files hardcode absolute paths
# under it with no override at all (the teleop origin anchor, read by missions
# and written by teleop, and the YOLO directory default).
if ! id -u "${FPMS_USER}" >/dev/null 2>&1; then
    useradd -m -s /bin/bash "${FPMS_USER}"
fi
usermod -aG sudo,dialout,video,plugdev,audio "${FPMS_USER}"

# Passwordless sudo: the deploy tooling (deploy_rover.py) requires it.
cat > /etc/sudoers.d/90-fpms-user <<EOF
${FPMS_USER} ALL=(ALL) NOPASSWD:ALL
EOF
chmod 0440 /etc/sudoers.d/90-fpms-user
visudo -cf /etc/sudoers.d/90-fpms-user >/dev/null

# No password is ever set. Login is by key only (below). A baked default
# password on a fleet image is a credential leak the moment one image is shared.
passwd -l "${FPMS_USER}" >/dev/null 2>&1 || true

# --- ssh --------------------------------------------------------------------
#
# HANDOFF.md records that the previously-used shared password should be treated
# as exposed and rotated, and that the same secret was reused for the dashboard
# login. Key-only closes that.
#
# NOTE FOR WHOEVER RUNS deploy_rover.py: it authenticates with a PASSWORD today
# (paramiko connect(..., password=...)). discovery.py already has a key_path
# parameter with no caller. Those tools need to be pointed at a key, or they
# break against this image on day one. That is a deliberate, visible tradeoff.
mkdir -p /etc/ssh/sshd_config.d
cat > /etc/ssh/sshd_config.d/10-fpms.conf <<'EOF'
# FPMS-OS ssh policy.
PasswordAuthentication no
PermitRootLogin no
KbdInteractiveAuthentication no
PubkeyAuthentication yes
EOF

# Host keys are NOT generated here. fpms-firstboot generates them, so every
# flashed board gets its own -- a shared host key across a fleet is a real
# problem and it also makes every laptop's known_hosts conflict.
rm -f /etc/ssh/ssh_host_*

install -d -m 0700 -o "${FPMS_USER}" -g "${FPMS_USER}" "${FPMS_HOME}/.ssh"
touch "${FPMS_HOME}/.ssh/authorized_keys"
chmod 0600 "${FPMS_HOME}/.ssh/authorized_keys"
chown -R "${FPMS_USER}:${FPMS_USER}" "${FPMS_HOME}/.ssh"

systemctl enable ssh >/dev/null 2>&1 || true

# --- mosquitto --------------------------------------------------------------
# Config comes from the overlay (stage 50). The password file is generated at
# first boot. Create the file now so mosquitto does not refuse to start on a
# missing password_file before firstboot has run.
touch /etc/mosquitto/fpms.passwd
chmod 0600 /etc/mosquitto/fpms.passwd
chown mosquitto:mosquitto /etc/mosquitto/fpms.passwd 2>/dev/null || true
systemctl enable mosquitto >/dev/null 2>&1 || true

# --- directories ------------------------------------------------------------
install -d -m 0755 /etc/fpms
install -d -m 0755 -o "${FPMS_USER}" -g "${FPMS_USER}" /var/lib/fpms
install -d -m 0755 -o "${FPMS_USER}" -g "${FPMS_USER}" "${FPMS_HOME}/yolo"

# --- ModemManager -----------------------------------------------------------
# It probes unknown serial devices by toggling handshake lines. On this board
# RTS drives the ESP32-S3's EN pin, so a probe is a HARDWARE RESET of the drive
# board, costing a 90-225 second re-link. The udev rules mark both CP2102s
# ID_MM_DEVICE_IGNORE, but the surest fix is not to have it installed at all -
# nothing on this rover is a modem.
apt-get purge -y -qq modemmanager >/dev/null 2>&1 || true

echo "--- 00-base-system OK"
