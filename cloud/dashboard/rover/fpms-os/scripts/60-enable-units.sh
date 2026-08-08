#!/usr/bin/env bash
# Stage 60 - enable, mask, and deliberately-not-enable.
#
# EVERY BOOT UNIT IS ENABLED HERE. fpms-missions was once found DISABLED on the
# running Pi, so the rover came back from a power cycle with no mission
# executor and nothing said so. An image is the right place to make that
# impossible.
set -euo pipefail
echo "--- 60-enable-units"

systemctl daemon-reload 2>/dev/null || true

BOOT_UNITS=(
    fpms-firstboot.service          # must run before anything reads config.env
    fpms-cored.service              # stop authority, before anything that moves
    fpms-rover-agent.service
    fpms-lidar-ros.service
    fpms-tf.service
    fpms-map-anchor.service
    fpms-odom-tf.service
    fpms-teleop.service
    fpms-missions.service
    fpms-uros-supervisor.service
    fpms-wifi-powersave-hold.service
    micro-ros-agent.service
    # The NPU layer. fpms-npud owns the RKNN runtime so that a slow or wedged
    # inference cannot stall the camera pump that also feeds the fire detector.
    # fpms-npu-tune runs in REPORT mode -- it changes nothing unless an operator
    # passes --apply -- and exists so thermal throttling is visible rather than
    # silently eating the latency budget.
    fpms-npud.service
    fpms-npu-tune.service
    fpms-ros-publishers.target
    fpms-rosbridge.service
    fpms-console.service
    fpms-selftest.service
)

for u in "${BOOT_UNITS[@]}"; do
    if systemctl enable "$u" >/dev/null 2>&1; then
        echo "    enabled  $u"
    else
        echo "    FAILED to enable $u" >&2
        exit 1
    fi
done

# --- masked, permanently ----------------------------------------------------
#
# Both can write /cmd_vel. Two writers is not a race the mission executor can
# win, or even reliably detect in time.
#
# MASKED, NOT DISABLED. `disable` only removes the WantedBy symlink; anything
# that pulls the unit in by name still starts it. Masking points it at
# /dev/null so no path can start it at all.
#
# They are SHIPPED so they can be masked: masking a unit that does not exist
# is not the same guarantee, because a restore from backup or a copy from the
# old Pi would put an unmasked writer back on the robot.
for u in fpms-ros-tunnel.service fpms-rtos-follower.service; do
    systemctl disable "$u" >/dev/null 2>&1 || true
    systemctl mask "$u"    >/dev/null 2>&1 || true
    state="$(systemctl is-enabled "$u" 2>/dev/null || true)"
    [ "$state" = "masked" ] || { echo "FATAL: $u is '$state', not masked" >&2; exit 1; }
    echo "    masked   $u"
done

# --- installed but deliberately NOT enabled ---------------------------------
#
# fpms-ros-settle: 75 seconds of sleep mitigating the SHM bug that
#   /etc/fpms/fastdds_udp_only.xml now removes at the cause. Kept as a
#   fallback, because removing a mitigation for a bug not yet confirmed fixed
#   on hardware is a mistake this project has made before.
#
# fpms-nav2, fpms-slam-*: three preconditions are unmet -- the LiDAR mount
#   transform is unmeasured, LIDAR_ROTATION_SIGN is unverified, and a saved
#   map cannot ship in an image. Their [Install] sections are absent so they
#   cannot drift into the boot path by accident.
systemctl disable fpms-ros-settle.service >/dev/null 2>&1 || true
echo "    installed-not-enabled: fpms-ros-settle, fpms-nav2, fpms-slam-mapping, fpms-slam-localization"

# --- verify --------------------------------------------------------------
echo "--- enable state:"
for u in "${BOOT_UNITS[@]}"; do
    printf '    %-36s %s\n' "$u" "$(systemctl is-enabled "$u" 2>/dev/null || echo '?')"
done

echo "--- 60-enable-units OK"
