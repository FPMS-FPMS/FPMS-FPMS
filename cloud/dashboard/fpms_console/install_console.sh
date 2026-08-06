#!/usr/bin/env bash
# ===========================================================================
# FPMS operator console — installer. RUN THIS ON THE PI.
#
#   scp -r fpms_console ubuntu@fpms-pi.local:~/          # or the deploy helper
#   ssh ubuntu@fpms-pi.local
#   cd ~/fpms_console && sudo ./install_console.sh
#
#   sudo ./install_console.sh --status     # check without changing anything
#   sudo ./install_console.sh --dry-run    # print every action, do none
#
# It installs TWO services and ONE config file:
#   fpms-rosbridge.service   ROS 2 <-> WebSocket, :9090, whitelisted
#   fpms-console.service     serves the UI on :8090, mirrors the plan to ROS
#   /etc/fpms/rosbridge_params.yaml   the topic whitelist (the safety part)
#
# It TOUCHES NOTHING ELSE. It does not restart fpms-missions, fpms-teleop,
# fpms-cored, micro-ros-agent, the rover agent, or either Foxglove unit. The
# Foxglove bridge stays installed and running as a debug tool; this console
# does not replace it, it just is not it.
# ===========================================================================
set -uo pipefail

DRY=0; STATUS=0
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=1 ;;
    --status)  STATUS=1 ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "unknown option $a"; exit 2 ;;
  esac
done

SRC="$(cd "$(dirname "$0")" && pwd)"
DEST=/home/ubuntu/fpms_console
UNITS=/etc/systemd/system
ETC=/etc/fpms
OWNER="-o ubuntu -g ubuntu"   # install(1) takes -o USER -g GROUP, not user:group

say()  { printf '%s\n' "$*"; }
head_() { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }
ok()   { printf '  \033[1;32mOK\033[0m   %s\n' "$*"; }
bad()  { printf '  \033[1;31mFAIL\033[0m %s\n' "$*"; FAILED=1; }
warn() { printf '  \033[1;33mWARN\033[0m %s\n' "$*"; }
run()  { if [ "$DRY" = 1 ]; then say "  would run: $*"; else "$@"; fi; }
FAILED=0

# --------------------------------------------------------------- status only
if [ "$STATUS" = 1 ]; then
  head_ "services"
  systemctl --no-pager --plain status fpms-rosbridge.service fpms-console.service 2>&1 | \
    grep -E 'Loaded|Active|●|^\s+fpms' | sed 's/^/  /'
  head_ "listening ports (expect 9090 rosbridge, 8090 console)"
  ss -ltn 2>/dev/null | grep -E ':(8090|9090)' | sed 's/^/  /' || say "  NEITHER PORT IS OPEN"
  head_ "the whitelist actually in force"
  # set +u: setup.bash dereferences unset variables and would EXIT this script
  # under `set -u`, skipping every check below it.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash 2>/dev/null
  set -u
  ROS_DOMAIN_ID=20 timeout 25 ros2 param get /rosbridge_websocket topics_glob 2>&1 | sed 's/^/  /'
  head_ "reachability from this Pi"
  curl -fsS -m 4 "http://127.0.0.1:8090/healthz" 2>/dev/null | sed 's/^/  /' || say "  console /healthz did not answer"
  exit 0
fi

if [ "$(id -u)" != 0 ] && [ "$DRY" = 0 ]; then
  echo "run me with sudo:  sudo ./install_console.sh"; exit 1
fi

# ------------------------------------------------------- 1. rosbridge is present
head_ "1. rosbridge_server"
if [ -x /opt/ros/humble/lib/rosbridge_server/rosbridge_websocket ]; then
  ok "already installed (ros-humble-rosbridge-server)"
else
  warn "not installed — installing ros-humble-rosbridge-suite from apt"
  run apt-get update -qq
  if ! run apt-get install -y ros-humble-rosbridge-suite; then
    bad "apt install failed. Fallback: build rosbridge_suite from source into"
    bad "  ~/ros2_ws (git clone -b ros2 https://github.com/RobotWebTools/rosbridge_suite)"
    bad "  and add its install/setup.bash to the ExecStart in fpms-rosbridge.service."
    exit 1
  fi
  [ -x /opt/ros/humble/lib/rosbridge_server/rosbridge_websocket ] \
    && ok "installed" || bad "still not present after apt"
fi

# --------------------------------------------------------------- 2. the files
head_ "2. files"
run install -d $OWNER -m 0755 "$DEST"
for f in fpms_console.py console.html README.md verify_console.py; do
  if [ -f "$SRC/$f" ]; then
    run install $OWNER -m 0644 "$SRC/$f" "$DEST/$f"; ok "$DEST/$f"
  else
    bad "missing from the source tree: $f"
  fi
done
if [ "$DRY" = 0 ]; then
  chmod 0755 "$DEST/fpms_console.py" "$DEST/verify_console.py" 2>/dev/null
fi

run install -d -m 0755 "$ETC"
if [ -f "$SRC/units/rosbridge_params.yaml" ]; then
  run install -m 0644 "$SRC/units/rosbridge_params.yaml" "$ETC/rosbridge_params.yaml"
  ok "$ETC/rosbridge_params.yaml  (the topic whitelist — read it)"
else
  bad "units/rosbridge_params.yaml missing. WITHOUT IT ROSBRIDGE HAS NO"
  bad "WHITELIST AND A BROWSER COULD PUBLISH /cmd_vel. Refusing to continue."
  exit 1
fi

if [ ! -f "$ETC/config.env" ]; then
  warn "$ETC/config.env does not exist. The console falls back to defaults"
  warn "(thing=rover2, broker=127.0.0.1:1883). If the plan never draws, that"
  warn "is the first thing to check."
fi

# ------------------------------------------------------------- 3. the units
head_ "3. systemd units"
for u in fpms-rosbridge.service fpms-console.service; do
  if [ -f "$SRC/units/$u" ]; then
    run install -m 0644 "$SRC/units/$u" "$UNITS/$u"; ok "$UNITS/$u"
  else
    bad "units/$u missing"
  fi
done
run systemctl daemon-reload

# Enabled, so they come back on a cold boot. This stack has been bitten before
# by a service that was running but not enabled and was silently absent after
# the next reboot.
head_ "4. enable + start"
for u in fpms-rosbridge.service fpms-console.service; do
  run systemctl enable "$u"
  run systemctl restart "$u"
done

if [ "$DRY" = 1 ]; then say ""; say "dry run complete — nothing was changed."; exit 0; fi

# ---------------------------------------------------------------- 5. verify
head_ "5. verify (this is the part that matters)"
sleep 4

for u in fpms-rosbridge.service fpms-console.service; do
  if systemctl is-active --quiet "$u"; then ok "$u active"; else
    bad "$u NOT active — journalctl -u $u -n 40"
  fi
  if systemctl is-enabled --quiet "$u"; then ok "$u enabled for cold boot"; else
    bad "$u NOT enabled — it will be missing after a reboot"
  fi
done

if ss -ltn 2>/dev/null | grep -q ':9090'; then ok "rosbridge listening on 9090"
else bad "nothing listening on 9090"; fi
if ss -ltn 2>/dev/null | grep -q ':8090'; then ok "console listening on 8090"
else bad "nothing listening on 8090"; fi

if curl -fsS -m 5 http://127.0.0.1:8090/ | grep -q 'FPMS CONSOLE'; then
  ok "the page is served"
else bad "GET / did not return the console page"; fi

# The whitelist, read back from the RUNNING server rather than from the file.
#
# `set +u` around the ROS setup script is not optional: setup.bash dereferences
# unset variables, and under `set -u` that does not print a warning — it EXITS
# THE SCRIPT. An earlier version of this installer silently stopped right here
# and reported success, having skipped the only safety check it performs.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null
set -u
GLOB="$(ROS_DOMAIN_ID=20 timeout 25 ros2 param get /rosbridge_websocket topics_glob 2>&1)"
say "  topics_glob in force: $GLOB"
case "$GLOB" in
  *cmd_vel*|*cmd_duty*|*cmd_enable*)
      bad "A DRIVE TOPIC IS IN THE WHITELIST. Fix rosbridge_params.yaml NOW." ;;
  *estop*)
      ok "whitelist is in force and contains no drive topic" ;;
  *)
      warn "could not read topics_glob back from the ROS graph (this call is"
      warn "flaky for the first ~90 s after a boot). It proves nothing either"
      warn "way. RUN THE REAL CHECK:  python3 $DEST/verify_console.py" ;;
esac

head_ "done"
if [ "$FAILED" = 0 ]; then
  IP="$(hostname -I | awk '{print $1}')"
  say "  Open on the laptop:  http://fpms-pi.local:8090/     (or http://$IP:8090/)"
  say "  STOP is the big red button, and the spacebar."
  say ""
  say "  NOW PROVE THE SAFETY CLAIMS — this is not optional the first time:"
  say "      python3 $DEST/verify_console.py"
  say "  It tries to advertise /cmd_vel from a browser-equivalent client and"
  say "  reports whether rosbridge refused."
else
  say "  THERE WERE FAILURES ABOVE. Do not run a mission from this console"
  say "  until they are fixed."
  exit 1
fi
