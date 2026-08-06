#!/usr/bin/env bash
# =============================================================================
# deploy_stack.sh — install the whole FPMS Pi-side stack, in one command.
#
#   ./deploy_stack.sh              install + enable + restart everything
#   ./deploy_stack.sh --dry-run    show exactly what would change, touch nothing
#   ./deploy_stack.sh --no-restart install and enable, but do not restart
#   ./deploy_stack.sh --status     report only
#
# Run it FROM THE PI (it is a local install script), or over ssh:
#   scp -r rover ubuntu@fpms-pi.local:~/deploy && \
#     ssh ubuntu@fpms-pi.local 'cd ~/deploy && sudo ./deploy_stack.sh'
#
# DESIGN RULES THIS SCRIPT FOLLOWS
#
#  1. IT NEVER TOUCHES micro-ros-agent UNLESS ASKED (--include-uros). That
#     serial link costs 90-225 s to re-establish and a restart of it takes
#     every topic on the robot down. A routine deploy must not be able to cost
#     four minutes of odometry.
#  2. IT BACKS UP BEFORE IT OVERWRITES. Every replaced file is copied to
#     ~/fpms_deploy_backup/<timestamp>/ first. fpms_missions.py in particular
#     has been edited live on this Pi more than once, and a deploy that
#     silently discarded that work has already happened.
#  3. IT REFUSES TO GO BACKWARDS on fpms_missions.py without --force-missions.
#     The repo copy has been BEHIND the Pi copy before. See the guard below.
#  4. IT WRITES NO SECRETS. /etc/fpms/config.env holds the broker password and
#     is NEVER created, overwritten or printed by this script. If it is
#     missing, the script says so and stops — every service reads config from
#     it and would otherwise come up on defaults pointing at the wrong broker.
# =============================================================================
set -uo pipefail

DRY=0; RESTART=1; STATUS_ONLY=0; INCLUDE_UROS=0; FORCE_MISSIONS=0
for a in "$@"; do
  case "$a" in
    --dry-run)        DRY=1 ;;
    --no-restart)     RESTART=0 ;;
    --status)         STATUS_ONLY=1 ;;
    --include-uros)   INCLUDE_UROS=1 ;;
    --force-missions) FORCE_MISSIONS=1 ;;
    -h|--help)        sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown option: $a"; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "$0")" && pwd)"
HOME_DIR="${SUDO_USER:+/home/$SUDO_USER}"; HOME_DIR="${HOME_DIR:-$HOME}"
OWNER="${SUDO_USER:-$(id -un)}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$HOME_DIR/fpms_deploy_backup/$STAMP"
UNIT_DIR=/etc/systemd/system

# The full boot set, in dependency order. Everything here gets ENABLED, so a
# cold power-on brings the rover up with no human action.
UNITS=(
  fpms-cored.service          # STOP authority — first, before anything can move
  fpms-rover-agent.service    # camera + LiDAR -> MQTT
  fpms-lidar-ros.service      # MQTT LiDAR -> /scan_lidar
  fpms-tf.service             # static TF incl. the map->odom arena anchor
  fpms-odom-tf.service        # /odom_raw + /imu -> /odom, odom->base_footprint
  fpms-teleop.service         # MQTT -> /cmd_vel (joystick + characterisation)
  fpms-missions.service       # planner + B8B measured-segment executor
  fpms-uros-supervisor.service
  fpms-wifi-powersave-hold.service
)

# Units that can write /cmd_vel and are NOT part of the boot set. Masked, not
# merely disabled: `disable` only removes the WantedBy symlink, so anything
# that pulls them in by name still starts them. A second writer on /cmd_vel is
# not a race the executor can win or even reliably detect in time.
MASK=( fpms-ros-tunnel.service fpms-rtos-follower.service )

# Retired. fpms-map-odom's single static transform is now published by
# fpms-tf.service, and leaving the old unit enabled would give that TF edge two
# publishers — the one thing TF_TREE.md says must never happen.
RETIRE=( fpms-map-odom.service )

say()  { printf '  %s\n' "$*"; }
head_() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
run()  { if [ "$DRY" = 1 ]; then say "[dry-run] $*"; else eval "$@"; fi; }

need_root() {
  if [ "$(id -u)" != 0 ]; then
    echo "This script installs systemd units; re-run with sudo." >&2; exit 1
  fi
}

# --------------------------------------------------------------- status ----
report() {
  head_ "unit status"
  for u in "${UNITS[@]}"; do
    printf '  %-34s %-10s %s\n' "$u" \
      "$(systemctl is-enabled "$u" 2>/dev/null || echo -)" \
      "$(systemctl is-active  "$u" 2>/dev/null || echo -)"
  done
  printf '  %-34s %-10s %s\n' "micro-ros-agent.service" \
    "$(systemctl is-enabled micro-ros-agent.service 2>/dev/null || echo -)" \
    "$(systemctl is-active  micro-ros-agent.service 2>/dev/null || echo -)"
  head_ "masked (must stay masked — second /cmd_vel writers)"
  for u in "${MASK[@]}"; do
    printf '  %-34s %s\n' "$u" "$(systemctl is-enabled "$u" 2>/dev/null || echo -)"
  done
}

if [ "$STATUS_ONLY" = 1 ]; then report; exit 0; fi
need_root

head_ "preflight"
if [ ! -f /etc/fpms/config.env ]; then
  cat >&2 <<'MSG'
  /etc/fpms/config.env does not exist.

  Every FPMS service reads its broker host, credentials and calibration
  overrides from that file, and NOT from the environment. Without it they all
  come up on built-in defaults, which point at a broker that is probably not
  yours, and the failure looks like "the dashboard shows nothing".

  It holds the mosquitto password, so it is deliberately NOT in the repo and
  this script will not create it. Copy the template and fill it in:

      sudo install -d -m 0755 /etc/fpms
      sudo cp rover/config.env.example /etc/fpms/config.env
      sudo chown root:root /etc/fpms/config.env
      sudo chmod 0640 /etc/fpms/config.env
      sudo nano /etc/fpms/config.env        # set FPMS_MQTT_PASS

MSG
  exit 1
fi
say "config.env present"

# --- the version guard -----------------------------------------------------
# fpms_missions.py is the one file on this rover that has repeatedly been newer
# on the Pi than in the repo, because it gets edited live during a session.
# Overwriting it with an older copy silently deletes gate-verified planner work.
# Compare line counts (a crude but honest proxy) and refuse to go backwards.
if [ -f "$HOME_DIR/fpms_missions.py" ] && [ -f "$HERE/fpms_missions.py" ]; then
  OLD=$(wc -l < "$HOME_DIR/fpms_missions.py"); NEW=$(wc -l < "$HERE/fpms_missions.py")
  say "fpms_missions.py: on-Pi ${OLD} lines, deploying ${NEW} lines"
  if [ "$NEW" -lt "$OLD" ] && [ "$FORCE_MISSIONS" != 1 ]; then
    cat >&2 <<MSG

  REFUSING to overwrite fpms_missions.py with a SHORTER file.
    on Pi:     ${OLD} lines
    deploying: ${NEW} lines

  The copy on the Pi is probably newer than the repo — this has happened
  before and it cost gate-verified planner work. Diff them first:

      diff <(ssh ubuntu@fpms-pi.local cat fpms_missions.py) $HERE/fpms_missions.py

  If the repo really is correct, re-run with --force-missions.
MSG
    exit 1
  fi
fi

# ------------------------------------------------------------- install ----
head_ "backup -> $BACKUP"
run "install -d -o '$OWNER' '$BACKUP'"

install_file() {  # src dest mode
  local src="$1" dest="$2" mode="${3:-0755}"
  [ -f "$src" ] || { say "SKIP (missing): $src"; return; }
  if [ -f "$dest" ]; then
    if cmp -s "$src" "$dest"; then say "unchanged: $dest"; return; fi
    run "cp -a '$dest' '$BACKUP/'"
  fi
  run "install -m '$mode' '$src' '$dest'"
  say "installed: $dest"
}

head_ "payload -> $HOME_DIR"
install_file "$HERE/fpms_missions.py"   "$HOME_DIR/fpms_missions.py"
install_file "$HERE/fpms_lidar_ros.py"  "$HOME_DIR/fpms_lidar_ros.py"
install_file "$HERE/fpms_teleop.py"     "$HOME_DIR/fpms_teleop.py"
install_file "$HERE/fpms_odom_tf.py"    "$HOME_DIR/fpms_odom_tf.py"
install_file "$HERE/stack/fpms_cored.py"    "$HOME_DIR/fpms_cored.py"
install_file "$HERE/stack/fpms_charact.py"  "$HOME_DIR/fpms_charact.py"
install_file "$HERE/STACK.md"           "$HOME_DIR/STACK.md" 0644
run "chown -R '$OWNER' '$HOME_DIR'/fpms_*.py '$HOME_DIR/STACK.md' 2>/dev/null || true"

head_ "payload -> /usr/local/bin"
install_file "$HERE/fpms-rover-agent.py"      /usr/local/bin/fpms-rover-agent
install_file "$HERE/stack/fpms-uros-supervisor" /usr/local/bin/fpms-uros-supervisor
install_file "$HERE/stack/fpms-uros-agent-run"  /usr/local/bin/fpms-uros-agent-run

head_ "systemd units"
for u in "${UNITS[@]}"; do install_file "$HERE/units/$u" "$UNIT_DIR/$u" 0644; done
if [ "$INCLUDE_UROS" = 1 ]; then
  install_file "$HERE/units/micro-ros-agent.service" "$UNIT_DIR/micro-ros-agent.service" 0644
else
  say "micro-ros-agent.service NOT touched (pass --include-uros to update it)"
fi

# --- escalation privilege --------------------------------------------------
# fpms-cored runs as `ubuntu` and must be able to SIGTERM fpms-missions when
# the executor is wedged under a moving rover. Exactly one command, no
# wildcards, no shell.
head_ "stop-escalation privilege"
SUDOERS=/etc/sudoers.d/fpms-cored
if [ "$DRY" = 1 ]; then
  say "[dry-run] would write $SUDOERS"
else
  printf '%s ALL=(root) NOPASSWD: /bin/systemctl kill -s SIGTERM fpms-missions.service\n' \
    "$OWNER" > "$SUDOERS.tmp"
  chmod 0440 "$SUDOERS.tmp"
  if visudo -cf "$SUDOERS.tmp" >/dev/null 2>&1; then
    mv "$SUDOERS.tmp" "$SUDOERS"; say "installed $SUDOERS"
  else
    rm -f "$SUDOERS.tmp"
    say "WARNING: sudoers rule failed validation; escalation will log a failure."
    say "         The other three stop paths are unaffected."
  fi
fi

head_ "enable + mask"
run "systemctl daemon-reload"
for u in "${RETIRE[@]}"; do
  if systemctl list-unit-files "$u" >/dev/null 2>&1; then
    run "systemctl disable --now '$u' 2>/dev/null || true"
    run "rm -f '$UNIT_DIR/$u'"
    say "retired: $u (merged into fpms-tf.service)"
  fi
done
for u in "${MASK[@]}"; do
  run "systemctl disable --now '$u' 2>/dev/null || true"
  run "systemctl mask '$u' 2>/dev/null || true"
  say "masked: $u"
done
for u in "${UNITS[@]}"; do
  run "systemctl enable '$u' >/dev/null 2>&1 || true"
  say "enabled: $u"
done
# micro-ros-agent must be enabled for cold boot even though we do not restart it.
run "systemctl enable micro-ros-agent.service >/dev/null 2>&1 || true"
run "systemctl daemon-reload"

if [ "$RESTART" = 1 ]; then
  head_ "restart (micro-ros-agent deliberately excluded)"
  # Order matters: the stop authority first, sensors next, the executor last.
  for u in "${UNITS[@]}"; do
    run "systemctl restart '$u' || true"
    say "restarted: $u"
    sleep 1
  done
else
  head_ "restart skipped (--no-restart)"
fi

head_ "result"
report

cat <<'NEXT'

  NEXT, AND NONE OF IT IS OPTIONAL BEFORE YOU DRIVE:

  1. Confirm the LiDAR is live AND honest:
       mosquitto_sub -h localhost -t 'fpms/rover2/telemetry/lidar' -C 1 | \
         python3 -c 'import sys,json; d=json.load(sys.stdin); \
           print({k:d[k] for k in ("health","stale","hz","scan_age_s","seq")})'
     health must be "ok" and hz must be non-zero. If health is "dead" the feed
     is still publishing on cadence — that is correct behaviour, not a bug.
     Cover the scanner and watch health go to "dead" and hz to 0.0.

  2. Confirm STOP is answered:
       mosquitto_sub -h localhost -t 'fpms/rover2/events/command_receipt' &
       mosquitto_pub -h localhost -t 'fpms/rover2/commands/stop' -m '{}'
     A receipt with state "honoured" must appear immediately.

  3. Settle the odometry sign WITHOUT DRIVING — push the rover by hand:
       python3 ~/fpms_charact.py --push-check
     Pushed forward, x must INCREASE. If it decreases, set
     FPMS_MISSION_ODOM_POSE_SIGN in /etc/fpms/config.env and re-zero.

  4. Only then, a PREVIEW (plans, publishes, does not move):
       mosquitto_pub -h localhost -t 'fpms/rover2/commands/mission' \
         -m '{"name":"m2","preview":true}'

NEXT
