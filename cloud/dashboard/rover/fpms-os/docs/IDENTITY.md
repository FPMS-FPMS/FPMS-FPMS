# Rover identity

**This image is Rover 1.** Thing name `rover1`, hostname `fpms-rover1`.

It was `rover2` / `fpms-pi` until 2026-08-13. If you are reading a document,
a shortcut or a printout that says either of those, it predates this change and
nothing forwards the old name to the new one.

This file exists because "which rover is this" is not one setting. It is five
independent things, they are set in four different places at three different
times, and **every one of them fails silently when it disagrees with a peer.**
Nothing in this system errors on a wrong identity. It just goes quiet.

---

## 1. The five identities

| # | Identity | Value on this image | Where it is set | When |
|---|---|---|---|---|
| 1 | MQTT thing name (topic root) | `rover1` | `overlay/etc/fpms/config.env` → `FPMS_THING_NAME` | baked |
| 2 | Hostname / mDNS name | `fpms-rover1` | `config/fpms-os.conf` → `FPMS_HOSTNAME`, applied by `scripts/00-base-system.sh`; **re-applied by `fpms-firstboot`** | baked, then first boot |
| 3 | Broker credentials | user `fpms`, password generated per board | `fpms-firstboot` → `config.env`, `fpms.passwd`, bridge `remote_password` | first boot |
| 4 | `machine-id` | none in the image | cleared by `scripts/90-finalise.sh`; systemd generates one | first boot |
| 5 | SSH host keys | none in the image | `fpms-firstboot` runs `ssh-keygen -A` | first boot |

And one thing that looks like an identity and is **not**:

| ROS domain | `20` | `FPMS_ROS_DOMAIN_ID` in `config.env`, `ROS_DOMAIN_ID` in `fpms-os.conf` | **shared by every rover** |

Two rovers on domain 20 on the same LAN are one DDS graph. They will discover
each other's `/scan`, `/odom` and `/cmd_vel`, and the topic names do not carry
the thing name, so there is nothing to disambiguate them. Rover 1 can be driven
by Rover 2's teleop node. **This is unaddressed today** and is the reason a
second rover is not simply "flash the same card twice". See §5.

### 1a. What is *not* per-rover, though it looks like it

- **The mosquitto ACL does not isolate rovers.** `overlay/etc/mosquitto/fpms.acl`
  has `pattern write fpms/%u/telemetry/#`, and `%u` is the *username*, not the
  thing name. Every rover service authenticates as `fpms`, so those patterns
  expand to `fpms/fpms/...` and match nothing; what actually grants access is
  the `user fpms` block's `topic readwrite fpms/#`. The practical consequence:
  **a rover is not prevented from publishing on another rover's topic root, and
  a command addressed to the wrong rover is accepted by the broker, not
  rejected.** The topic root is a routing convention here, not a boundary.
  (The file's own comment says the ids are `<FPMS_THING_NAME>-<role>` and that
  `%c` is the client id — but the rules use `%u`. Do not read that comment as a
  description of what the rules do.)
- **The bridge is `topic # both 0`** — the whole tree, both directions. It does
  not filter by thing name. Two rovers bridged to one laptop put both topic
  roots on that laptop's broker, which is what makes the dashboard's thing
  selector meaningful at all.

---

## 2. What changing the hostname costs

Changing `fpms-pi` → `fpms-rover1` is correct for a two-rover fleet — see §5 —
but it is not free. This is the complete list of what it moves.

**Breaks (functional):**

| Where | What happens |
|---|---|
| `backend/ros_bridge.py` | defaults `FPMS_ROS_HOST=fpms-pi.local` (line 121). Left alone it dials a name that no longer exists: **connection failure**, not a cosmetic wrong label. Set `FPMS_ROS_HOST=fpms-rover1.local` or point it at the IP. The same file also defaults `FPMS_ROS_THING="rover2"` (line 126) — see §5. |
| `scripts/Resolve-Broker.ps1` | `[string[]]$Names = @('fpms-pi.local')` is the default it resolves the broker by, and `Start-FPMS-Dashboard.template.cmd` falls back to `FPMS_MQTT_HOST=fpms-pi.local` when that script is missing. Two independent paths to the old name. |
| `fpms_console/Open-FPMS-Console.cmd`, `foxglove/Open-FPMS-Foxglove.ps1` | `set HOST=fpms-pi.local` / `$PRIMARY_HOST = "fpms-pi.local"`. These are the shortcuts the operator actually double-clicks. |
| `rover/deploy_rover.py` | `fpms-pi.local` is the first SSH candidate in its host list. It falls through to hardcoded IPs, so this degrades to slow rather than broken. |
| `fpms_console/fpms_console.py` | builds its printed URL from `socket.gethostname() + ".local"`, so it now *advertises* `fpms-rover1.local:8090`. That is correct and self-updating — but it means the console's own output no longer matches any document that says `fpms-pi.local`. |
| `overlay/usr/local/sbin/fpms-firstboot` | hardcodes `WANT_HOST=fpms-pi` and runs **after** the image's hostname is set. **Unchanged, it sets the hostname straight back to `fpms-pi` on first boot and `FPMS_HOSTNAME` in `fpms-os.conf` is never seen.** This is the one that makes the change a no-op. |
| `overlay/usr/local/sbin/fpms-wifi-provision` | the fallback-AP banner tells the operator to open `http://fpms-pi.local:8090/` — printed at exactly the moment they have no other way in. |
| `overlay/usr/local/bin/fpms-doctor` | runs `ping fpms-pi.local` as a diagnostic. It will now report a failure that is not a failure, on the tool people reach for when something is wrong. |
| `scripts/90-finalise.sh` | prints `http://${FPMS_HOSTNAME}.local:8090/` — self-updating, correct, listed here so the build banner change is expected rather than alarming. |
| Desktop shortcuts / `.cmd` / `.ps1` launchers | anything with the old name baked in stops working. Nothing warns; the browser just fails to resolve. |

**Breaks (documentation):** every `ssh ubuntu@fpms-pi.local`, `scp` and
`http://fpms-pi.local:8090/` in `README.md`, `SPEC.md`, `docs/FLASHING.md`,
`docs/RUNBOOK.md`, `docs/FAILURE_MODES.md`, `firstboot/README.md`,
`npu/README.md`, `npu/convert/README.md`, `npu/convert/convert_yolo26.sh`,
`scripts/30-fpms-payload.sh`. These are copy-paste instructions; a wrong one
costs an operator a support round-trip, not a fault.

**Does not break:** `/etc/hosts` and avahi (both derived from the hostname),
the MQTT stack (localhost + a bridge to an IP — no name involved), the ROS/DDS
layer (UDPv4 discovery by address). **The hostname is not on the telemetry
path.** A rover with a "wrong" hostname still drives and still reports; only
the humans and the browser-facing tools lose it.

---

## 3. Baking the WiFi in

The competition network is baked into the image:

```
FPMS_WIFI_SSID=FPMS_Net
FPMS_WIFI_PASS=fpms2026
```

in `overlay/etc/fpms/config.env` (0640 root:root, on the ext4 rootfs).

**SUPPLIED 2026-08-13 by the operator. UNVERIFIED** — no board has associated
with this SSID. A typo presents as "the venue WiFi is down": the rover comes up
on its own `FPMS-Rover-Setup` AP and offers no more specific complaint.

**The boot-partition mechanism is unchanged and still wins.** `fpms-wifi.conf`
on the FAT partition is registered at a *higher* NetworkManager autoconnect
priority; the baked pair is appended last, at the lowest. The baked value is a
default for a card that nobody prepared, not a replacement for the file.

It is in `config.env` and not in a shipped `fpms-wifi.conf` on the boot
partition because that partition is FAT and mounts automatically on every
Windows machine that sees the card — the same reason `fpms-wifi-provision`
moves the operator's file to `.applied` and chmods it 600 after reading it.

### Required change to `fpms-wifi-provision` — NOT YET APPLIED

As shipped, `fpms-wifi-provision` reads **only** `$BOOTDIR/fpms-wifi.conf`. It
takes no client SSID from the environment — its `FPMS_AP_SSID`/`FPMS_AP_PASS`
are the *fallback access point*, a different thing — and
`fpms-firstboot.service` has no `EnvironmentFile=`, so nothing puts
`config.env` in front of it. **Until the two hunks below land, the two keys
above are inert and a bare card still comes up on the fallback AP.**

Hunk 1 — after `SCAN_WAIT_S="${FPMS_WIFI_SCAN_WAIT_S:-45}"`:

```bash
# Baked-in default network, from /etc/fpms/config.env.
#
# Read ONE key at a time rather than `. /etc/fpms/config.env`. Sourcing it puts
# FPMS_MQTT_PASS into this process's environment and from there into every
# nmcli child's /proc/<pid>/environ, where any local user can read it.
cfg_get() {
    [ -r /etc/fpms/config.env ] || return 0
    sed -n "s/^${1}=//p" /etc/fpms/config.env | tail -n1 | tr -d '"\r'
}
DEF_SSID="$(cfg_get FPMS_WIFI_SSID)"
DEF_PASS="$(cfg_get FPMS_WIFI_PASS)"
```

Hunk 2 — immediately after the `fi` that closes the `if [ -r "$CONF" ]` block
(the one ending `log "no $CONF - see README-FPMS.txt on the boot partition"`),
and **before** `prio=$(( ${#SSIDS[@]} + 10 ))`:

```bash
# Appended LAST, so it takes the LOWEST autoconnect priority and anything the
# operator wrote on the boot partition still wins. Skipped if they already
# listed the same SSID - re-adding it would demote their password below ours.
if [ -n "$DEF_SSID" ]; then
    _dup=0
    for _s in ${SSIDS[@]+"${SSIDS[@]}"}; do
        [ "$_s" = "$DEF_SSID" ] && { _dup=1; break; }
    done
    if [ "$_dup" -eq 0 ]; then
        SSIDS+=("$DEF_SSID"); PASSES+=("$DEF_PASS")
        log "added baked-in default network '$DEF_SSID' (lowest priority)"
    else
        log "baked-in default '$DEF_SSID' already listed on the boot partition"
    fi
fi
```

Notes on the hunks:

- `${SSIDS[@]+"${SSIDS[@]}"}` guards the empty array under this script's
  `set -u`. Ubuntu 22.04's bash 5.1 already tolerates `"${arr[@]}"` when empty;
  the guard costs nothing and survives a shell downgrade.
- The `.applied` rename is untouched: it is still gated on `[ -r "$CONF" ]`, so
  a card with no operator file has nothing renamed and the baked default is
  re-applied on every re-run.
- The "wait up to 45 s for a known network" branch now runs on a bare card too,
  which is the point — there is finally a network worth waiting for.
- Behaviour is unchanged when the keys are absent, so the patch is safe to
  carry on an image that has not been re-baked.

---

## 4. Building Rover 2 (or Rover 3)

Change these, together, in one commit:

1. `overlay/etc/fpms/config.env` → `FPMS_THING_NAME=rover2`
2. `config/fpms-os.conf` → `FPMS_HOSTNAME="fpms-rover2"`
3. `overlay/usr/local/sbin/fpms-firstboot` → `WANT_HOST` (both occurrences)
4. `overlay/boot/README-FPMS.txt` and `fpms-wifi.conf.example` → the name the
   operator is told to type
5. **The dashboard's rover bays.** `frontend/src/lib/things.ts` is *not* the
   place — it already discovers things from `things_seen` in `/api/health` and
   hardcodes nothing. The hardcoded pairs live in the consumers, which each
   union a literal `["rover1", "rover2"]` into whatever was discovered:
   `pages/Lidar.tsx`, `pages/Camera.tsx`, `pages/Drive.tsx`,
   `pages/Control.tsx`, `pages/Analyst.tsx` — plus `pages/Rover2Test.tsx`,
   which pins `const THING = "rover2"` outright. Today that union means a
   `rover1` bay appears whether or not a rover1 exists; a third rover would be
   invisible until these five lines learn its name.
6. `backend/ros_bridge.py` → `FPMS_ROS_HOST` and `FPMS_ROS_THING` defaults, or
   set both per rover in the environment
7. Decide the DDS question in §5 before the two boards are ever powered on
   together

Do **not** change: `FPMS_MQTT_USER`, the ACL, the bridge topic pattern, or
anything under "SERIAL DEVICES" / "MISSION EXECUTOR" in `config.env`. Those are
hardware and fleet-wide, not per-rover — and the mission constants in
particular are per-*chassis* calibration that has been measured on one robot.
Copying an image does not copy a wheel.

### What is *not* shared between two boards flashed from one image

Nothing per-board is baked in, by design. Both boards get:

- their **own** `machine-id` (cleared at build by `90-finalise.sh`, generated by
  systemd on first boot). `verify_image.sh` FAILs the build if it is non-empty:
  a shared machine-id collides journald and DHCP client identifiers, and the
  symptom on the second rover is "the dashboard shows one rover twice"
- their **own** SSH host keys (`ssh-keygen -A` in `fpms-firstboot`; the image
  ships none)
- their **own** broker password (24 random chars, generated in
  `fpms-firstboot`, written to `config.env`, `fpms.passwd` and the bridge's
  `remote_password`, and printed once to the console)

So two rovers do **not** share credentials — but that also means the operator
laptop's broker needs **both** passwords entered, and each laptop's
`known_hosts` gets two entries. Use `fpms-mqtt-password` on the boot partition
if you deliberately want them to share one broker password.

---

## 5. The traps

**A dashboard subscribed to the wrong root sees nothing, and reports nothing.**
This is the headline. `FPMS_THING_NAME` is the MQTT topic root — every
`fpms/<thing>/telemetry/...`, `commands/...` and `events/...` moves with it, in
one step. A consumer still on `fpms/rover2/#` connects, authenticates, stays
connected, and receives nothing forever. Every unit is `active`, the broker is
up, the bridge is up, and the rover is silent. **Check the topic root before
believing a silence.** `mosquitto_sub -t 'fpms/#' -v` on the laptop settles it
in one command: it shows you which root is actually in use.

The command direction is worse than the telemetry direction. A **STOP published
to `fpms/rover2/commands/stop` is accepted by the broker** (see §1a — the ACL
grants `fpms/#`) **and read by nobody.** No error reaches the operator. Treat a
thing-name mismatch as a safety defect, not a configuration one.

**Every service's code fallback is still `rover2`, and always will be.** The
thing name is set in exactly one place — `config.env` — but roughly twenty
call sites read it as `CFG.get("FPMS_THING_NAME", "rover2")`: `fpms_missions`,
`fpms_lidar_ros`, `fpms_rover_agent`, `fpms_teleop`, `fpms_cored`,
`fpms_charact`, `fpms_cloud_uplink`, `fpms_rtos_follower`, `fpms_ros_tunnel`,
`fpms-uros-supervisor`, `fpms-selftest`, `fpms-npud`, `fpms-npu-selftest`,
`fpms-doctor`, `fpms_console.py`, `fpms_foxglove_cmd.py`, and
`ros_bridge.py`'s `FPMS_ROS_THING`. `rover/profile_steps.py` hardcodes
`THING = "rover2"` with no override at all.

**So a Rover 1 board that loses `/etc/fpms/config.env` does not fail — it
becomes Rover 2.** Every `load_config()` swallows `FileNotFoundError`, so the
whole stack quietly republishes itself under the other rover's topic root, on
a shared broker, with an ACL that permits it (§1a). If both rovers are live,
one dashboard bay then shows two robots interleaved and neither is labelled
wrong. This is why `deploy_stack.sh` hard-refuses to run without that file,
and why `fpms-selftest` checks it. Changing the fallbacks to `rover1` would
not fix it, only move which rover gets impersonated; the fix is that the file
must exist, which is what FPMS-OS ships it for.

**`.local` resolution is per-resolver, not per-machine.** In this project, mDNS
has resolved from .NET and failed from Python's `getaddrinfo` on the same box,
at the same time, for the same name. So `ping fpms-rover1.local` succeeding
proves nothing about whether your *tool* will resolve it, and a tool failing
proves nothing about the rover. Confirm per-tool, or use the IP from the
hotspot's client list. This is also why `fpms-doctor`'s ping test is weak
evidence in both directions.

**mDNS name collision does not error either — it renames.** If two boards both
answer to `fpms-rover1`, avahi does not refuse; it renames one of them to
something like `fpms-rover1-2`, at random, per boot. From then on the name you
type reaches whichever rover won the race that morning. This is the strongest
argument for the hostname change and against "just flash the same card twice".

**Same ROS domain, one DDS graph.** `ROS_DOMAIN_ID=20` is baked identically
into every image. ROS topic names (`/scan`, `/odom`, `/cmd_vel`) carry no thing
name, so two rovers on one LAN cross-subscribe. Nothing in the MQTT identity
touches this. Before running two boards together, give the second one a
different `ROS_DOMAIN_ID` or a namespace — and note that changing the domain
also moves rosbridge and Foxglove for that rover.

**The image's hostname is not the last word.** `fpms-firstboot` runs after it
and hardcodes its own. Verify with `hostnamectl` on a booted board, not by
reading `fpms-os.conf`.

**Renaming a rover invalidates nothing about its calibration — and that is a
trap too.** `FPMS_MISSION_*`, the arena anchor and
`~/.fpms_teleop_origin.json` are per-chassis and survive a rename untouched.
Copying this image to a second physical robot copies a first robot's measured
constants along with it. See `config.env` and `docs/CALIBRATION.md`.
