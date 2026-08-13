# FPMS-OS runbook

Short on purpose. You are meant to be able to use this under pressure.

---

## The 30-second pre-run check

```sh
ssh ubuntu@fpms-rover1.local
fpms-selftest
```

Then look at the console at `http://fpms-rover1.local:8090/` and confirm:

- the LiDAR ring is **moving**, and the age counter is under a second
- the battery reads something plausible
- STOP is reachable

**Judge motion by your own eyes, never by odometry.** This rover has reported
clean travel while spinning in place and has lied about its direction of
travel. Nothing in this image changes that — the residual stream makes it
*visible*, not trustworthy.

---

## Before the first mission on a new board

In this order. Step 1 is not optional and nothing downstream means anything
without it.

### 1. The sign — 30 seconds, no battery, no driving

```sh
python3 ~/fpms_charact.py --push-check
```

Push the rover forward along a tape measure. **`x` must INCREASE.**

If it decreases, flip `FPMS_MISSION_ODOM_POSE_SIGN` in `/etc/fpms/config.env`
and restart `fpms-missions`. Getting this wrong is silent and total: every
displacement comes from differencing pose, so an inverted sign makes a forward
burst measure as reverse travel, the residual correction push the wrong way,
and the retrace replay the error again — all with plausible numbers.

> Changing the sign **invalidates `~/.fpms_teleop_origin.json`**. Re-zero with
> `set_coordinate` (the RESET POSE button) afterwards.

### 2. Heading

```sh
python3 ~/fpms_charact.py --spin-check 90
```

### 3. Coast, minimum duty, left/right asymmetry

```sh
python3 ~/fpms_charact.py --drive
python3 ~/fpms_charact.py --report     # what is set, and where it came from
```

`fpms_charact.py` only *watches* — it publishes nothing to `/cmd_vel` or
`/cmd_duty`. You drive, using the dashboard's own controls. What it measures is
what the rover did, not what you pressed.

### 4. Only now, a preview mission

Plan first (`preview: true`), read the route, then run it.

---

## Reading `telemetry/residual`

Published the instant each segment settles, with the chassis stationary, so
both numbers are trustworthy. It is the most diagnostic thing this rover
produces, and it separates three failures that look identical on the map:

| Pattern | Cause | Fix |
|---|---|---|
| constant **ratio** — every drive short or long by the same factor | scale, i.e. counts/mm | set `odom_scale` |
| constant **offset** regardless of leg length | coast after the burst is cut | set `coast_mm` |
| grows **only on turns** | gyro | set `gyro_scale` |

Per segment, not per leg. A leg is several segments and averaging hides exactly
the pattern you are looking for.

---

## When something breaks

Start here, always:

```sh
fpms-selftest          # what is wrong
fpms-doctor <symptom>  # what to do about it
```

### The dashboard is empty but every unit is green

Almost certainly the broker password. Nothing logs an error — every service
just retries forever.

```sh
grep FPMS_MQTT_PASS /etc/fpms/config.env    # still the placeholder?
sudo fpms-firstboot --force                 # regenerate
```

### A topic exists, has a publisher, and delivers nothing

The DDS bug. `ros2 topic info` will happily report `Publisher count: 1`.

```sh
ros2 topic hz /scan_lidar     # hangs => confirmed
systemctl show fpms-lidar-ros -p Environment | grep FASTRTPS
```

If the profile is not in the unit's environment, that is the fault. If it is,
see `docs/DDS.md` and enable the fallback:

```sh
sudo systemctl enable --now fpms-ros-settle
```

### The rover will not move

In order of likelihood:

1. **Not armed.** Missions require an explicit arm; it expires after 120 s.
2. **STOP is latched.** Check `events/stop_asserted`.
3. **micro-ROS link is down.** `ros2 topic hz /odom_raw`. If dead, **wait** —
   it takes 90–225 s to re-establish and a restart makes it worse.
4. **LiDAR is stale** and the executor is refusing to move without it. Correct
   behaviour.

### The LiDAR is stale

```sh
mosquitto_sub -h 127.0.0.1 -u fpms -P "$(sudo cat /var/lib/fpms/broker-password)" \
  -t 'fpms/rover1/telemetry/lidar' -C 1 | jq '{health,hz,stale,seq}'
```

- `seq` climbing while `stale: true` → **the scanner** is at fault. Check the
  tty, the baud, and whether something else opened port 1.2.
- `seq` frozen → **the process** is at fault. `systemctl restart
  fpms-rover-agent`.

Those need different fixes and were indistinguishable before `seq` existed.

### The micro-ROS link is dead

**Wait first.** 90–225 seconds, unaided, after any agent restart. No
reset-button press is needed or helpful.

If it is genuinely wedged after ~5 minutes:

```sh
sudo systemctl restart micro-ros-agent
# then, after it comes back, the three subscribers must re-match:
sudo systemctl restart fpms-teleop fpms-odom-tf fpms-missions
```

`fpms-uros-supervisor` does this automatically. If it fires often, something
upstream is wrong.

### Everything is confusing

```sh
journalctl -b -u 'fpms-*' --no-pager | tail -100
cat /var/lib/fpms/selftest.json | jq '.checks[] | select(.status!="PASS")'
cat /etc/fpms/versions.json          # exactly what this image is
```

---

## Things not to do

- **Do not restart `micro-ros-agent` casually.** 90–225 s to re-link.
- **Do not turn-test for liveness.** It destroys heading. Check pose passively.
- **Do not unmask `fpms-ros-tunnel` or `fpms-rtos-follower`.** Two writers on
  `/cmd_vel` is not a race the executor can win.
- **Do not widen the rosbridge whitelist.** The drive topics are absent by
  construction so a stray click cannot turn a wheel.
- **Do not run `teleop_twist_joy`** without stopping `fpms-teleop` first — it
  publishes `/cmd_vel` directly.
- **Do not set `message_size_limit`** on mosquitto below 262144. Camera frames
  vanish silently.
- **Do not trust `/odom_raw`'s `twist`.** Its sign is inverted relative to its
  own `pose`. Trust pose.

---

## Capturing state for a post-mortem

```sh
mkdir -p /tmp/fpms-dump && cd /tmp/fpms-dump
fpms-selftest --json                     > selftest.json
journalctl -b --no-pager                 > journal.txt
systemctl list-units 'fpms-*' --all      > units.txt
cp /etc/fpms/versions.json /etc/fpms/config.env .
cp /var/lib/fpms/*.json . 2>/dev/null
sed -i 's/^FPMS_MQTT_PASS=.*/FPMS_MQTT_PASS=<redacted>/' config.env
ros2 run tf2_tools view_frames           # frames.pdf, if TF is suspect
cd .. && tar czf fpms-dump-$(date +%s).tar.gz fpms-dump
```

Redact `config.env` before sharing it — it holds the broker password.
