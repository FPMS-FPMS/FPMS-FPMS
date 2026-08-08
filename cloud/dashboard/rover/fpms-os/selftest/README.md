# Verification

Two layers: one that needs no hardware, one that runs on the rover.

## Offline — on the build machine

```sh
python3 selftest/test_image_offline.py
```

Twelve structural checks over the overlay and the units. Run it before every
build; it takes under a second.

The check that justifies the file is **`exec_paths_exist`**: every `ExecStart`
and `ExecStartPre` must point at something the image actually contains. That is
exactly the defect that shipped for months — `fpms-wait-net` was named by three
units as an `ExecStartPre` *without* a `-` prefix, and no repository contained
it, so two units could not start at all and the only symptom was a missing
LiDAR and a missing TF tree.

The others: units parse and declare `Restart=`; every ROS unit carries
`ROS_DOMAIN_ID=20`, `RMW_IMPLEMENTATION` and `FASTRTPS_DEFAULT_PROFILES_FILE`;
`EnvironmentFile=` precedes `Environment=` so `config.env` cannot override the
domain; the DDS profile parses *and* has `useBuiltinTransports` false; the
sudoers rule names `fpms-missions.service` exactly and passes `visudo -cf`;
the rosbridge whitelist contains no drive topic and `params_glob` is empty;
nothing is both enabled and masked, and every masked unit is actually shipped;
no secret-shaped string is baked in; `FPMS_LIDAR_PORT` is not a `ttyUSBn`; and
no real `calibration.json` is present.

Writing this file caught two bugs in itself — it was matching directive names
inside comment prose, and it did not know stage 30 installs the supervisor from
the repo rather than the overlay — and one real gap: four variables the units
expand that `config.env` never defined.

## On the rover

```sh
fpms-selftest            # human readable
fpms-selftest --json     # machine readable
fpms-doctor <symptom>    # what to do about a failure
```

Runs automatically 90 seconds after every boot and publishes to
`telemetry/health`. The 90 s is not arbitrary: the micro-ROS link takes up to
225 s to establish from cold, so a check at 30 s would report a healthy rover
as broken every single boot, and an alarm that cries wolf is worse than none.

It exists because **a list of green units has repeatedly meant nothing.** Every
check maps to something that happened:

| Check | What it catches |
|---|---|
| `DDS carries data across eras` | the one that matters. Subscribes from a *freshly started* process — the case that fails while every topic list passes |
| `paho-mqtt >= 2.0` | apt's 1.6.1 has no `CallbackAPIVersion`; the STOP authority imports it unguarded |
| `numpy is 1.x` / `no shadowing user-local numpy` | a user-local 2.x shadowed the system one and broke things far from the cause |
| `boot units enabled` | `fpms-missions` was once found active-but-disabled — fine until the next reboot |
| `second /cmd_vel writers masked` | `disabled` is not `masked` |
| `broker password provisioned` | an auth failure crashes nothing; services retry forever and the dashboard is empty |
| `STOP escalation sudoers rule` | sudo matches literal argv; a mismatch is a silent password prompt |
| `NPU runtime` / `NPU model present` | the agent degrades silently and streams video while detecting nothing |
| `LiDAR healthy` | distinguishes a dead *scanner* from a dead *process* via `seq` |
| `rosbridge whitelist` | a drive topic reachable from a browser |
| `clock sane` | no RTC backup battery; boots to the filesystem epoch |

**It never moves the rover.** It does not turn-test for liveness — that
destroys heading — and it does not open the drive board's tty, because opening
it resets the ESP32 and costs a 90–225 s re-link. Port ownership is read from
`/proc` instead.

Exit codes: `0` all pass, `1` warnings only, `2` at least one failure.

### Warnings that are correct on a fresh image

- `calibration profile: none` — a missing profile changes nothing; a guessed
  one is silent and total.
- `LiDAR mount transform: unmeasured` — Nav2 and SLAM refuse to start. Intended.
- `NPU model present: FAIL` — the `.rknn` is not in the repository.
