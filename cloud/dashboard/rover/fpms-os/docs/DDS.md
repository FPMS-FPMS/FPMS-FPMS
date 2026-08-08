# The DDS bug: connected, subscribed, silent

This document exists so nobody has to re-derive this in six months.

## The symptom

Everything says healthy. No data moves.

```
$ systemctl status fpms-lidar-ros
   Active: active (running)
$ journalctl -u fpms-lidar-ros
   scans=2848 dropped=0 rate=9.55 Hz
$ ros2 topic info /scan_lidar
   Publisher count: 1
$ ros2 topic echo /scan_lidar
   (nothing, forever)
```

There is no error anywhere. Not in the journal, not in the DDS logs, not in the
client. The publisher is genuinely publishing. The subscriber is genuinely
subscribed. They have discovered each other. Nothing arrives.

## It appeared twice, in two subsystems, and was diagnosed as two bugs

**First, as a LiDAR problem.** Recorded in `units/fpms-ros-settle.service`:

> the node is active, it logs `scans=2848 dropped=0 rate=9.55 Hz`, and
> `ros2 topic info /scan_lidar` reports "Publisher count: 1". And yet NO
> subscriber created afterwards ever receives a single LaserScan… Restarting
> the unit fixes it instantly and reproducibly.

**Second, as a rosbridge problem.** Recorded in `ROS_PORT.md`, after three
reproductions:

> **rosbridge only delivers topics whose PUBLISHER already existed when
> rosbridge started.** A publisher created afterwards is never discovered, and
> the client subscription asking for it is accepted and then silent forever.
>
> Evidence: after a Pi reboot, `fpms-rosbridge` started 22:26:33 and
> `fpms-odom-tf` 22:45:45 — nineteen minutes later — and `/odom` delivered 0
> messages while publishing healthily at 6.6 Hz. A restart gave 53 messages in
> 8 seconds.
>
> What to remember is the symptom, because it is the worst kind: **connected,
> subscribed, silent — no error anywhere.**

A third note, in `fpms-tf.service`, got closest to the mechanism:

> A DomainParticipant created in that window binds an interface set that is
> then invalidated: the node runs, the publisher is discoverable, it logs
> publishes with `dropped=0`, and NO subscriber ever receives anything.

## The actual cause

These are one bug.

Fast DDS enables a **shared-memory transport** by default and prefers it for
peers on the same host. Every ROS node on this rover is on the same host, so
essentially all traffic goes over SHM.

SHM works through segments in `/dev/shm` that participants map into their
address space. Those segments do not reliably survive across what amount to
separate **process eras** — a reboot, a systemd restart cycle, a stale segment
left by a process that died without cleaning up. A participant created in one
era and a participant created in another can complete *discovery* — which
happens over UDP multicast and is unaffected — and then fail every attempt to
actually move a sample, silently.

That is why the symptom is always "the one that started later gets nothing",
and why a restart always fixes it: the restart puts both participants in the
same era.

**"The publisher must pre-exist the subscriber" is a description of the
symptom, not a cause.** It is also why the mitigations look so strange: they
are all attempts to force everything into one era.

## What the mitigations cost

| Mitigation | Cost |
|---|---|
| `fpms-ros-settle.service` | sleeps **75 seconds** after boot, then blindly restarts two units. About two minutes of boot-to-usable. |
| `deploy_stack.sh` consumer block | must restart rosbridge, both Foxglove units and the console *after* every publisher, with a long comment explaining why |
| `fpms-uros-supervisor` | incidentally re-creates four participants, papering over it further |
| the ordering rule itself | a permanent constraint on how you are allowed to restart anything |

None of them is a fix.

## The fix

`/etc/fpms/fastdds_udp_only.xml`, referenced by
`FASTRTPS_DEFAULT_PROFILES_FILE` in **every** ROS unit.

It declares one UDPv4 transport and — this is the load-bearing part —

```xml
<useBuiltinTransports>false</useBuiltinTransports>
```

Without that line the profile does nothing useful. Listing a UDPv4 transport in
`<userTransports>` **adds** it; the builtin set, which includes SHM, is still
present and Fast DDS still prefers SHM for same-host peers. A profile without
that line looks correct, parses correctly, and fixes nothing.

Every unit also sets `ROS_LOCALHOST_ONLY=1`. That is complementary, not
redundant: localhost-only restricts which *interfaces* are used but does **not**
disable the shared-memory transport. You need both. It is safe here because
every ROS node runs on the Pi — the operator laptop reaches the rover over the
rosbridge/Foxglove websockets and MQTT, and neither uses DDS.

Keeping DDS off the WiFi has a second benefit. The link has been measured at
~108 ms RTT on the operator hotspot where 2–5 ms is expected, and swings
between −47 and −78 dBm; below about −70 dBm the LiDAR telemetry stops
entirely. Discovery traffic does not belong on that link.

## A malformed profile is silently ignored

This is important enough to have its own section.

Fast DDS does not fail loudly on a profile it cannot parse. It logs at a level
nobody reads and **falls back to defaults** — which reintroduces exactly the
bug the file exists to remove, while the file sits there looking correct.

So it is checked three times:

1. `selftest/test_image_offline.py` parses it and asserts
   `useBuiltinTransports` is false — on the build machine, no hardware needed.
2. `scripts/40-ros-layer.sh` does the same inside the chroot and **fails the
   build** if it does not hold.
3. `fpms-selftest` re-checks it on every boot of the real rover.

## How to verify it worked

The check must exercise the case that actually fails: a subscriber created
**long after** the publisher. `ros2 topic list` and `ros2 topic info` pass
either way, because discovery was never the broken part — that is what made
this so hard to see.

Leave the stack running for a few minutes, then in a fresh shell:

```sh
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fpms/fastdds_udp_only.xml
ros2 topic hz /scan_lidar        # must report a rate, not hang
```

`fpms-selftest` runs exactly this on every boot, as
`DDS carries data across eras`. It is the single most valuable check in the
file.

## If it does not work

Then the root cause on this board is something other than SHM, this document
is wrong, and it should be corrected rather than worked around.

Fall back to the old mitigation:

```sh
sudo systemctl enable --now fpms-ros-settle.service
```

That unit ships with the image, disabled, for precisely this reason. Removing a
mitigation for a bug you have not confirmed fixed on hardware is a mistake this
project has made before.

The other escape hatch is a different middleware entirely —
`ros-humble-rmw-cyclonedds-cpp` is installed. Cyclone's shared-memory support
is off unless Iceoryx is configured, so it sidesteps this class of problem:

```sh
# in /etc/fpms/config.env, then restart the stack
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Be aware this changes the middleware for the **entire** stack including the
micro-ROS agent, and that every unit's `Environment=RMW_IMPLEMENTATION` would
need to change with it — a mixed-RMW stack discovers nothing and, in the
tradition of this whole document, reports no error.
