# Rover source sync manifest

What is actually deployed on the rover Pi (`fpms-pi`, 192.168.137.184, thing
`rover2`), copied into this repo so the repo reflects the running system.

Every file below was verified byte-for-byte: the sha256 was read from the Pi
with `sha256sum` BEFORE the transfer and recomputed locally after it, and any
mismatch was re-downloaded. That check is not ceremony — the first attempt over
this link produced a 0-byte `fpms_missions.py` that reported success, and an
unverified copy would have committed it.

`units/` is authoritative for the systemd units. Some `.service` files also sit
in the parent directory from earlier sessions; those are older and were left
untouched rather than deleted.

**No credentials are in this directory.** `/etc/fpms/config.env` holds the
broker password and is deliberately NOT synced — see `config.env.example` for
the same keys with the secret redacted.

Synced: 2026-08-04 15:55

| repo path | rover path | bytes | sha256 |
|---|---|---:|---|
| `fpms_missions.py` | `/home/ubuntu/fpms_missions.py` | 243939 | `ac4aa3043af3c7707dc4821ef0079dee4d74cb412a605aa38fbe64d00850b8b9` |
| `fpms_teleop.py` | `/home/ubuntu/fpms_teleop.py` | 142093 | `28a8d30d9bfc23444efd66e80419dd7dffbe91df9ce8207c547b07464f9841d5` |
| `fpms-rover-agent.py` | `/home/ubuntu/fpms-rover-agent.py` | 30603 | `5dbdc87e50fd941443723b60741fd5803e968ff090ad31120cf051cf0620def1` |
| `fpms_lidar_ros.py` | `/home/ubuntu/fpms_lidar_ros.py` | 22736 | `2f413fe378ee96582eb2bbfed4de42aca6989e065e4f60a2390277acba4e3153` |
| `fpms_ros_tunnel.py` | `/home/ubuntu/fpms_ros_tunnel.py` | 38992 | `e8b787ee1522d55e9660817f452ab6bc4e633462686090d580f2c9ffe92e03ca` |
| `fpms_rtos_follower.py` | `/home/ubuntu/fpms_rtos_follower.py` | 49299 | `09c8f8d6dfac10f8324e687844ec9d36b963dea2a4c3b220f53851eb78cf12ca` |
| `units/fpms-lidar-ros.service` | `/etc/systemd/system/fpms-lidar-ros.service` | 1796 | `5fe35e8819b73361a52b7c9c83da1e0694f47fe6ddb88a15769c41c313ad8e0f` |
| `units/fpms-map-odom.service` | `/etc/systemd/system/fpms-map-odom.service` | 509 | `9c3723a82c062ba464e8f843f0e31b48b784e29c43256fc5983de4ee20e6d115` |
| `units/fpms-missions.service` | `/etc/systemd/system/fpms-missions.service` | 2792 | `ec815a8e6973cc260026291522d39dbd1e16c3923c473814db04348944c5166a` |
| `units/fpms-odom-tf.service` | `/etc/systemd/system/fpms-odom-tf.service` | 1428 | `7b69188075564e6fe033599ae4978a806c2a84bbba981f78bc905df6745b4d92` |
| `units/fpms-ros-tunnel.service` | `/etc/systemd/system/fpms-ros-tunnel.service` | 2099 | `0d7e52ecd4e4c9ceb2941410f587080ca19a83b0afb3d6ad64ccf0b1571173d8` |
| `units/fpms-rover-agent.service` | `/etc/systemd/system/fpms-rover-agent.service` | 349 | `25ea6b651d6a0b6ccf00215022eca164010d509304d15d2f00d95b3710d89791` |
| `units/fpms-rtos-follower.service` | `/etc/systemd/system/fpms-rtos-follower.service` | 2961 | `4b38c38d13a9ad8cb282dfc3be6565a420ffb1de1e2a2cb7808e6f9738377ef0` |
| `units/fpms-teleop.service` | `/etc/systemd/system/fpms-teleop.service` | 1034 | `44fa98da05abee2ae6b95e075634e7f0a4f78df095bd0ac2d882d3118251ca87` |
| `units/fpms-tf.service` | `/etc/systemd/system/fpms-tf.service` | 434 | `f1ed932528a1d45be0205bb4f0ff85d990d901eccd7140d969e8b71e9ecbe798` |
| `units/fpms-wifi-powersave-hold.service` | `/etc/systemd/system/fpms-wifi-powersave-hold.service` | 277 | `219ac4396ca1b2a68f370fd091f0de8d7e8781a21b5263b9a3198157e91460ad` |
| `units/micro-ros-agent.service` | `/etc/systemd/system/micro-ros-agent.service` | 2472 | `799f60827c57e9bbd6315e77d251f64af573a793100cdd590d10a3187b385db3` |
