# Flashing FPMS-OS to the Orange Pi 5B

## Before you start

Have: the `.img.xz` and its `.sha256`, a USB-C cable, and either a microSD card
(32 GB+, U3/A2) or the board's onboard eMMC.

**Verify the checksum first.** A partially downloaded image flashes fine and
fails in ways that look like hardware faults.

```sh
sha256sum -c fpms-os-1.0.0-<date>.img.xz.sha256
```

---

## Which storage

| | eMMC | microSD |
|---|---|---|
| Speed | much faster | fine |
| Reliability under power loss | better | the usual SD card story |
| Recovery if it will not boot | needs maskrom mode | pull the card, edit it on a laptop |
| **Recommended** | for competition | **for development** |

The rover gets power-cycled a great deal during development, and being able to
pull the card and read `/var/log` on a laptop has been worth more than the
speed. Move to eMMC once the configuration has settled.

---

## Option A — microSD (easiest)

**Windows:** [balenaEtcher](https://etcher.balena.io/) — select the `.img.xz`
directly, it decompresses on the fly. Or Raspberry Pi Imager → *Use custom*.

**Linux/macOS:**

```sh
lsblk                                    # IDENTIFY THE CARD. Get this wrong
                                         # and you overwrite your own disk.
xzcat fpms-os-1.0.0-<date>.img.xz | sudo dd of=/dev/sdX bs=4M status=progress conv=fsync
sync
```

**Do not eject yet** — see "Before first boot" below.

---

## Option B — eMMC (competition)

The Orange Pi 5B has onboard eMMC. Writing to it needs the board in maskrom
mode and Rockchip's tool.

```sh
# 1. install the tool (once)
git clone https://github.com/rockchip-linux/rkdeveloptool
cd rkdeveloptool && autoreconf -i && ./configure && make && sudo make install

# 2. board OFF. Hold the MASKROM button, connect USB-C to your PC, release.
sudo rkdeveloptool ld            # must list one device in Maskrom mode

# 3. load the loader, then write
sudo rkdeveloptool db rk3588_spl_loader.bin
xz -d fpms-os-1.0.0-<date>.img.xz
sudo rkdeveloptool wl 0 fpms-os-1.0.0-<date>.img
sudo rkdeveloptool rd            # reboot
```

The SPL loader comes from Rockchip's `rkbin` repository, or from the
`ubuntu-rockchip` release assets for this board.

> If an SD card is inserted, the board boots from it in preference to eMMC.
> That is the recovery path: flash a known-good card, boot from it, and fix
> the eMMC from a running system.

---

## Before first boot — put your WiFi on the card

This is the step that makes the rover headless. **Do it before the card goes
in the board.**

After flashing, the FAT boot partition mounts automatically on any machine —
on Windows it appears as a drive letter. On it you will find
`README-FPMS.txt` and `fpms-wifi.conf.example`.

1. Rename `fpms-wifi.conf.example` → `fpms-wifi.conf`
2. Edit it — one network per line, `SSID:PASSWORD`, **first line tried first**:

   ```
   MyMobileHotspot:mypassword123
   HomeWiFi:anotherpassword
   ```

   Put the **Windows Mobile Hotspot first**. That is the `192.168.137.x`
   subnet the mosquitto bridge points at.

3. Eject properly, put the card in, power on.

Optional files on the same partition, all one line each:

| File | Purpose |
|---|---|
| `fpms-hostname` | rover name, default `fpms-pi`. Change only if running two rovers at once. |
| `fpms-broker-host` | operator laptop IP, default `192.168.137.1` |
| `fpms-mqtt-password` | reuse an existing broker password instead of generating one |

If you skip all of this the rover still boots — it just starts its own setup
network (`FPMS-Rover-Setup` / `fpmsrover`) so you can reach it.

---

## First boot

**Three to five minutes.** Do not pull the power. It is:

- growing the rootfs to fill the card
- generating unique SSH host keys
- connecting to WiFi
- generating the MQTT broker password
- verifying the USB serial topology

If you have a monitor attached, the broker password is printed once, in a
banner. Write it down — you need it on the operator laptop. It is also stored
at `/var/lib/fpms/broker-password` (root only).

Subsequent boots take about 40 seconds to a usable console, and up to ~4
minutes for the micro-ROS drive link, which takes 90–225 s to establish and
cannot be hurried.

---

## Verify

```sh
ssh ubuntu@fpms-pi.local
fpms-selftest
```

You want `All checks passed`. Expect these warnings on a fresh image — they are
correct, not faults:

- **`calibration profile: none`** — distance and heading run on defaults until
  you characterise. See the runbook.
- **`LiDAR mount transform: unmeasured`** — Nav2 and SLAM refuse to start.
  Intended.
- **`NPU model present: FAIL`** — the `.rknn` model is not in the repository.
  Copy it from the old Pi.

Then open **`http://fpms-pi.local:8090/`**. Always the name, never an IP.

---

## If it does not come up

**No lights, no boot at all.** Re-flash; verify the checksum. If you flashed
eMMC, put a known-good SD card in — the board prefers it and you get a shell.

**Boots but `fpms-pi.local` does not resolve.** mDNS resolution is per-resolver,
not per-machine — in one session it worked from .NET and failed from Python's
`getaddrinfo` on the same box at the same time. So "ping works" does not prove
your tool will resolve it. Find the IP from your router or hotspot's client
list, or join the fallback AP.

**No WiFi.** Join `FPMS-Rover-Setup` (password `fpmsrover`), then
`ssh ubuntu@10.42.0.1`, then `sudo fpms-wifi-provision`. Ethernet always works.

**SSH refuses your password.** It will — the image is key-only by design. Put
your public key on the card's boot partition, or use the fallback AP and a
monitor. Note this also means `deploy_rover.py`, which authenticates with a
password today, needs pointing at a key.

**Everything green but the dashboard is empty.** Almost always the broker
password. `fpms-selftest` catches it; nothing else will, because every service
retries forever without logging an error.

---

## Re-flashing without losing your calibration

These are the only files worth keeping off a rover:

```sh
scp ubuntu@fpms-pi.local:/etc/fpms/calibration.json          .
scp ubuntu@fpms-pi.local:/etc/fpms/config.env                .
scp ubuntu@fpms-pi.local:~/.fpms_teleop_origin.json          .
scp -r ubuntu@fpms-pi.local:~/yolo                           .
scp -r ubuntu@fpms-pi.local:~/slam/maps                      .
```

Everything else is in the image or in git — which is the entire point of
FPMS-OS.

Do **not** restore `.fpms_teleop_origin.json` blindly across a firmware change.
It records a pose pair captured under a particular
`FPMS_MISSION_ODOM_POSE_SIGN`, and flipping that sign reflects the anchor
through the origin while producing perfectly plausible numbers. Re-zero with
`set_coordinate` instead.
