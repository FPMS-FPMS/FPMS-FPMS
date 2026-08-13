# First boot

Everything that cannot be baked into an image shared by every rover.

`fpms-firstboot.service` runs once, before `mosquitto` and the FPMS stack, then
disables itself. Re-run by hand with `sudo fpms-firstboot --force`.

## What it does

| Step | Why it cannot be in the image |
|---|---|
| grow the rootfs | the image is built small so it compresses and flashes fast |
| generate SSH host keys | a shared host key across a fleet is a real problem, and it makes every laptop's `known_hosts` conflict between rovers |
| provision WiFi | credentials are per-site, and there was **no mechanism at all** in this project before |
| set hostname + `/etc/hosts` | avahi publishes the `.local` name from these, and `fpms_console.py` builds its printed URL from `socket.gethostname()` |
| generate the broker password | **no default is ever baked in** |
| verify USB topology | it is a property of the physical rover, not the image |

## The broker password

Generated per board and printed **once** to the console, with the username, in
a banner. Also stored at `/var/lib/fpms/broker-password`, root only.

This matters more than it looks. If `config.env` holds a password the broker
does not accept, **nothing crashes and nothing logs an error** — every service
retries forever and the dashboard is silently empty. `fpms-selftest` fails if
the placeholder is still in place, because that is the only way anyone would
find out.

To reuse an existing password, put it in `fpms-mqtt-password` on the FAT boot
partition before first boot.

## The topology check refuses to guess

It confirms a device is present at USB port 1.3 (drive board) and 1.2 (LiDAR).
If they are missing or swapped it writes a loud fault and **changes nothing**.

Both CP2102 adapters report `ID_SERIAL 0001`, so only the USB port
distinguishes them, and binding wrong sends motor command frames into the laser
scanner. `CLAUDE.md` is explicit: do not resolve this by trying both.

It also never *opens* either device — opening the drive tty raises the
handshake lines wired to EN/IO0 and resets the ESP32, costing a 90–225 s
re-link.

## Design rule: never block the boot

Every step is individually fenced and the service is `SuccessExitStatus=0 1`.
A rover that came up with a bad hostname is recoverable over the network; a
rover that refused to boot because a WiFi file had a typo is a card-reader job.

Problems are collected into `/var/lib/fpms/firstboot.json` and reported by
`fpms-selftest`.

## Files on the boot partition

All optional, all one line, all CRLF-tolerant because they are edited in
Notepad. See `overlay/boot/README-FPMS.txt` — that is the operator-facing copy.

| File | Default |
|---|---|
| `fpms-wifi.conf` | none — falls back to an AP |
| `fpms-hostname` | the image's own name (`fpms-rover1`) |
| `fpms-broker-host` | `192.168.137.1` |
| `fpms-mqtt-password` | generated |

`fpms-wifi.conf` is renamed to `.applied` and locked down after it is read, so
the password is not left in cleartext on a partition every Windows machine
mounts automatically.
