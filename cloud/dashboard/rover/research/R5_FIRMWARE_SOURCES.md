# R5 — FIRMWARE SOURCE HUNT

**Board:** Yahboom MicroROS Board V2.0 (ESP32-S3, CP2102-GMR USB-UART)
**Date:** 2026-08-02
**Status:** Research only. No rover access, nothing flashed, no code changed.
**Supersedes gaps in:** `R4_FIRMWARE.md` §2 and §6 (several are now closed).

---

## 0. HEADLINE — four of R4's seven evidence gaps are now closed

R4 could not open the Google Drive folder and therefore could not say whether Yahboom
publishes firmware, source, or neither. **That is now resolved, and the answer is
better than R4 feared in one respect and worse in another.**

| R4 gap | Status now |
|---|---|
| "No Yahboom firmware `.bin` URL was found or confirmed" | **CLOSED.** `Factory-Firmware.zip` (882 KB) exists and is enumerated, with a file ID. The factory image is recoverable. |
| "Whether Yahboom publishes ESP32 firmware source is unresolved" | **CLOSED, split answer.** Integrated factory-firmware source: **NOT published** — Yahboom says so in writing. Per-peripheral ESP-IDF + micro-ROS **sample source: IS published**, in `Samples.zip` (139 MB). |
| "The stock firmware version number is unknown" | **PARTIALLY CLOSED.** `microROS_Robot_V0.0.4` is referenced in Yahboom's own tutorial. |
| "The PrwTsrt GPIO pin map is unverified against Board V2.0" | **CLOSED.** Vendor pin map obtained from two independent Yahboom pages; PrwTsrt uses the **identical pin set** with M1/M2 swapped. |

The two headline consequences:

1. **The Phase 0 backup is no longer the only insurance.** A factory image is
   independently downloadable, so a bad flash is recoverable even if the backup fails.
   This materially lowers the risk of the whole exercise.
2. **There is no licensed vendor source.** Yahboom publishes sample code with **no
   licence attached**, and the integrated firmware source not at all. Every
   board-specific option is therefore licence-encumbered for a PUBLIC repo. This is now
   the binding constraint, not the technical one.

---

## 1. YAHBOOM'S OWN SOURCE — definitive answer

### 1.1 Yahboom states plainly that firmware source is not provided

Verbatim, from Yahboom's own flashing tutorial:

> "The factory firmware of the microROS control board only provides bin files for
> burning, and does not provide program source code."

Source: <http://www.yahboom.net/public/upload/upload-html/1708423522/1.%20Write%20firmware.html>

The same sentence (with a copy-paste slip, "the camera", betraying a shared template)
appears in the firmware-update page:
<https://www.yahboom.net/public/upload/upload-html/1737552446/How%20to%20Update%20the%20Firmware.html>

**This is dispositive.** The integrated `microROS_Robot_Vx.x.x` firmware source is not
published anywhere, by design, and no amount of further searching will find it. R4 was
right to refuse to assume otherwise.

### 1.2 The Drive folder — traversed, contents confirmed

R4 could not enumerate this. It is traversable via the `embeddedfolderview` endpoint,
which returns a plain HTML listing where the normal Drive UI returns a JS shell.

**"Firmware And Code"**, linked from <https://www.yahboom.net/study/MicroROS-Board>:
<https://drive.google.com/drive/folders/1Vp9qGsNR_A76mk58JC9xvXpZ0nmrffEy?usp=sharing>

| File | Size | File ID | Modified |
|---|---|---|---|
| `Factory-Firmware.zip` | 882 KB | `1SEmSzRV3Mtjzd30ZhCSzmF6MQwGZro-k` | 2024-08-04 |
| `ROS_Source_Code.zip` | 6.3 MB | `1zEqi7tDXXVmfVV_7DZYpixIKXpsB8SAk` | 2024-08-04 |
| `Samples.zip` | 139 MB | `19fjOE6TT8YUISrM4g3g7L-Png_iCw6Hc` | 2024-08-04 |

A second, near-identical copy is published for the MicroROS-ESP32 robot
(<https://www.yahboom.net/study/MicroROS-ESP32> → "Code and Firmware",
<https://drive.google.com/drive/folders/1mDx8VEdl75yw8wrlYWQkmNEfY_TB5HVl?usp=sharing>),
dated 2024-08-11 — slightly newer, same three filenames, different IDs
(`1Wjdj11BZhXjgDY9Cr5eIESCmBwCgXna_`, `1MD3qMkMstuSTge-RlACcjcqQAACPOF3-`,
`13Iv4NzWCAEDFAj_Ksnq0j1pqmSmluhLP`).

The `AllFile_Download_Link.txt` in the GitHub repo points at a *third* folder,
`1ZJ-egn8Co_nm32R35ND8S-OIpbhNaF60` (the one named in the brief). **That folder
returns HTTP 404** — it has been deleted or unshared. It is a dead link and should not
be used. The two folders above are live.

Direct-download URL form (untested — do not assume it bypasses Drive's virus-scan
interstitial for files this large):
`https://drive.google.com/uc?export=download&id=<FILE_ID>`

**What is in each zip — verified vs inferred:**

- `Factory-Firmware.zip` — **inferred: the `.bin` only.** 882 KB compressed is
  consistent with a single ~1.5–2 MB application image, and Yahboom states bin-only.
  I did not open it. *Confidence: high.*
- `ROS_Source_Code.zip` — **inferred: host-side ROS2 packages** (the Pi-side driver
  node), not ESP32 firmware. 6.3 MB and the name match Yahboom's usual layout.
  *Confidence: medium.* This is the trap R4 correctly warned about.
- `Samples.zip` — **inferred: the ESP-IDF sample tree.** See §1.3; this is the
  important one. *Confidence: high.*

### 1.3 The real find — Yahboom DOES publish per-peripheral ESP-IDF source

Yahboom's tutorials cite concrete sample-project paths inside the extracted tree:

| Path | Source page |
|---|---|
| `~/esp/Samples/esp32_samples/encoder` | <http://www.yahboom.net/public/upload/upload-html/1708418518/Read%20motor%20encoder%20data.html> |
| `~/esp/Samples/microros_samples/imu_publisher` | <https://www.yahboom.net/public/upload/upload-html/1708420558/Release%20IMU%20data%20topic.html> |

The tutorial index covers **exactly the seven topics this rover exposes** — driving
encoder motors, PID speed control, reading the IMU, battery-voltage detection,
subscribing to the buzzer topic, subscribing to the PWM-servo topic, publishing IMU,
subscribing to `cmd_vel`. The IMU sample names the actual part: **ICM42670P**.

**This is the decisive discovery.** Yahboom withholds the *integrated* firmware but
ships *driver-level source for every peripheral on the board*, with the correct pin
map, the correct IMU part, and working micro-ROS publisher/subscriber scaffolding for
`/imu`, `/battery`, `/beep`, `/servo_s1`, `/servo_s2`. R4 assumed those four topics
would have to be reverse-engineered or lost. **They do not — the source for them
exists and is downloadable.**

**Caveat, stated plainly:** I have not opened `Samples.zip`. The path evidence is
strong and independently corroborated (§2.1), but the exact contents are inferred.

### 1.4 Licence — the blocker

`Samples.zip` carries **no licence**. The GitHub repo `YahboomTechnology/MicroROS-Board`
has **no LICENSE file**. Yahboom states no terms of reuse anywhere I could find.

Default position: **all rights reserved.** Vendoring this into a PUBLIC repo is a
licensing problem. Mitigation in §6.

### 1.5 Other Yahboom orgs/repos checked

- `YahboomTechnology/MicroROS-Board` — docs only, no `.bin`, no firmware source, no LICENSE
- `YahboomTechnology/Mirco-Ros-Car_VM` — VM/host-side
- `YahboomTechnology/MicroROS-Car-Pi5` — host-side Pi5 robot code
- `www.yahboom.net/study/MicroROS-Pi5`, `/MicroROS-ESP32`, `/SBR-microROS` — same
  Drive-hosted layout, no ESP32 firmware source

**No mirror of the factory firmware source exists on GitHub.** Searched; nothing.

---

## 2. THIRD-PARTY FIRMWARE FOR THIS EXACT BOARD

### 2.1 `PrwTsrt/microros_esp32_diffdrive` — now understood to be a Yahboom derivative

<https://github.com/PrwTsrt/microros_esp32_diffdrive> — 8 stars, 0 forks, 1 watcher.

**New finding: this is not independent work. It is Yahboom's sample tree, integrated.**

Three independent proofs:

1. Its README's build steps say to clone the micro-ROS component into
   **`~/esp/Samples/extra_components`** — the *same* `~/esp/Samples/` root Yahboom's
   tutorials use. The author extracted `Samples.zip` and built inside it.
2. It uses Yahboom's exact macro names: `ENCODER_GPIO_H1A`, `ENCODER_GPIO_H1B` —
   identical to the identifiers quoted in Yahboom's encoder tutorial.
3. Its pin set is Yahboom's pin set (§4).

**Consequences.** The licence question is not really about PrwTsrt — it is about
Yahboom's underlying code, which PrwTsrt redistributes without permission or notice.
Asking PrwTsrt to add a licence, as R4 suggested, **would not cure this**; they are not
the copyright holder of the parts that matter. *(Assessment, not legal advice.)*

Confirmed: **no LICENCE.** `https://api.github.com/repos/PrwTsrt/microros_esp32_diffdrive/license`
returns **HTTP 404**. R4's finding stands, now with an API-level check.

Confirmed unchanged: `PWM_MOTOR_DEAD_ZONE (200)` — the defect R4 identified is present
here too. Verbatim from `components/pwm_motor/pwm_motor.h`:

```c
#define PWM_MOTOR_TIMER_RESOLUTION_HZ 10000000
#define PWM_MOTOR_FREQ_HZ 25000
#define PWM_MOTOR_DUTY_TICK_MAX (PWM_MOTOR_TIMER_RESOLUTION_HZ / PWM_MOTOR_FREQ_HZ)
#define PWM_MOTOR_DEAD_ZONE (200)
#define PWM_MOTOR_MAX_VALUE (PWM_MOTOR_DUTY_TICK_MAX-PWM_MOTOR_DEAD_ZONE)
```

### 2.2 Other third-party firmware for this board — searched, none found

Swept GitHub topics `micro-ros-esp32`, `micro-ros`, `differential-drive-robot`, plus
targeted searches. Every repo surfaced is either unrelated to this board or generic:

| Repo | Relevance |
|---|---|
| `PrwTsrt/micro_ros_action_server` | Same author, ROS actions demo, not drive firmware |
| `camrbuss/diff-drive-esp32-uros` | ESP32 + ODrive — different hardware entirely |
| `2b-t/esp32s3-microros` | ESP32-S3 camera streaming, no motors |
| `kaiaai/firmware` | Apache-2.0, 60★, but **2-motor differential only** (4WD listed TODO), Arduino/ESP32, not this board |
| `dabmake/ESPROS` | ESP8266/ESP32, generic, unmaintained |
| `ALUIS97/uROS_mobile_robot_1` | 0★, no board match |

**Conclusion: `PrwTsrt` is the only public firmware project targeting this board.**
R4's assessment was correct and remains so.

---

## 3. GENERIC LICENSED ALTERNATIVES

### 3.1 `linorobot2_hardware` — the only clean-licence serious contender

Upstream: <https://github.com/linorobot/linorobot2_hardware>
Active fork: <https://github.com/hippo5329/linorobot2_hardware> (ESP32 work originated
here, since merged upstream)

| Criterion | Finding |
|---|---|
| **Licence** | **Apache-2.0** — verified by reading the LICENSE file. *(The brief said MIT. It is Apache-2.0. Both are fine for a public repo; Apache-2.0 additionally grants patent rights.)* |
| **ESP32-S3** | **Yes, explicitly.** "Only esp32 esp32-s2 and esp32-s3 are supported." Configs `esp32s3_config.h` (serial) and `esp32s3_wifi_config.h` (WiFi) |
| **4 motors** | **Yes.** `SKID_STEER` (4WD) and `MECANUM` alongside `DIFFERENTIAL_DRIVE` |
| **Encoders + IMU + PID + odometry** | **Yes, all four.** Plus LiDAR forwarder |
| **Build** | PlatformIO — `pio run -e esp32s3 -t upload`. Far simpler than ESP-IDF v5.1.2 |
| **Maintained** | Yes, actively |
| **Transport** | micro-ROS over serial or WiFi |

**Rework required for the Yahboom board — larger than it first appears:**

1. **Pin remap** — mechanical, low risk. `lino_base_config.h`, using §4's map.
   Linorobot's stock ESP32-S3 map (motors 10–17, encoders 4–7/39–42, I2C 8–9)
   **collides badly** with Yahboom's (I2C on 39/40, encoders on 47/48 and 1/2,
   servos on 8/21). Every pin must be overridden; none can be left at default.
2. **IMU driver — the real cost.** Linorobot supports GY-85, MPU6050, MPU9150,
   MPU9250, QMI8658. The Yahboom board has an **ICM42670P**, which is **not on that
   list**. A new driver must be written or ported. Non-trivial.
3. **Topic renames break the host stack.** Linorobot publishes `/odom/unfiltered` and
   `/imu/data`; this rover's stack consumes `/odom_raw` and `/imu`. Remap in firmware
   or host-side.
4. **`/battery`, `/beep`, `/servo_s1`, `/servo_s2` do not exist** and must be written
   from scratch — *unless* cribbed from Yahboom's samples, which reintroduces the
   licence problem you switched to linorobot to escape.
5. **PID retune** from scratch for these motors.

**Honest verdict:** clean licence, real engineering. Not a drop-in.

### 3.2 `micro-ROS/micro_ros_espidf_component` — substrate, not an alternative

<https://github.com/micro-ROS/micro_ros_espidf_component> — **Apache-2.0**, ESP32-S3
supported, ESP-IDF v5.4/v5.5/v6.0 validated, branches for humble/jazzy/kilted/rolling,
UDP and UART transports, actively maintained.

This is the **transport/middleware layer, not robot firmware**. It contains no motor,
encoder, IMU, or odometry code. You need it under *any* ESP-IDF option including
PrwTsrt and Yahboom's samples. It is not a competing choice — it is a dependency.
Note it now targets ESP-IDF v5.4+, whereas PrwTsrt pins v5.1.2; version skew is a
likely build friction point.

### 3.3 `Reinbert/ros2_esp32` — not assessed

Named in the brief. Did not surface in any search I ran, and I did not fetch it. **I am
not going to assess a repo I have not opened.** R4 was right that a previous pass
listed repos seen only in a search index; I will not repeat that.

### 3.4 diffbot firmware — rejected on relevance

ROS 1-era, Teensy/Arduino-oriented, 2-motor. Not a credible base for an ESP32-S3
4-motor micro-ROS board.

---

## 4. THE GPIO / PIN MAP — obtained, cross-validated, HIGH confidence

**This is the single most valuable output of this pass.** R4 flagged it as the most
likely thing to brick a build. It is now nailed down.

### 4.1 Vendor pin map

Published by Yahboom on **two independent pages**, byte-identical between them:

- <http://www.yahboom.net/public/upload/upload-html/1708480467/Introduction%20to%20microROS%20control%20board.html>
- <http://www.yahboom.net/public/upload/upload-html/1713429128/Brief%20introduction%20of%20microROS%20control%20board.html>

| Function | Pins |
|---|---|
| **Motor M1 PWM** | GPIO4 (M1A), GPIO5 (M1B) |
| **Motor M2 PWM** | GPIO15 (M2A), GPIO16 (M2B) |
| **Motor M3 PWM** | GPIO9 (M3A), GPIO10 (M3B) |
| **Motor M4 PWM** | GPIO13 (M4A), GPIO14 (M4B) |
| **Encoder M1** | GPIO6 (H1A), GPIO7 (H1B) |
| **Encoder M2** | GPIO47 (H2A), GPIO48 (H2B) |
| **Encoder M3** | GPIO11 (H3A), GPIO12 (H3B) |
| **Encoder M4** | GPIO1 (H4A), GPIO2 (H4B) |
| **IMU I2C (ICM42670P)** | SCL GPIO39, SDA GPIO40, INT GPIO41 |
| **Servo S1 / S2** | GPIO8 / GPIO21 |
| **LiDAR (MS200, UART1)** | TX GPIO17, RX GPIO18 |
| **Type-C serial (UART0)** | TX GPIO43, RX GPIO44 |
| **Battery ADC** | GPIO3 |
| **Buzzer** | GPIO46 |
| **MCU LED** | GPIO45 |
| **BOOT / KEY1** | GPIO0 / GPIO42 |
| **Spare UART (WiFi cam)** | GPIO35 / GPIO36 |

### 4.2 Independent cross-validation against PrwTsrt source

| | PrwTsrt PWM | PrwTsrt encoder |
|---|---|---|
| Ch1 | M1A **16**, M1B **15** | H1A **47**, H1B **48** |
| Ch2 | M2A **4**, M2B **5** | H2A **7**, H2B **6** |
| Ch3 | M3A **9**, M3B **10** | H3A **11**, H3B **12** |
| Ch4 | M4A **13**, M4B **14** | H4A **1**, H4B **2** |

The **sets are identical** to the vendor map: PWM `{4,5,15,16,9,10,13,14}`, encoders
`{6,7,47,48,11,12,1,2}`. The only difference is that **PrwTsrt's channel 1 is
Yahboom's M2 and its channel 2 is Yahboom's M1** — a consistent swap across *both* PWM
and encoder, i.e. the author renumbered channels to match their chassis wiring, not a
different board revision. Note also H2A/H2B is `7,6` where Yahboom lists `6,7` —
a phase inversion, which flips encoder count sign.

**Confidence: HIGH.** Two vendor pages agree with each other and with independently
written third-party source. Three sources, one map.

**Two caveats, stated honestly:**

1. Yahboom's docs do not carry an explicit "V2.0" marking. This map is for "the
   microROS control board". A silent V1→V2 pin change cannot be excluded from
   documents alone — though the fact that PrwTsrt's independently-authored code matches
   makes it unlikely.
2. **No schematic PDF was found.** The Hardware Information folders contain
   `310 motor parameters.pdf`, `CP2102-GMR_Specification.PDF`, the CP2102 Windows
   driver, an IMU sensor folder and the flash tool — **but no board schematic**.
   <https://drive.google.com/drive/folders/1klzxonweuEEfFQa3jetL4P3L6vyZWkr4?usp=sharing>
   and <https://drive.google.com/drive/folders/1Akl4eNcx0iPCiOKzo19IAXzehRkr9kRv?usp=sharing>.
   Yahboom does not publish schematics for this board.

**Channel→wheel mapping is still unknown.** Which physical wheel is M1 vs M2 vs M3 vs
M4, and each encoder's sign, must be determined empirically on blocks. That is a
bring-up step, not a research gap.

---

## 5. FLASHING

### 5.1 USB-UART chip: CP2102-GMR — confirmed

Confirmed by `CP2102-GMR_Specification.PDF` and `CP2102-Windows driver file.zip` in
Yahboom's Hardware Information folder (§4.2). The brief's "CP210x" is correct. The
ESP32-S3's native USB-Serial-JTAG is **not** the path in use; a bridge is.

*(This slightly weakens one of R4's recovery arguments — R4 cited native USB-Serial-JTAG
as a recovery path. On this board the Type-C port is wired to UART0 via the CP2102,
GPIO43/44. The ROM bootloader UART download path over that bridge still works, and
BOOT+RESET still forces it, so the conclusion "hard to brick" survives — but not for
the native-USB reason R4 gave.)*

### 5.2 Does auto-reset (DTR/RTS) work? — Probably yes, with a documented fallback

**Evidence for auto-reset being wired:** Yahboom's instruction is *conditional* —

> "**If** the firmware burning does not start automatically, please press and hold the
> boot0 key first, then press the reset button, release the boot0 key, and enter the
> burning mode manually."

Source: <http://www.yahboom.net/public/upload/upload-html/1708423522/1.%20Write%20firmware.html>

The phrasing "if it does not start automatically" implies the normal case *is*
automatic, i.e. the DTR/RTS→GPIO0/EN transistor pair is populated. The CP2102-GMR
exposes both control lines.

Espressif's reference: esptool asserts DTR and RTS on the bridge (FTDI, CP210x or
CH340x), which drive GPIO0 and EN (CHIP_PU).
<https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html>

**Evidence against / uncertainty:** Yahboom repeats the manual fallback in *every*
flashing document, which is either prudence or experience. **No schematic exists to
settle it (§4.2), and I have not seen the board.**

**Verdict: auto-reset is LIKELY to work. Confidence: MEDIUM — inferred from wording,
not verified.** Plan for it working; be ready to do BOOT+RESET. The fallback is
100% reliable and costs seconds:

> **Hold BOOT (GPIO0) → tap RESET → release BOOT.**

This forces the ROM serial bootloader regardless of flash contents and cannot be
disabled by any `write_flash`. Note this **is** a manual physical step, and given the
operator's constraint it should be assumed necessary rather than hoped away — someone
must be at the rover with a hand on the board for the first flash.

### 5.3 Backup — exact commands

Run these **before anything else**. `esptool` v4 syntax (underscores); v5 renamed the
subcommands to hyphens (`flash-id`, `read-flash`) — both shown.

```bash
pip install esptool

# 1. Identify chip and REAL flash size. Do not skip: the read length depends on it.
esptool.py --port COM7 flash_id
#   esptool v5:  esptool --port COM7 flash-id

# 2. Confirm flash encryption and secure boot are OFF.
#    If either is ON, a raw dump is encrypted or refused and is NOT restorable. STOP if so.
esptool.py --port COM7 get_security_info
#   esptool v5:  esptool --port COM7 get-security-info

# 3. Full backup. 'ALL' auto-sizes to the detected flash, avoiding a wrong-length dump.
esptool.py --chip esp32s3 --port COM7 --baud 460800 \
  read_flash 0 ALL yahboom_v2_factory_backup.bin
#   esptool v5:  esptool --chip esp32s3 --port COM7 --baud 460800 \
#                  read-flash 0 ALL yahboom_v2_factory_backup.bin

# If 'ALL' is unsupported on your version, substitute the size from step 1, e.g. 8 MB:
#   ... read_flash 0 0x800000 yahboom_v2_factory_backup.bin
```

Restore:

```bash
esptool.py --chip esp32s3 --port COM7 --baud 921600 \
  --before default_reset --after hard_reset \
  write_flash -z --flash_mode keep --flash_freq keep --flash_size keep \
  0x0 yahboom_v2_factory_backup.bin
```

Notes:
- **Read at 460800, not 921600.** A silently corrupted backup is worse than none.
  Check the output size and record its SHA-256.
- Store the dump **outside the public repo** — multi-MB vendor firmware.
- **Never run `espefuse.py burn_efuse`.** That is the only genuinely unrecoverable
  command (R4 §4 covers this correctly).

### 5.4 Vendor flash procedure (for the Yahboom `.bin`)

Espressif **Flash Download Tool**, SPIDownload, ESP32-S3:

- Single-file factory image: `microROS_Robot_Vx.x.x.bin` at **`0x0`**, tick
  `DoNotChgBin`
- Three-file image: `bootloader.bin` @ `0x0000`, `partition-table.bin` @ `0x8000`,
  `microROS_Robot.bin` @ `0x10000`
- Verify: serial terminal **115200 8N1**, press RESET, board prints its version banner

Version seen referenced in Yahboom's own material: **`microROS_Robot_V0.0.4`**
(<https://www.yahboom.net/public/upload/upload-html/1708570175/First%20Trial.html>).
Whether the rover currently runs V0.0.4 is unknown — read the banner to find out.

---

## 6. RANKED SHORTLIST

### #1 RECOMMENDATION — Yahboom `Samples.zip`, built in-tree, firmware kept OUT of the public repo

<https://drive.google.com/drive/folders/1mDx8VEdl75yw8wrlYWQkmNEfY_TB5HVl?usp=sharing>
(2024-08-11 copy, newer than the MicroROS-Board one)

**Why it wins.** It is the only option where *every* hard problem is already solved by
the vendor: the correct pin map, the correct IMU part (ICM42670P), the correct PCNT
encoder setup, and micro-ROS scaffolding for all seven topics the rover actually
exposes — including `/battery`, `/beep`, `/servo_s1`, `/servo_s2`, which R4 expected to
lose. The fix R4 identified is a **one-constant edit** on top of it. Nothing else on
this list gets you to a working rover faster or with less novel code.

- **Licence:** **None stated → all rights reserved.** This is the one real objection,
  and it is a distribution problem, not a use problem. Building and flashing vendor
  sample code onto the vendor's own board that you bought is ordinary intended use.
  **Publishing it in a public repo is what you must not do.**
- **Mitigation (required):** keep the firmware tree entirely out of the public repo.
  Options, in order of preference: (a) a separate **private** repo; (b) `.gitignore`d
  local build directory plus a checked-in **patch file** recording only *your* edits —
  a diff of constants you authored is your work and is safe to publish; (c) document
  the constant changes in Markdown and have the operator apply them. Option (b) is the
  sweet spot: reproducible, and publishes nothing of Yahboom's.
- **Effort:** ~1 day (toolchain is the bulk). Lower than R4's 1.5–3 day estimate for
  PrwTsrt, because the pin map is no longer a bring-up unknown.
- **Risk:** **Low.** Factory `.bin` independently downloadable (§1.2) *and* a local
  backup (§5.3) — two independent recovery paths. Pin map cross-validated (§4).
- **Unverified:** zip contents not opened; must be confirmed on download. If
  `Samples.zip` turns out not to contain the integrated `main`, you assemble from the
  per-peripheral samples — more work, but every driver is still there.

### #2 — `PrwTsrt/microros_esp32_diffdrive`

<https://github.com/PrwTsrt/microros_esp32_diffdrive>

Same code as #1 but **already integrated into one buildable ESP-IDF project** — that
integration is genuine value and is why it ranks this high. Take it if `Samples.zip`
disappoints.

- **Licence: NONE (404 verified), and it redistributes Yahboom's code**, so it is
  *worse* than #1 legally, not better (§2.1). Same mitigation applies, non-negotiably.
- **Loses `/battery`, `/beep`, `/servo_s1`, `/servo_s2`**; adds unused `/bumper_state`.
- Pins are the vendor's with M1/M2 swapped and H2 phase-inverted — **must be
  reconciled** against §4.1 or two wheels drive backwards.
- Pinned to ESP-IDF **v5.1.2**; expect friction on a modern host.
- `PWM_MOTOR_DEAD_ZONE` is still `200` — **does not fix the problem out of the box.**

### #3 — `linorobot2_hardware` (Apache-2.0)

<https://github.com/linorobot/linorobot2_hardware>

**The only option you could legally vendor into the public repo**, and the right answer
if publishable firmware is a hard requirement. Actively maintained, PlatformIO (much
easier than ESP-IDF), ESP32-S3 + `SKID_STEER` 4-motor + encoders + PID + odometry.

Ranked third because the rework is real, not cosmetic: **ICM42670P is unsupported and
needs a new IMU driver**; every pin collides with its defaults; topic names differ from
the host stack; and the four auxiliary topics must be written from scratch — and
cribbing them from Yahboom would forfeit the licence advantage that is this option's
entire reason for existing. **Effort: 3–5 days**, with the IMU driver the main unknown.

### #4 — `micro-ROS/micro_ros_espidf_component` (Apache-2.0)

Not an alternative — a **dependency** of #1 and #2. Listed only to be explicit that the
micro-ROS layer itself is cleanly licensed, actively maintained, and ESP32-S3-supported,
so no part of the licence problem lives here. Watch the ESP-IDF version skew (§3.2).

### Rejected

- **Reflashing the stock `.bin`** — reproduces the exact current behaviour (R4 §5).
  Its value is purely as a **recovery artefact**: download it and keep it.
- `kaiaai/firmware` — Apache-2.0 and well maintained, but **2-motor only**, 4WD is TODO.
- `camrbuss/diff-drive-esp32-uros`, `dabmake/ESPROS`, diffbot — wrong hardware or stale.
- `Reinbert/ros2_esp32` — **not assessed; not opened.** No opinion offered.

---

## 7. WHAT IS VERIFIED VS INFERRED

**Verified (I read the page or file listing):**
- Yahboom's written statement that firmware source is not provided
- Drive folder contents, sizes, file IDs, dates — both live folders
- That `1ZJ-egn8Co_nm32R35ND8S-OIpbhNaF60` (the brief's link, and the one in the repo's
  `AllFile_Download_Link.txt`) is **404 / dead**
- The full vendor GPIO map, from two independent Yahboom pages
- PrwTsrt's PWM and encoder `#define`s, and `PWM_MOTOR_DEAD_ZONE (200)`
- PrwTsrt has **no licence** (GitHub API 404)
- linorobot2_hardware is **Apache-2.0** (LICENSE file read) with ESP32-S3 + SKID_STEER
- micro_ros_espidf_component is Apache-2.0, ESP32-S3, actively maintained
- Sample paths `~/esp/Samples/esp32_samples/encoder`, `~/esp/Samples/microros_samples/imu_publisher`
- CP2102-GMR is the USB-UART chip; IMU is ICM42670P; motor is the "310"
- No board schematic is published in either Hardware Information folder
- Flash addresses and the BOOT+RESET fallback wording

**Inferred (reasoned, not directly observed):**
- `Samples.zip` contains the ESP-IDF sample source (*high* — corroborated by tutorial
  paths + PrwTsrt's directory structure + macro-name match)
- `Factory-Firmware.zip` is bin-only (*high* — size + vendor statement)
- `ROS_Source_Code.zip` is host-side ROS2, not ESP32 firmware (*medium*)
- DTR/RTS auto-reset is wired and works (*medium* — from conditional phrasing only)
- The documented pin map is V2.0-current (*high* — but no explicit V2.0 marking exists)
- Stock firmware shares `PWM_MOTOR_DEAD_ZONE 200` (*high*, inherited from R4; the
  common-ancestor finding in §2.1 strengthens it — PrwTsrt and the factory firmware are
  now known to descend from the same Yahboom sample code)

**Still unknown:**
- Actual `Samples.zip` contents (download and check — this is the one open item)
- Which physical wheel each motor channel drives, and each encoder's sign
- The rover's currently-flashed firmware version (read the 115200 boot banner)
- The correct replacement dead-zone value — must be measured on loaded motors, on blocks
- `MOTOR_WHEEL_CIRCLE 150.8` remains unverified (R4 §6.5); measure it

---

## 8. SOURCES

- <http://www.yahboom.net/public/upload/upload-html/1708423522/1.%20Write%20firmware.html>
- <https://www.yahboom.net/public/upload/upload-html/1737552446/How%20to%20Update%20the%20Firmware.html>
- <http://www.yahboom.net/public/upload/upload-html/1708480467/Introduction%20to%20microROS%20control%20board.html>
- <http://www.yahboom.net/public/upload/upload-html/1713429128/Brief%20introduction%20of%20microROS%20control%20board.html>
- <http://www.yahboom.net/public/upload/upload-html/1708418518/Read%20motor%20encoder%20data.html>
- <http://www.yahboom.net/public/upload/upload-html/1711444141/Read%20motor%20encoder%20data.html>
- <https://www.yahboom.net/public/upload/upload-html/1708420558/Release%20IMU%20data%20topic.html>
- <https://www.yahboom.net/public/upload/upload-html/1708570175/First%20Trial.html>
- <https://www.yahboom.net/public/upload/upload-html/1733396757/Set%20up%20ESP32-IDF%20development%20environment.html>
- <https://www.yahboom.net/study/MicroROS-Board>
- <https://www.yahboom.net/study/MicroROS-ESP32>
- <https://www.yahboom.net/study/MicroROS-Pi5>
- <https://drive.google.com/drive/folders/1Vp9qGsNR_A76mk58JC9xvXpZ0nmrffEy?usp=sharing> (Firmware And Code)
- <https://drive.google.com/drive/folders/1mDx8VEdl75yw8wrlYWQkmNEfY_TB5HVl?usp=sharing> (Code and Firmware, newer)
- <https://drive.google.com/drive/folders/1klzxonweuEEfFQa3jetL4P3L6vyZWkr4?usp=sharing> (Hardware Info — CP2102, 310 motor)
- <https://drive.google.com/drive/folders/1Akl4eNcx0iPCiOKzo19IAXzehRkr9kRv?usp=sharing> (Hardware Info, Pi5)
- <https://github.com/YahboomTechnology/MicroROS-Board>
- <https://github.com/YahboomTechnology/MicroROS-Board/blob/main/AllFile_Download_Link.txt> (points at a dead folder)
- <https://github.com/YahboomTechnology/Mirco-Ros-Car_VM>
- <https://github.com/YahboomTechnology/MicroROS-Car-Pi5>
- <https://github.com/PrwTsrt/microros_esp32_diffdrive>
- <https://github.com/PrwTsrt/microros_esp32_diffdrive/blob/main/components/pwm_motor/pwm_motor.h>
- <https://github.com/PrwTsrt/microros_esp32_diffdrive/blob/main/components/encoder/encoder.h>
- <https://github.com/linorobot/linorobot2_hardware>
- <https://github.com/linorobot/linorobot2_hardware/blob/master/LICENSE>
- <https://github.com/hippo5329/linorobot2_hardware>
- <https://github.com/hippo5329/linorobot2_hardware/wiki>
- <https://github.com/micro-ROS/micro_ros_espidf_component>
- <https://github.com/kaiaai/firmware>
- <https://github.com/camrbuss/diff-drive-esp32-uros>
- <https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html>
- <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/flashing-troubleshooting.html>
- <https://www.espressif.com.cn/en/support/download/other-tools>
