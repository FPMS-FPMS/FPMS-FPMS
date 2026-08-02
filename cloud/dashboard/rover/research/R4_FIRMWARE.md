# R4 — FIRMWARE INVESTIGATION

**Board:** Yahboom MicroROS Board V2.0 (ESP32-S3)
**Date:** 2026-08-02
**Status:** Report only. Nothing was flashed, no rover access, no code changed.

---

## 0. HEADLINE — the earlier research pass was backwards

The previous pass concluded the 50%-duty behaviour is *"the textbook signature of **missing** min-PWM / dead-zone mapping on the ESP32."*

**That is the opposite of what is happening.** The dead-zone mapping is present, is working exactly as written, and is the *cause* of the problem. It is not missing — it is grossly oversized.

I found the actual driver source and the arithmetic falls out exactly:

```c
// components/pwm_motor/pwm_motor.h
#define PWM_MOTOR_TIMER_RESOLUTION_HZ    10000000
#define PWM_MOTOR_FREQ_HZ                25000
#define PWM_MOTOR_DUTY_TICK_MAX          (PWM_MOTOR_TIMER_RESOLUTION_HZ / PWM_MOTOR_FREQ_HZ)
#define PWM_MOTOR_DEAD_ZONE              (200)
#define PWM_MOTOR_MAX_VALUE              (PWM_MOTOR_DUTY_TICK_MAX-PWM_MOTOR_DEAD_ZONE)
```

```c
// components/pwm_motor/pwm_motor.c
// Motor dead zone filtering
static int PwmMotor_Ignore_Dead_Zone(int speed)
{
    if (speed > 0) return speed + PWM_MOTOR_DEAD_ZONE;
    if (speed < 0) return speed - PWM_MOTOR_DEAD_ZONE;
    return 0;
}
```

Source: <https://github.com/PrwTsrt/microros_esp32_diffdrive/blob/main/components/pwm_motor/pwm_motor.h> and `.../pwm_motor.c`

Work it through:

| Quantity | Value |
|---|---|
| `PWM_MOTOR_DUTY_TICK_MAX` | 10 000 000 / 25 000 = **400 ticks** |
| `PWM_MOTOR_DEAD_ZONE` | **200 ticks = exactly 50.0% duty** |
| `PWM_MOTOR_MAX_VALUE` (PID output clamp) | 400 − 200 = **200** |
| Smallest possible non-zero output | (1 + 200)/400 = **50.25% duty** |
| Reachable duty band | **50.25% … 100%.** The bottom half of the actuator range does not exist. |

The dead zone is **added as a feed-forward offset** to the PID output. So the instant the PID asks for any non-zero effort — even one tick — the motor gets 50% duty. This is a legitimate technique (dead-zone compensation for stiction), implemented correctly, but with a constant roughly 4–10× larger than these motors actually need.

**This exactly reproduces every measurement on the rover:**

- *"roughly 50% motor duty on ANY non-zero `/cmd_vel` setpoint"* → the computed floor is 50.25%. Match.
- *"amplitude does nothing"* → true across the whole low range. A command asking for 2% effort and one asking for 45% effort both clamp up to ~50% duty. Amplitude only starts to matter above the half-way point.
- *"0.00295 m/s on the floor produced NO motion; 0.0010 free-spinning ran fast"* → see §1.
- *"shortest burst that moves anything is ~0.35 s"* → that is the stiction break-free time of a lurch, not a control parameter.
- *"a 300 mm segment crossed ~1 m"* and *"outran its own 11 Hz odometry"* → inevitable. Once it breaks free at 50% duty it is moving far faster than any 300 mm segment plan assumed.

The rover does not have a slow speed that is hard to reach. **It has no slow speed at all.** Minimum commandable motion is set by a hard-coded PWM floor, and `/cmd_vel` amplitude cannot reach below it.

### Why the two contradictory low-speed observations both happen

The command→encoder-target conversion is:

```c
// components/motor/motor.c
speed_count[i] = speed_m[i] / (MOTOR_WHEEL_CIRCLE/MOTOR_ENCODER_CIRCLE/MOTOR_PID_PERIOD);
```

With this rover's quoted constants (`MOTOR_ENCODER_CIRCLE 1040`, `MOTOR_WHEEL_CIRCLE 150.8` mm, `MOTOR_PID_PERIOD 10` ms):

- one encoder count = 150.8/1040 = **0.145 mm of wheel travel**
- 1.0 m/s → 10 mm per 10 ms tick → **69 counts per PID tick**
- **0.00295 m/s → 0.20 counts per PID tick**

The setpoint is *below the encoder resolution*. The PID sees an integer feedback of 0 or 1 and a target of 0.2. It is not controlling anything — it is dithering. Every time it asks for a hair of effort, the dead zone converts that into a 50% duty kick; the wheel lurches, overshoots by tens of counts, the PID slams to zero, the wheel coasts to a stop. That is a bang-bang limit cycle, not velocity control.

- **On the floor** (load + stiction, short burst): the lurch may never break free within the burst → *no motion*, as measured.
- **Free-spinning** (no load): the same 50% kick spins an unloaded wheel up hard and the PID cannot settle it → *runs fast*, as measured.

Both observations are the same defect seen under two load conditions. Nothing here indicates a hardware fault or a corrupted flash.

*(Confidence note: the 1040 / 150.8 constants are quoted from elsewhere in this project and I could not verify their source. However they are dimensionally self-consistent with the formula above and Yahboom's own encoder documentation independently derives 1040 — "13 lines × 20 reduction × 4 edge detection = 1040" — at <https://www.yahboom.net/public/upload/upload-html/1708418518/Read%20motor%20encoder%20data.html>. So 1040 is corroborated; 150.8 mm is not, but is plausible for a 48 mm wheel.)*

---

## 1. Is this expected behaviour, a fault, or a misconfiguration?

**Verdict: expected behaviour of the shipped design. Not a fault. A vendor misconfiguration for this motor/chassis combination.**

Three independent lines of evidence say the stock Yahboom firmware contains this same dead-zone code:

1. **Yahboom's own PID tutorial** references the identical symbol set — `PWM_MOTOR_MAX_VALUE` as the PID output limit, `MOTOR_PID_PERIOD`, `MOTOR_ENCODER_CIRCLE`, `MOTOR_WHEEL_CIRCLE` — and gives default gains `kp = 1.0`, `ki = 0.2`, `kd = 0.2`. Same names, same structure as the open-source tree.
   <http://www.yahboom.net/public/upload/upload-html/1708418593/PID%20controls%20car%20speed.html>

2. **Topic-name fingerprint.** The open-source tree publishes `odom_raw`, `imu`, subscribes `cmd_vel` — `odom_raw` is a distinctive non-standard name and it matches this rover exactly. The repo is explicitly a port of Yahboom's sample code for this board and retains its bilingual Chinese/English comments.

3. **Numerical match.** The predicted floor is 50.25% duty; the measured floor is "roughly 50%". That is not a coincidence.

I did **not** read Yahboom's stock firmware source directly (see §2 — I could not obtain it), so this is a strong inference, not a direct confirmation. But it is strong enough to act on, and it has a decisive consequence: **reflashing stock firmware will reproduce the exact behaviour you already have.**

### Is the dead zone configurable at runtime? No — on this board.

This matters, and the answer is board-specific:

- On the **microROS ESP32-S3 board**, `PWM_MOTOR_DEAD_ZONE` is a `#define`. It is baked in at compile time. There is no topic, no service, no parameter, and no serial command to change it. The board exposes only `/cmd_vel`, `/odom_raw`, `/imu`, `/battery`, `/beep`, `/servo_s1`, `/servo_s2` — which matches what you measured, and none of those touch it.
- On Yahboom's **other** driver boards (the USART/IIC expansion boards) the same vendor *does* expose it as a runtime call, `send_motor_deadzone(...)`, with documented per-motor-type values: **1900** (520 motor), **1600** (310 motor), **1250** (speed-code-disc TT), **1000** (TT DC geared).
  <https://www.yahboom.net/public/upload/upload-html/1742007185/Drive%20motor%20and%20read%20encoder-USART.html>

The second point is useful in two ways. It confirms Yahboom treats dead zone as a value that **must be tuned per motor type** — and it shows they know different motors need materially different values (a 1.9× spread across their own catalogue). The microROS board simply hard-codes one value and offers no way to change it.

### Is there a chassis / car-type setting?

I found **no** chassis-type or car-type selector exposed on the MicroROS Board V2.0 — no topic, no service, no menuconfig option surfaced in the docs. Chassis geometry is compile-time only (`ROBOT_WIDTH`, `ROBOT_LENGTH`, `ROBOT_APB`, `ROBOT_SPIN_SCALE` in `car_motion.h`). Yahboom's docs offer "Line speed calibration" and "Angular velocity calibration" lessons, but those calibrate the *host-side* scaling, not the firmware's PWM floor. They cannot fix this.

### Documented velocity range

Yahboom documents the accepted input range as **−1.0 to 1.0 m/s**, and the firmware clamps at `MOTOR_MAX_SPEED (1.5)`. **No minimum commandable speed is documented anywhere.** That omission is precisely the trap: the API advertises a continuous range down to zero, and the actuator silently quantises everything below ~50% duty into a lurch.
<http://www.yahboom.net/public/upload/upload-html/1708418593/PID%20controls%20car%20speed.html>, <http://www.yahboom.net/public/upload/upload-html/1706695326/Subscribe%20speed%20control%20topics.html>

---

## 2. Yahboom's official firmware — what is actually published

**Honest summary: I could not obtain a firmware binary, and I will not guess a URL.**

### What the GitHub repo contains

<https://github.com/YahboomTechnology/MicroROS-Board> holds documentation only:

```
00. ReadMe/
01. microROS control board development environment/
02. ESP32 basic course/
03. microros basic course/
04. ROS2 basic course/
05. Robot basic course/
AllFile_Download_Link.txt
MicroROS_Control_Board.jpg
README.md
```

No `.bin`. No firmware source. This confirms the previous pass's finding. The README states firmware is pre-installed before shipment and points at `AllFile_Download_Link.txt`, which contains a Google Drive link.

### The Google Drive link

The study page <https://www.yahboom.net/study/MicroROS-Board> lists a **"Firmware And Code"** Google Drive download alongside VM_File, 3D Model File, and ROS2_Basic_VM. The label implies source *and* binary are both there.

**I could not enumerate that Drive folder.** WebFetch cannot traverse Google Drive. So:

- I **cannot confirm** a `.bin` exists in it.
- I **cannot confirm** whether the "Code" half is ESP32 firmware source or only host-side ROS2 Python/C++ examples (Yahboom frequently ships the latter and calls it "code").
- I **cannot state a version number** for the stock firmware.

**Do not treat "Yahboom publishes the source" as established.** It is unverified. Someone with a browser should open that Drive folder and check — it is a two-minute task and it would materially change the options in §3.

### Flash procedure (this part *is* well documented)

Yahboom's own instructions, two variants:

**Single-file factory image:**
- Tool: **ESP32 Flash Download Tool** (Espressif), SPIDownload mode, from <https://www.espressif.com.cn/en/support/download/other-tools>
- File: `microROS_Robot_Vx.x.x.bin` at address **`0x0`**
- Check **`DoNotChgBin`**, select COM port, leave everything else default, press START
- Blue **FINISH** = done; power-cycle or press RESET

**Three-file image** (the form actually shown for microROS Robot):
| File | Address |
|---|---|
| `bootloader.bin` | `0x0000` |
| `partition-table.bin` | `0x8000` |
| `microROS_Robot.bin` | `0x10000` |

**If flashing does not auto-start:** *"press and hold the boot0 key, then press the reset key, release the boot0 key"* to enter download mode manually.

**Verify:** serial terminal at **115200 8N1**, press RESET, the board prints its firmware version.

Sources: <https://www.yahboom.net/public/upload/upload-html/1737552446/How%20to%20Update%20the%20Firmware.html>, <https://www.yahboom.net/public/upload/upload-html/1708480530/Flash-tool%20burning%20firmware.html>, <http://www.yahboom.net/public/upload/upload-html/1708423522/1.%20Write%20firmware.html>

Yahboom explicitly notes: *the board ships with factory firmware already burned, and if you have not flashed anything else you do not need to reflash it.* That applies to this rover.

---

## 3. The open-source replacement — honest assessment

**Repo:** <https://github.com/PrwTsrt/microros_esp32_diffdrive>
ESP-IDF source for a differential-drive robot on the Yahboom ESP32-S3 microROS board (author runs it on an Orange Pi 5B with ROS 2 Humble over USB/UART).

### Does it solve the slow-speed problem?

**No. Not as-is.** This is the most important finding in this section and it contradicts the framing in the brief.

`PWM_MOTOR_DEAD_ZONE` is **`200`** in this repo — the identical value causing the problem. Flash it unmodified and you get the same 50% floor. It inherits the defect because it inherits the vendor's code.

**What it gives you is not a fix — it is the *ability* to fix.** The constant becomes a one-line edit you own, instead of an opaque blob you cannot touch. That is genuinely valuable, but it is a different proposition from "this repo solves it."

### Correction to the brief

> *"runtime-tunable PID (`Motor_Update_PID_Parm`)"*

`Motor_Update_PID_Parm(float pid_p, float pid_i, float pid_d)` exists in `components/motor/motor.c` and updates all four PID controllers. But I checked `main/main.c` and **it is not wired to any ROS service, topic, or parameter.** It is an internal C function. It is *compile-time* tunable, not runtime tunable. Do not plan around being able to retune PID from the host.

The repo's ROS interface is: publishes `odom_raw`, `imu`, `bumper_state`; subscribes `cmd_vel`. No service is registered.

### What must change before it would work on this rover

| Constant | Repo value | This rover needs | Why |
|---|---|---|---|
| `PWM_MOTOR_DEAD_ZONE` | `200` (50% duty) | **~30–60** (7.5–15%), tuned empirically | **The actual fix.** Must be found by experiment: ramp duty up on a loaded wheel, find the tick count at which it just breaks free, use ~80% of that. |
| `MOTOR_ENCODER_CIRCLE` | `2244` | `1040` | Different motor/gearbox. Corroborated by Yahboom docs. |
| `MOTOR_WHEEL_CIRCLE` | `396.4` | `150.8` | Different wheel. **Unverified — measure it.** |
| `ROBOT_WIDTH` | `0.135f` | measure | Chassis geometry |
| `ROBOT_LENGTH` | `0.095f` | measure | Chassis geometry |
| `ROBOT_APB` | `0.121f` | derive | Kinematics constant |
| `ROBOT_SPIN_SCALE` | `5.0f` | verify | Turn-rate scaling |
| GPIO map | M1A=16, M1B=15, M2A=4, M2B=5, M3A=9, M3B=10, M4A=13, M4B=14 | **VERIFY against V2.0** | See risk below |

**Note the geometry ratio.** The repo's counts-per-metre is 2244/396.4 = 5.66/mm; this rover's is 1040/150.8 = 6.90/mm. So this rover has *finer* encoder resolution per mm than the repo's robot — which makes the oversized dead zone comparatively even worse here.

### Functional regressions you would accept

The repo does **not** implement `/battery`, `/beep`, `/servo_s1`, `/servo_s2`. All four exist on the rover today and would be **lost**. It adds `/bumper_state`, which the rover has no hardware for. Anything in the stack depending on `/battery` breaks.

### Real build + flash effort

Per the repo README:

```bash
# ESP-IDF v5.1.2 exactly
git clone -b v5.1.2 --recursive https://github.com/espressif/esp-idf.git
cd ~/esp/esp-idf && ./install.sh esp32s3 && source ./export.sh

# micro-ROS component, humble branch
git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git
pip3 install catkin_pkg lark-parser empy colcon-common-extensions

# EDIT colcon.meta: rmw_microxrcedds -> publishers/subscribers/history = 3,
# transport = "custom" (for USB/UART)

idf.py set-target esp32s3 && idf.py menuconfig   # micro-ROS Settings -> Micro XRCE-DDS over UART
idf.py build flash monitor
```

Honest estimate:

- **Toolchain + first successful build of the unmodified tree: half a day.** ESP-IDF v5.1.2 is pinned and old; expect friction on a modern host. The micro-ROS component build is slow (it compiles the whole middleware) and is the usual failure point. Do this on a Linux box, not Windows.
- **Constant edits: minutes.** Trivial once building.
- **Bring-up and dead-zone tuning on hardware: half to a full day**, most of it spent finding the true stiction threshold and re-validating odometry scaling.
- **Restoring `/battery` + `/beep` + servos: extra**, and only feasible if you can see Yahboom's source (§2) or reverse the peripherals.

**Realistic total: 1.5–3 days** for someone comfortable with ESP-IDF. Considerably more for someone who is not.

### Two non-technical blockers

1. **No `LICENSE` file.** I checked — `LICENSE` returns 404. The repo has no stated licence, so by default **all rights are reserved**. This project's repo is **PUBLIC**. Vendoring this code into it is a licensing problem. Ask the author, or keep the firmware tree out of the public repo.
2. **Pin map unverified for V2.0.** The GPIO assignments are for the author's board. If V2.0 differs, motors will not respond. Recoverable (reflash), not fatal — but budget for it.

---

## 4. Risk of reflashing

**Overall: LOW, and reversible — provided you take a backup first.** The ESP32-S3 is one of the harder microcontrollers to genuinely brick.

### Why it is hard to brick

- The **ROM bootloader lives in mask ROM**. It physically cannot be erased or overwritten by any flash operation. A bad application image cannot remove your ability to reflash.
- **BOOT (GPIO0) + RESET always forces download mode**, regardless of what is in flash — the same sequence Yahboom documents for normal flashing.
- The ESP32-S3 has **native USB-Serial-JTAG**, so recovery does not depend on an external USB-UART bridge surviving.

### What actually bricks an ESP32-S3

All of these are **eFuse** operations — one-time, irreversible, and **none of them are performed by `esptool write_flash` or the Flash Download Tool in normal use**:

- Burning the **Secure Boot** eFuse (then only signed images boot — an unsigned build will be rejected forever)
- Enabling **Flash Encryption** in release mode (plaintext images no longer boot; raw flash reads return garbage)
- Burning `DIS_DOWNLOAD_MODE` / disabling the USB-Serial-JTAG eFuse (removes the recovery path — this is the true brick)
- Physical damage: wrong voltage on IO, damaging the flash chip

**Rule: never run `espefuse.py burn_efuse` on this board.** That is the only category of command that is unrecoverable.

Non-brick failure modes — all recoverable by reflashing — include a wrong flash address, a wrong pin map (motors silent), a corrupted partition table, or a boot loop.

### What is lost

- **Everything currently in flash is overwritten** at the addresses you write. Without a backup, the factory firmware is gone.
- **NVS partition contents are lost** if you erase or overwrite that region — any stored calibration, WiFi credentials, or persisted settings. Unknown what this board keeps there; assume something.
- **The factory firmware is not recoverable from Yahboom with certainty** (§2 — I could not confirm a downloadable `.bin` exists). **This is the single biggest risk in the whole exercise.** If you flash without a backup and the Drive folder turns out to hold only host-side examples, the rover's original firmware is unrecoverable except by asking Yahboom support (support@yahboom.com).

### Backup — do this first, it is cheap

```bash
pip install esptool

# 1. Identify chip and, critically, the real flash size
esptool.py --port <PORT> flash_id

# 2. Confirm flash encryption / secure boot are NOT enabled.
#    If either is on, the dump will be encrypted or refused ("invalid head of packet").
esptool.py --port <PORT> get_security_info

# 3. Full dump. Adjust 0x800000 to the size reported in step 1 (8MB shown).
esptool.py --port <PORT> -b 460800 read_flash 0 0x800000 yahboom_v2_factory_backup.bin
```

Restore:

```bash
esptool.py --chip esp32s3 --port <PORT> --baud 921600 \
  --before default_reset --after hard_reset \
  write_flash -z --flash_mode keep --flash_freq keep --flash_size keep \
  0x0 yahboom_v2_factory_backup.bin
```

Notes:
- **Use 460800 baud, not 921600, for the read.** Higher rates are flaky and a corrupted backup is worse than none. Verify the dump's size and SHA-256 afterwards, and store it **outside the public repo** (it is a multi-MB binary and it is vendor firmware).
- If `get_security_info` shows encryption or secure boot enabled, **stop** — a raw dump will not be restorable, and the whole reflash plan needs rethinking.

Sources: <https://www.espboards.dev/blog/standalone-esptool-basics/>, <https://esp32-si4732.github.io/ats-mini/recovery.html>, <https://esp32s.com/blog/the-complete-and-definitive-guide-to-esp32-flash-memory-master-factory-reset-recovery-and-troubleshooting-with-esptool/>

---

## 5. RECOMMENDATION

### Reject: reflash stock firmware

**Do not do this.** It cannot help. The evidence in §1 says the stock firmware *is* the code containing `PWM_MOTOR_DEAD_ZONE 200`, so reflashing reproduces the exact behaviour you already have. There is also no evidence of corruption — the board enumerates, publishes all seven topics, and responds deterministically. It is behaving *correctly*, per a bad constant. Reflashing stock spends the risk and gains nothing.

### Adopt: a phased plan

**Phase 0 — Back up the flash. Do this before anything else, this week.**
Zero risk, ~10 minutes, and it is the precondition for every other option. Right now the rover is one careless flash away from an unrecoverable state, because no confirmed source of the factory `.bin` exists (§2). Run the three commands in §4. Store the dump off-repo.

While you are there, **open the "Firmware And Code" Google Drive folder** on <https://www.yahboom.net/study/MicroROS-Board> and record what is actually in it. If Yahboom's ESP32 firmware *source* is in there, that is strictly better than the third-party repo — same architecture, but with `/battery`, `/beep`, and the servos already implemented and the correct V2.0 pin map. **That single check could cut Phase 2 in half.** I could not do it; a human with a browser can.

**Phase 1 — Make the rover safe on current firmware, now.**
No flashing. Accept the hard truth: **the minimum controllable motion is fixed by a 50% duty floor, and 300 mm precision segments are not achievable on this firmware.** Stop issuing small `/cmd_vel` amplitudes — they do not mean what the API implies, and today's collision is the direct result of assuming they do. Interim measures:
- Treat motion as **quantised**: the atomic unit is a ~0.35 s burst, and that burst travels a large and only roughly predictable distance.
- **Do not rely on 11 Hz `/odom_raw` as a safety guard.** It has already been outrun. Add a wall-clock burst-duration cap enforced independently of odometry, sized well below the distance that caused the collision.
- Use pulse-and-coast: burst, stop, let odometry settle, integrate, decide. Slow and coarse, but bounded.

This is a workaround, not a fix. Its ceiling is low and it should not be mistaken for a solution.

**Phase 2 — Build custom firmware. This is the only real fix.**
Because the dead zone is a compile-time constant with no runtime override on this board (§1), **owning the build is the only way to change it.** There is no configuration path, no calibration lesson, and no topic that can reach it.

Base it on Yahboom's own source if Phase 0 finds it; otherwise on `PrwTsrt/microros_esp32_diffdrive` — with clear eyes that the repo **does not fix the problem out of the box** (its `PWM_MOTOR_DEAD_ZONE` is also `200`) and **has no licence**, which matters for a public repo. The change that actually matters is one constant; everything else in the table in §3 is bookkeeping.

- **Effort:** 1.5–3 days, mostly toolchain setup and on-hardware dead-zone tuning.
- **Risk:** low and reversible with the Phase 0 backup in hand. Worst realistic outcome is silent motors from a wrong pin map — fixed by reflashing.
- **Payoff:** restores the bottom ~40% of the actuator range and makes small `/cmd_vel` values mean something. Nothing else on the table does this.
- **Watch:** you lose `/battery`, `/beep`, `/servo_s1`, `/servo_s2` unless you port them.

### What the operator must physically do

For **backup** (Phase 0) and **flashing** (Phase 2):

1. Connect the ESP32-S3 to a computer by **USB**. Prefer a Linux host for the ESP-IDF build.
2. Note the serial port (`/dev/ttyUSB0` or `/dev/ttyACM0` on Linux, `COMx` on Windows).
3. **If the tool does not connect or flashing does not auto-start:** hold **BOOT (boot0)**, tap **RESET**, then release **BOOT**. This forces download mode and is Yahboom's own documented recovery step. It always works — the ROM bootloader cannot be erased.
4. After flashing, **power-cycle or press RESET**.
5. **Verify** on a serial terminal at **115200 8N1** — press RESET and confirm the board prints its version banner.
6. **Safety, non-negotiable:** put the rover **on blocks with the wheels clear of the ground** for all firmware work and all dead-zone tuning. Given that a 300 mm command already produced a ~1 m run and a collision, no motor testing should happen with the rover on the floor until the new dead-zone value is confirmed.
7. **Never run `espefuse.py burn_efuse`.** That is the one command that can permanently brick the board.

---

## 6. Evidence gaps — stated plainly

Things I could **not** verify, and which should not be reported as fact:

1. **No Yahboom firmware `.bin` URL was found or confirmed.** The Google Drive folder is labelled "Firmware And Code" but I could not open it. I have not invented a link and none should be quoted.
2. **The stock firmware version number is unknown.** (`microROS_Robot_Vx.x.x.bin` is a filename *pattern* from the docs, not an observed release.)
3. **Whether Yahboom publishes ESP32 firmware source is unresolved.** The GitHub repo is documentation-only; the Drive folder is unexamined.
4. **I did not read the stock firmware's source.** The conclusion that it shares `PWM_MOTOR_DEAD_ZONE 200` is a strong multi-line inference (§1), corroborated by an exact numerical match to the measured 50% duty — but it is inference, not direct observation.
5. **`MOTOR_WHEEL_CIRCLE 150.8` is unverified.** `MOTOR_ENCODER_CIRCLE 1040` is independently corroborated by Yahboom's docs; the wheel circumference is not. Measure it.
6. **The PrwTsrt GPIO pin map is unverified against Board V2.0.**
7. **The correct replacement dead-zone value is unknown** and cannot be derived from documents — it must be measured on this specific motor under load.

---

### Source list

- <https://github.com/YahboomTechnology/MicroROS-Board>
- <https://www.yahboom.net/study/MicroROS-Board>
- <https://www.yahboom.net/study/MicroROS-ESP32>
- <http://www.yahboom.net/public/upload/upload-html/1708418593/PID%20controls%20car%20speed.html>
- <https://www.yahboom.net/public/upload/upload-html/1708418518/Read%20motor%20encoder%20data.html>
- <http://www.yahboom.net/public/upload/upload-html/1706695326/Subscribe%20speed%20control%20topics.html>
- <https://www.yahboom.net/public/upload/upload-html/1742007185/Drive%20motor%20and%20read%20encoder-USART.html>
- <https://www.yahboom.net/public/upload/upload-html/1737552446/How%20to%20Update%20the%20Firmware.html>
- <https://www.yahboom.net/public/upload/upload-html/1708480530/Flash-tool%20burning%20firmware.html>
- <http://www.yahboom.net/public/upload/upload-html/1708423522/1.%20Write%20firmware.html>
- <https://github.com/PrwTsrt/microros_esp32_diffdrive>
- <https://github.com/PrwTsrt/microros_esp32_diffdrive/blob/main/components/pwm_motor/pwm_motor.h>
- <https://github.com/PrwTsrt/microros_esp32_diffdrive/blob/main/components/pwm_motor/pwm_motor.c>
- <https://github.com/PrwTsrt/microros_esp32_diffdrive/blob/main/components/motor/motor.h>
- <https://github.com/PrwTsrt/microros_esp32_diffdrive/blob/main/components/motor/motor.c>
- <https://github.com/PrwTsrt/microros_esp32_diffdrive/blob/main/components/car_motion/car_motion.h>
- <https://github.com/micro-ROS/micro_ros_espidf_component>
- <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/index.html>
- <https://www.espressif.com.cn/en/support/download/other-tools>
- <https://www.espboards.dev/blog/standalone-esptool-basics/>
- <https://esp32-si4732.github.io/ats-mini/recovery.html>
- <https://esp32s.com/blog/the-complete-and-definitive-guide-to-esp32-flash-memory-master-factory-reset-recovery-and-troubleshooting-with-esptool/>
- <https://github.com/YahboomTechnology/Mirco-Ros-Car_VM>
