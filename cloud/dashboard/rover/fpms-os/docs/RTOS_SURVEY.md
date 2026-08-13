# RTOS survey: what of this can actually run on the FPMS rover

Five names were put on the table — Wind River VxWorks, QNX Neutrino, FreeRTOS,
Zephyr and NuttX. This document says what is really behind each name, what it
would cost to adopt, and which one is already running on this rover without
anyone having decided to run it.

It is a **research document**. Nothing in the repo was changed to write it and
no image was built.

**The recommendation is: change nothing in the firmware.** The reasoning is in
§6 and §7, and the survey exists partly to record why the obvious-looking
change is not worth making.

## How to read the evidence tags

Every load-bearing claim carries one:

- **MEASURED** — a number this project took off its own hardware.
- **VERIFIED-FROM-SOURCE** — read out of source in this repo, or out of the
  upstream source/devicetree/Kconfig of the project being described.
- **FROM-DOCS** — vendor or project documentation, not source.
- **UNVERIFIED** — believed, not established. Treat as a lead, not a fact.
- **UNRESOLVED** — actively attempted and not completed. §4 is this.

---

## 0. The short answer

The rover's real-time question is on the ESP32-S3, not the Pi. A Linux box
publishing setpoints at 25 Hz does not need hard real-time.

**FreeRTOS is already running on the ESP32-S3.** It is not a candidate; it is
the incumbent. `loop()` is a FreeRTOS task called `loopTask`, pinned to core 1,
priority 1, on a mandatory 1000 Hz tick. The firmware calls no FreeRTOS API at
all. So "adopt FreeRTOS" is not an available action — it happened the moment
somebody wrote `framework = arduino`.

**The motor deadman is already deterministic enough, and it is already safe.**
`applyMotors()` runs unconditionally at the top of every `loop()` iteration,
outside the executor, and `AGENT_DISCONNECTED` calls `fullStop()` the instant
the link drops. What the cooperative superloop costs is *timing regularity*, not
safety — and on inspection the two places that regularity could have mattered
turn out to be insulated by design decisions already taken on the Pi side.

**Zephyr's ESP32-S3 support is real and better than expected**; its micro-ROS
story is not. **NuttX is UNRESOLVED** (§4) and loses on porting cost regardless.
**Nothing belongs on the RK3588S** — Zephyr can boot there and offers a serial
console in exchange for rknpu, Mali and bcmdhd.

---

## 1. `wind-river` and `qnx` — the GitHub orgs are not the OS

The suspicion in the question was right, and it is right for both.

### 1.1 Wind River

There are two orgs, not one.

- **`Wind-River`** — ~20 public repos (VERIFIED-FROM-SOURCE, org repo listing):
  - Yocto/BitBake layers for **Wind River Linux**, not VxWorks:
    `meta-secure-core`, `meta-wr-sbom`, `meta-lat`.
  - **VxWorks 7 Source Build (VSB) layers** — build glue that compiles a
    third-party library *into* a VxWorks you already own:
    `vxworks7-layer-for-bzip2`, `vxworks-layer-for-opencv`,
    `vxworks7-layer-for-ros2`, `vxworks7-google-test`,
    `vxworks7-layer-for-aws-iot-device-sdk`.
  - **ROS 2 on VxWorks**: `vxworks7-ros2-build` (Apache-2.0, 118 stars, pushed
    2026-07-28) and `vxworks7-layer-for-ros2` — the genuinely interesting pair,
    and actively maintained.
  - Tooling/forks: `crypto-detector`, `qemu`, `rust`, `wasm-micro-runtime`,
    `wr-conductor-*`, `cloud-platform-deployment-manager` (StarlingX).
  - **`vxworks7-BSPs`** — an *index* of BSP repositories, pointing at Raspberry
    Pi and TI Sitara AM65x. **No Rockchip RK3588/RK3588S. No Espressif
    anything.** (VERIFIED-FROM-SOURCE)
- **`wind-river-vxworks7`** — 9 small sample apps (Galileo GPIO/I2C/PWM/MRAA
  demos). Sample code, nothing more. (FROM-DOCS)

**There is no VxWorks kernel source in either org** (VERIFIED-FROM-SOURCE, by
absence across both org listings; corroborated by the VSB layers, every one of
which is written on the premise that you install VxWorks 7 separately and these
build *against* it). VxWorks 7 source ships under a Wind River source licence.

**What is genuinely usable:** Wind River Labs publishes a free **VxWorks SDK**
under a non-commercial licence, with BSPs for Raspberry Pi 3B/3B+/4B and UP
Squared (FROM-DOCS, `labs.windriver.com/downloads/wrsdk_labs.html`). A real free
tier and a real reference BSP — for the wrong boards.

### 1.2 QNX

- **`qnx`** — codelabs, workshops, `python3-qnx-apis`, a ROS 2 gamepad/arm demo,
  `mcap`, and the useful part: **Apache-2.0 board support packages**
  `bsp_raspberrypi-bcm2712-rpi5` and `bsp_raspberrypi-bcm2711-rpi4`.
  (VERIFIED-FROM-SOURCE, org repo listing.)
- **`qnx-ports`** — 20+ repos, all *third-party open source ported to QNX*:
  `grpc`, `llama.cpp`, `whisper.cpp`, `busybox`, `htop`, `dnsmasq`, `openrc`,
  `strongswan`, `ntpsec`, `executorch`, `MNN`, plus `build-files` (the port
  instructions) and `libsysv-ipc-shim`. (VERIFIED-FROM-SOURCE.)
- Neither contains `procnto` or any microkernel source.

**The history matters, because it is easy to find a 2007 press release and draw
the wrong conclusion.** QNX did publish Neutrino microkernel source in September
2007 under a hybrid licence (FROM-DOCS). That access was **withdrawn in April
2010**, on the day RIM announced the acquisition — Foundry27 SVN checkout was cut
off for myQNX accounts and the microkernel, libraries and Photon went back behind
the licence (FROM-DOCS, contemporaneous reporting). It has not come back.

**What is genuinely usable:** **QNX Everywhere** — free non-commercial access to
**QNX SDP 8.0** and **QNX Hypervisor 8.0**, launched at CES 2025, with ready-made
images for Raspberry Pi 4 and 5 (FROM-DOCS, `qnx.com/products/everywhere/`). A
genuinely good free tier: binaries, tooling, docs, training, Apache-2.0 BSP
source for the Pi boards. Not kernel source, and not an RK3588S or ESP32-S3
target.

### 1.3 Verdict on both

Neither is usable here, and not primarily because of licensing — because of
**silicon**. Neither has a BSP for RK3588S, and neither runs on an Xtensa LX7
microcontroller at all. The one thing worth filing away for a future
non-competition project is that both vendors now have real free non-commercial
tiers with Raspberry Pi images, which was not true a few years ago.

---

## 2. FreeRTOS — confirmed: already running, and the firmware ignores it

### 2.1 The chain, end to end

```
firmware_v3/fpms_platformio_env.ini:17    framework = arduino
                                              |
                                          arduino-esp32
                                              |  (is an ESP-IDF component)
                                          ESP-IDF
                                              |  vendors the kernel in-tree at
                                          components/freertos/FreeRTOS-Kernel/
```

- `framework = arduino` (VERIFIED-FROM-SOURCE, `fpms_platformio_env.ini:17`).
- ESP-IDF vendors **`components/freertos/FreeRTOS-Kernel/`** and
  **`components/freertos/FreeRTOS-Kernel-SMP/`** directly in-tree — a vendored
  fork, not a git submodule, which is why searching `.gitmodules` finds nothing
  (VERIFIED-FROM-SOURCE, `espressif/esp-idf` contents API).
- Espressif states it plainly: *"IDF FreeRTOS source code is based on Vanilla
  FreeRTOS v10.5.1 but contains significant modifications to both kernel
  behavior and API in order to support dual-core SMP"* (FROM-DOCS, ESP-IDF
  ESP32-S3 `freertos_idf` page).
- `cores/esp32/esp32-hal.h` includes `freertos/FreeRTOS.h`, `task.h`, `queue.h`,
  `semphr.h`, `event_groups.h` unconditionally (VERIFIED-FROM-SOURCE).

### 2.2 The actual task and core arrangement

| Thing | Value | Evidence |
|---|---|---|
| Task name | `loopTask` | VERIFIED-FROM-SOURCE, `arduino-esp32/cores/esp32/main.cpp` |
| Created by | `xTaskCreateUniversal(loopTask, "loopTask", …, 1, &loopTaskHandle, ARDUINO_RUNNING_CORE)` | VERIFIED-FROM-SOURCE |
| Priority | **1** (one above idle) | VERIFIED-FROM-SOURCE |
| Stack | 8192 B (`ARDUINO_LOOP_STACK_SIZE`) | VERIFIED-FROM-SOURCE, `Kconfig.projbuild` |
| Core pinning | `ARDUINO_RUNNING_CORE` → `CONFIG_ARDUINO_RUNNING_CORE`, default **0 if unicore, 1 if dual-core**. ESP32-S3 is dual-core ⇒ **core 1 (APP_CPU)** | VERIFIED-FROM-SOURCE, `Kconfig.projbuild` + `esp32-hal.h` |
| Tick rate | **1000 Hz, mandatory.** arduino-esp32 hard-fails the build otherwise: `FATAL_ERROR "esp32-arduino requires CONFIG_FREERTOS_HZ=1000 (currently ${CONFIG_FREERTOS_HZ})"` | VERIFIED-FROM-SOURCE, arduino-esp32 `components/CMakeLists.txt` |
| `delay(ms)` | `vTaskDelay(ms / portTICK_PERIOD_MS)` — an RTOS block, not a spin | VERIFIED-FROM-SOURCE, `esp32-hal-misc.c` |
| `yield()` | `vPortYield()` | VERIFIED-FROM-SOURCE, same |

**Verdict: CONFIRMED.** FreeRTOS is running on this rover right now. The
`FreeRTOS/FreeRTOS-Kernel` repository is MIT-licensed kernel source
(`list.c`/`queue.c`/`tasks.c` + `portable/`) and it is upstream of the fork we
already execute.

Even micro-ROS agrees: its RTOS comparison notes that the ESP32 toolchain
"integrates FreeRTOS as framework and allows the user to run micro-ROS tasks
simultaneously along with other user process" (FROM-DOCS, micro.ros.org).

Grepping `fpms_main.cpp` and `fpms_config.h` for `FreeRTOS`, `freertos`,
`xTask`, `portMUX`, `taskENTER` returns **zero hits** (VERIFIED-FROM-SOURCE).
We have a preemptive dual-core scheduler with a 1 ms tick and we run a
cooperative superloop on top of it.

### 2.3 The safety case is already handled — do not re-open it

This needs saying explicitly, because it is the trap in this question.

`loop()` (`fpms_main.cpp:1036`) calls **`applyMotors()` unconditionally on line
1045, at the very top, before the micro-ROS state machine** (VERIFIED-FROM-SOURCE).
`applyMotors()` (`:315`) is where the estop latch (`:330`) and the deadman
(`:341`, against `FPMS_CMD_TIMEOUT_MS` = 300 ms, `fpms_config.h:56`) live. It is
deliberately outside the executor path, and the comment at `:1038-1044` records
the bug that put it there: *"the control timer only fires while an agent is
connected and the executor is spinning; if the agent dies mid-burst the timer
stops and, without this line, the last duty would remain latched on the pins
indefinitely. An earlier build had exactly that bug and the rover kept driving
after the host was shut down."*

`AGENT_DISCONNECTED` (`:1073`) additionally calls `fullStop()` the instant the
link drops, before destroying entities.

That design is correct. It was written by someone who had already been bitten by
the exact failure a naive reading of "cooperative superloop" would worry about.
**Nothing in this document proposes moving it.**

### 2.4 What the superloop actually costs: regularity, and how much

Two calls can make an iteration long, both in `AGENT_CONNECTED`:

1. **`rmw_uros_ping_agent(100, 1)`** every 200 ms (`:1067-1068`). The first
   argument is a **100 ms timeout**.
2. **`rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20))`** (`:1070`), a 20 ms
   budget. The 50 Hz control timer (`fpms_config.h:104`,
   `rclc_timer_init_default` at `:869`) fires inside it, and so does all
   telemetry via `EXECUTE_EVERY_N_MS` (`:763-788`).

The transport underneath both genuinely blocks (VERIFIED-FROM-SOURCE,
`micro_ros_platformio/platform_code/arduino/serial/micro_ros_transport.cpp`):

```c
size_t platformio_transport_read(struct uxrCustomTransport * transport,
                                 uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
  Stream * stream = (Stream *) transport->args;
  stream->setTimeout(timeout);
  return stream->readBytes((char *)buf, len);
}
```

`Stream::readBytes()` polls `available()` until `len` bytes arrive or the timeout
expires, and yields to nothing, because there is no higher-priority task to yield
to. Whether `rmw_uros_ping_agent` returns early on a successful reply rather than
waiting out its timeout is **UNVERIFIED** — I did not read its implementation.
The 100 ms is a ceiling, not a cost.

**Confirming the arithmetic that was put to me.** The claim was that a stalled
ping drops deadman evaluation to ~10 Hz, and that against a 300 ms timeout this
degrades granularity but not the guarantee.

**Confirmed, and it is milder than that**, for a reason worth recording:

- A stalled ping costs one iteration ≈ 100 ms (ping) + 20 ms (spin) ≈ **120 ms**.
  That is the worst single gap between two `applyMotors()` calls — an
  instantaneous evaluation rate around 8 Hz for that one interval.
- It does **not** sustain. The state machine is
  `state = (RMW_RET_OK == rmw_uros_ping_agent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED`
  (`:1068`) — a **single** failed ping flips to `AGENT_DISCONNECTED`, which calls
  `fullStop()` (`:1074`). A genuinely stalled link therefore produces one 120 ms
  iteration and then stops the motors outright. The pathological case is
  self-limiting.
- Ping only runs once per 200 ms, so no two consecutive iterations can both
  carry it. Sustained rate stays well above 10 Hz.
- Worst-case latency from a command going stale to motors zeroing is therefore
  ≈ 300 ms + 120 ms ≈ **420 ms**, not 300 ms. Granularity, not guarantee.

**How much jitter is there really, on the 50 Hz control tick and the 25 Hz
`FPMS_TICKS_HZ` publication? UNVERIFIED.** I have not measured it and I am not
going to estimate it as though I had. The bound above is derived from source
constants; the *distribution* — how often a ping actually approaches its ceiling
on a healthy 230400-baud link — is unknown.

**And it is already instrumented.** `/fpms_health[6]` carries `g_control_hz_x10`
(`fpms_main.cpp:740`, computed at `:670` over a ≥250 ms window), `[3]`/`[4]`
carry `since_duty`/`since_vel`. Caveat that matters: a ≥250 ms averaging window
barely moves when one 120 ms stall lands inside it, so a healthy mean does not
disprove a tail. To see the tail you would want the *minimum* observed rate, not
the mean — which the current health field does not expose.

---

## 3. Zephyr — the ESP32-S3 hardware layer is real; micro-ROS is the weak link

This came out stronger than expected. Zephyr's ESP32-S3 support is not a toy.

**Board support: EXISTS, mature.** `boards/espressif/esp32s3_devkitc` with two
SoC targets, `…/procpu` and `…/appcpu` — real AMP, with a shared-memory `shm0`
region, an enabled `ipm0` node and an AMP partition table
(`partitions_0x0_amp.dtsi`). (VERIFIED-FROM-SOURCE.) The HAL is an out-of-tree
west module `hal_espressif` pinned by commit in `west.yml`, and RF **binary blobs
are required** (`west blobs fetch hal_espressif`). Current release line is Zephyr
4.4.0 (April 2026). The widely-cited Espressif blog post saying "use ESP32/S2
until multicore lands" is from **May 2022** and is superseded.

Per-item porting cost for what this firmware actually does:

| Subsystem | Zephyr status | Cost to move |
|---|---|---|
| **4-motor PWM** | `drivers/pwm/pwm_led_esp32.c` (LEDC) + `pwm_mc_esp32.c` (MCPWM). Binding `espressif,esp32-ledc`: S3 has **8 channels, 4 timers, no high-speed mode**. Resolution is *derived* (`log2(CLK/FREQ)`, capped at S3's 14-bit timer width), so 20 kHz off 80 MHz APB gives ~11 bits — 10-bit duty is comfortably available. Channels sharing a timer must share a frequency; four motors at 20 kHz is fine. (VERIFIED-FROM-SOURCE) | **Moderate.** The catch is our topology: `USE_BTS7960_MOTOR_DRIVER` (`fpms_config.h:123`) drives each motor from **two complementary PWM pins** (`MOTORn_IN_A`/`IN_B`, `MOTORn_PWM == -1`). Four motors × 2 pins = **all 8 of the S3's LEDC channels, zero spare**. And `spin(0)` must land in `brake()` which writes **both pins low = COAST**, not active brake — phase6's entire turn calibration, including the 0.93 turn-coast factor, was tuned against coast (`fpms_config.h:125-129`). Reproduce it exactly or every motion constant is invalid. |
| **4 quadrature encoders** | `drivers/sensor/espressif/pcnt_esp32/` (`espressif,esp32-pcnt`), exposed through the **sensor** API (`SENSOR_CHAN_ENCODER_COUNT`/`SENSOR_CHAN_ROTATION`), *not* Zephyr's QDEC API — there is no `qdec_esp32.c`. Quadrature via `sig-pos-mode`/`ctrl-h-mode`; per-unit glitch filter; software accumulator across the 16-bit wrap. S3 has **4 units — exactly our count, zero spare**. S3-tested by `samples/boards/espressif/qdec_trigger`, which ships an `esp32s3_devkitc_procpu.overlay`. (VERIFIED-FROM-SOURCE) | **High, and dangerous.** Not because the driver is missing but because of decoding mode. This firmware uses `ESP32Encoder::attachHalfQuad()` — **x2 decoding**, and the whole distance calibration was MEASURED through it (`fpms_config.h:180-186`: *"If anyone ever switches to `attachFullQuad()`, this constant must DOUBLE … in the same commit"*). A Zephyr PCNT overlay configured the natural way is x4. That is a silent 2× error in every distance, in a project whose memory already records a 2.7× counts/mm error as the cause of every distance overshoot. |
| **ICM42670P IMU** | **EXISTS in-tree, for the -P part specifically**: `drivers/sensor/tdk/icm42x70/` with `DT_DRV_COMPAT invensense_icm42670p`, bindings `invensense,icm42670p-i2c.yaml` and `-spi.yaml`, covering ICM-42670-P/-S and ICM-42370-P. Requires the `hal_tdk` west module. (VERIFIED-FROM-SOURCE) | **Low on paper, real risk in practice.** The one place Zephyr clearly beats us — a maintained vendor driver instead of ours. But ours exists because of a part-specific defect: on this part `GYRO_DATA_X1` is `0x11` and is **not contiguous** with the accel block at `0x0B`, and reading gyro as bytes 6-11 of a 12-byte accel burst silently returns zeros — the bug that made a real 180° turn integrate to 0.0° (`fpms_config.h:415-425`). Swapping mid-competition trades a fixed bug for an unknown one, the same reasoning already recorded for rejecting the TDK Arduino library. |
| **micro-ROS** | **PARTIAL / stale.** `micro_ros_zephyr_module` has a `humble` branch, but the README says tested only on **Zephyr 4.0.0 and 4.1.0**, with one example board (`disco_l475_iot1`). Open issue **#158** — broken on Zephyr 4.2+ because POSIX headers moved — created 2025-12-14, **still open**. Open issue **#131**, "Support for chip from espressif family with Zephyr" (Nov 2023, last touched Jul 2025): the sample overlay hardcodes a `usbotg_fs` label that doesn't exist on ESP32 DTs, so the build fails at devicetree stage. Last commit 2025-12-15. **No evidence of ESP32-S3 + Zephyr + micro-ROS working upstream at all.** (VERIFIED-FROM-SOURCE) | **This is the blocker.** The module trails the current Zephyr tree by ~3 releases and has an open ESP32 devicetree bug over two years old. The proven ESP32-S3 micro-ROS path today is the ESP-IDF component — i.e. FreeRTOS. |

**Zephyr summary:** the hardware layer is genuinely there. The transport we
depend on is not.

---

## 4. NuttX — **UNRESOLVED**

The NuttX research did not complete, and I am not going to invent it.

What was established:

- micro-ROS lists **NuttX as one of its three officially supported RTOSes**
  alongside FreeRTOS and Zephyr, downloadable natively through the micro-ROS
  build system (FROM-DOCS, micro.ros.org RTOS comparison). Its pitch there is
  POSIX compliance and small footprint.
- The proven ESP32-S3 micro-ROS path today is `micro_ros_espidf_component`
  (FreeRTOS), not NuttX (FROM-DOCS, same source).

What was **not** established, and must be checked before NuttX carries weight:

1. ESP32-S3 board maturity in `boards/xtensa/esp32s3/` and which peripherals are
   supported. **UNRESOLVED.**
2. Whether `esp32s3_ledc.c` exists and can drive **8 channels** at 20 kHz with
   10-bit resolution in the complementary pairs this chassis needs.
   **UNRESOLVED.**
3. Whether an ESP32-S3 PCNT/`qencoder` driver is upstream, how many units it
   exposes, and **whether it decodes x2 or x4**. **UNRESOLVED.** Same
   silent-2×-error trap as §3, and the question that decides NuttX's cost.
4. Whether NuttX has an in-tree **ICM-42670-P** driver, as opposed to
   ICM-20689/ICM-42688. **UNRESOLVED.**
5. `micro_ros_nuttx` maintenance status and Humble support. **UNRESOLVED.**
6. RK3588/RK3588S in `arch/arm64` / `boards/arm64`. **UNRESOLVED.**

**UNVERIFIED:** Espressif co-maintains NuttX upstream, so ESP32-S3 support is
generally believed to be good. That belief is not evidence, and it does not touch
items 3 and 4, which is where the cost lives.

**This gap does not change the recommendation.** Since the recommendation is "no
firmware change" (§7), a perfect NuttX result would still lose — it would have to
beat *doing nothing*, while costing a full rewrite.

---

## 5. Is any of this worth putting on the RK3588S? No.

**What FPMS-OS depends on that lives only in the Rockchip BSP kernel:** `rknpu`
(no driver → no YOLO → the failure `docs/NPU.md` exists to document), the Mali
G610 stack, and `bcmdhd` (the WiFi chip). `docs/ARCHITECTURE.md` names it in the
boot chain: *"u-boot ─ Rockchip BSP kernel 5.10/6.1 (rknpu, Mali, bcmdhd)"*.

**Zephyr on RK3588: EXISTS, and it is a bring-up skeleton.** There is no
`boards/rockchip`, but `soc/rockchip/rk35/` declares **`rk3568`, `rk3588` and
`rk3588s`**, and two boards exist under vendor directories:
`firefly/roc_rk3588_pc` (targets `/rk3588` and `/rk3588/smp`) and
`xunlong/orangepi_5_ultra_rk3588`. (VERIFIED-FROM-SOURCE.) The supported-features
table for both is, in full: **CPU, GICv3, PSCI 0.2, one ns16550 UART, on-chip
SRAM, ARM architected timer.** No GPIO, I2C, SPI, MMC, USB, Ethernet, PCIe, GPU,
NPU or WiFi. The image is tftp'd from U-Boot and run out of RAM, and the Firefly
board doc is marked **"Not actively maintained"** (FROM-DOCS). `rk3588.dtsi`
describes 8 CPUs, a GIC, two UARTs (both `status = "disabled"`) and PSCI, and
nothing else (VERIFIED-FROM-SOURCE).

The trade is: give up rknpu, Mali, bcmdhd, ROS 2 Humble, Nav2, slam_toolbox and
rosbridge — and receive a serial console.

**NuttX on RK3588: UNRESOLVED** (§4). Given Zephyr's state the prior is low, but
it was not checked. **VxWorks/QNX on RK3588S:** no BSP in either org (§1).

**Hypervisor / co-kernel: not a fantasy in general, a fantasy at this scale.**

- **Jailhouse** on RK3588 is real — 2025 published work (MemGuard/MemPol on ARM
  DynamIQ) launched a Jailhouse VM on a dedicated RK3588 Cortex-A55 core
  (FROM-DOCS). It is a static partitioning hypervisor that needs Linux to boot
  first and then takes cores away from it.
- **Xenomai 4 / EVL** on RK3588 is real — dovetail supports 6.1 on arm64, with
  published RK3588 write-ups (FROM-DOCS). Every one of them says the same thing:
  because the RK3588 BSP is not upstreamed, you must apply dovetail **into the
  vendor SDK kernel by hand and resolve the conflicts yourself**. That means
  hand-patching the exact kernel that carries rknpu, Mali and bcmdhd, and hoping
  all three still build and load.
- **The RK3588's own real-time cores are not available to us.** The SoC
  integrates three Cortex-M0 MCUs — in VD_PMU (16 KB cache/16 KB TCM), VD_NPU
  (16 KB/64 KB) and PD_CENTER (32 KB TCM) (FROM-DOCS, RK3588 datasheet). They run
  Rockchip's own power-management and NPU firmware. Taking the PMU or NPU MCU
  means breaking power sequencing or the NPU — the two things we least want to
  break — and 32 KB of TCM is not where a motor controller goes when there is
  already an ESP32-S3 on the bus.

**Nothing goes on the RK3588S.**

---

## 6. Does ESP32 timing jitter actually degrade the motion controller?

This was put to me as the strongest argument for a firmware change: the
continuous controller assumes a fixed control interval and derives its damping
from an actuator lag `tau`, so jitter corrupts the damping term and the
measured-rate cut band that removed the 3.9° turn bias.

I went looking to confirm it. **I mostly cannot**, and the reasons are specific
enough to be worth recording, because they are properties the controller was
deliberately given.

### 6.1 The turn controller does not use a finite difference at all

This is the decisive finding. §8.2.1's win — worst turn error 4.66° → 0.62° — is
in `TurnController`, and its measured-rate input is
`v_meas=abs(float(omega_meas_radps))` (VERIFIED-FROM-SOURCE, `fpms_motion.py:2052-2053`),
which is the **gyro rate**. The damping trim on the line above uses the same
quantity: `w_des = ref.v + self.kp*err - self.kd*abs(omega_meas_radps)` (`:2044-2045`).

A gyro rate is an **instantaneous measurement**. Its *value* does not depend on
how regularly samples arrive. Irregular arrival makes the sample older, not
wrong. So the mechanism I was asked to confirm — jitter corrupting the
measured-rate cut band — **does not apply to turns**, which is where that cut
band was introduced and where the bias it fixed lived.

### 6.2 The straight-line controller does use a finite difference — and is armoured against exactly this

`DistanceController.step` computes `v_meas_mps = (s_along - float(state.s_mm)) / dt / 1000.0`
(`:1690`), a one-tick difference over `/wheel_ticks` (published at
`FPMS_TICKS_HZ` = 25, `fpms_main.cpp:763`). Jitter in tick arrival is genuinely
noise in this estimate.

But the comment immediately above it (`:1682-1687`) already answers the question
that was put to me, and it was written before it was asked:

> *"It is a one-tick difference and therefore noisy, which is exactly why it is
> used only to WIDEN the cut band (via max() inside command_v) and never to
> narrow it: a noise spike can then only stop the move slightly early, never
> carry it past the target. A zero or negative difference — a stalled wheel, a
> dropped frame — simply leaves the fixed band in force."*

The mechanism is `band = max(band, vm * self.lag_s)` (`:1227`), and `vm <= 0`
skips it entirely. **So yes, the max()-only construction is sufficient
protection against the failure that matters**: jitter cannot re-introduce the
overshoot bias §8.2.1 removed. It is structurally incapable of it.

Two honest qualifications, neither large enough to change the recommendation:

- **A max() over a noisy estimate is upward-biased.** `E[max(a, X)] ≥ max(a, E[X])`,
  so noise systematically inflates the band and converts random jitter into a
  small *directional* undershoot. That is worse for repeatability than symmetric
  noise would be.
- **But the fixed term currently dominates anyway.** `MOTION.md` §8.2.1 records
  straight runs undershooting 5–9 mm because `arrive_tol` (20 mm) beats a ~12 mm
  carry, and notes this is "an arrival-reporting tolerance being used as a cut
  threshold — two different quantities sharing one constant." While that holds,
  `v_meas` noise mostly never reaches the `max()` on straight runs at all. Fixing
  *that* is worth more than fixing jitter, and it is a Pi-side change.
- Whichever term dominates on a given move is **UNVERIFIED** at the actual
  operating point.

Residuals are reported (`residual_mm`), so an early cut is visible to the caller
rather than silent.

### 6.3 Where jitter does have a real, if small, path in

Not through the damping term algebraically — `Kd = 2*zeta*sqrt(Kp*tau) - 1`
(`:1846-1852`) is a constant baked once from an assumed `tau = 0.15 s`, and the
Pi-side loop measures its own `dt` rather than assuming it (`:1643`, feeding
`integrate_bounded` at `:1667`).

The real path is that **any latency between command issued and duty applied adds
to the true `tau`, one-for-one.** And `tau` is the parameter `MOTION.md` §8.2.1
names as the most sensitive in the controller: *"a real τ of 0.30 s now overshoots
6.4°… τ is the single most valuable number to measure before this drives
anything, and when in doubt set it HIGHER."* Because `zeta ∝ 1/sqrt(tau)`, an
under-estimated `tau` is exactly an under-damped, overshooting turn.

So the worry is directionally right. Its magnitude is the question, and here is
the honest arithmetic: the *bound* on added latency from §2.4 is ~120 ms, but
that occurs only on a stalled ping, which immediately triggers `fullStop()` and
ends the move anyway. In normal operation the added latency is one superloop
iteration — **UNVERIFIED, plausibly single-digit milliseconds** — against a
`tau` of 0.15 s that is itself **assumed, not measured**, with documented advice
to bias it upward. A few milliseconds of extra lag sits comfortably inside an
uncertainty that already spans 0.15 s to 0.30 s.

### 6.4 Verdict

**I do not agree that determinism on the ESP32 is a precondition for the turn
accuracy §8.2.1 bought.** The turn path runs on gyro rate, which is immune to the
sampling-interval effect; the straight path's cut band can only widen, and is
currently dominated by a fixed tolerance anyway. The premise was overstated —
and it was overstated because the protections that make it overstated are
already in the code.

The one genuine coupling is jitter → `tau` inflation, and it is dwarfed by the
fact that `tau` is unmeasured. **Measure `tau` before touching the firmware.**
That is the change with the actual leverage.

---

## 7. Ranked recommendation

### Rank 1 — **no firmware change. Measure instead.**

**Cost: zero. Risk: zero. Benefit: it tells you whether anything else on this
list is worth doing.**

Three measurements, in order of value:

1. **Measure `tau`.** `MOTION.md` §8.2.1 already calls it "the single most
   valuable number to measure before this drives anything." It is currently
   assumed at 0.15 s and the controller's overshoot is documented as strongly
   sensitive to it. Every argument in §6 collapses into this one.
2. **Read `/fpms_health[6]`** (`g_control_hz_x10`) under motion with WiFi up and
   the agent connected. The instrument is already on the wire. Remember §2.4's
   caveat: the ≥250 ms window reports a mean, and a mean hides a tail. If you
   want the tail, the cheap change is to health telemetry (track and publish the
   *minimum* observed interval), not to the control architecture.
3. **Separate `arrive_tol` from the cut threshold** on the Pi. §6.2 — this is
   the term actually dominating straight-run error today, it is a Python change,
   and it is reversible in one commit.

### Rank 2 — a pinned FreeRTOS control task, **held in reserve, not recommended now**

The design, recorded so it does not need re-deriving:

```c
static void fpmsControlTask(void *arg) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        applyMotors();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / FPMS_CONTROL_HZ));
    }
}
xTaskCreatePinnedToCore(fpmsControlTask, "fpms_ctl", 4096, NULL,
                        5 /* > loopTask's 1 */, &g_ctl_task, 1 /* APP_CPU */);
```

`vTaskDelayUntil` schedules against an absolute deadline so execution time does
not accumulate as drift, and with the mandatory `CONFIG_FREERTOS_HZ = 1000`
(§2.2) a 20 ms period is exactly 20 ticks with no rounding error. Priority 5
preempts `loopTask`'s 1, so the blocking `readBytes` can no longer delay a
control tick. Core 1 rather than core 0, because core 0 runs the WiFi/BT stack at
priority ~23 which would preempt a priority-5 task anyway (reasoning FROM-DOCS;
the specific choice is **UNVERIFIED** until measured).

**Why it is not recommended now:**

- The safety case it looks like it fixes is **already fixed** (§2.3), better than
  a task would fix it — `applyMotors()` at the top of `loop()` runs in every link
  state including ones where a task might be blocked on a mutex.
- The regularity case it would fix is **not established to matter** (§6): the
  turn path is on gyro rate, the straight path's band only widens.
- It introduces a **genuine new failure mode**. `g_duty_cmd[4]`, `g_duty_cmd_ms`,
  `g_estop_latched` and `g_have_duty_cmd` are written by micro-ROS callbacks on
  `loopTask` and would be read by `applyMotors()` on another task. Individual
  aligned 32-bit reads are atomic on Xtensa, but the *set* — four duties plus
  their timestamp — is not consistent as a group, and a torn read across the
  deadman freshness check is safety-relevant. It needs a `portMUX_TYPE`
  spinlock around the snapshot. That is a new concurrency bug class introduced
  into `fpms_main.cpp:67`'s invariant — *"`applyMotors()` is the ONLY function
  in this file that touches a motor"* — the single most safety-critical function
  in the firmware, during a competition build.
- FreeRTOS is a **soft** real-time kernel. Priority preemption buys determinism
  against your own application, not a guaranteed bound. Against a 300 ms deadman
  window already satisfied with 120 ms of margin, that is buying something we
  already have.

Revisit only if measurement (Rank 1) shows the control tick is genuinely ragged
*and* that raggedness is traced to a motion error that is not explained by an
unmeasured `tau`.

### Rank 3 — Zephyr port

**Benefit: moderate. Porting cost: very high. Risk: high.** Better drivers on
paper. But it requires an unmaintained micro-ROS module pinned to Zephyr 4.0/4.1
with an open ESP32 devicetree bug from 2023 and no ESP32-S3 evidence at all;
re-deriving x2-vs-x4 encoder decoding and with it every distance constant;
reproducing coast-not-brake exactly or invalidating the 0.93 turn-coast factor;
and replacing a hand-written IMU driver that exists because of a part-specific
register bug that silently returns zeros. Every one is a way to lose a run to a
symptom that looks like something else.

### Rank 4 — NuttX port

**UNRESOLVED (§4), and last on cost regardless.** Same rewrite as Rank 3, plus I
cannot currently tell you whether the ICM-42670-P driver or a 4-unit x2-capable
encoder path even exist.

### The rewrite question, answered directly

**The case for rewriting this firmware is not overwhelming. It is not close, and
it is weaker than it looked when this survey started.** The firmware is
hardware-verified and its constants are measured, several the hard way — the
counts/mm error, the encoder-vs-motor polarity runaway, the gyro register
non-contiguity, the 8 MB flash header boot loop, the CDC-on-boot serial
rebinding. Every one is a lesson currently encoded in working code. A port keeps
the lesson and discards the encoding.

More than that: **the specific defect a port was supposed to fix is not there.**
The deadman is safe by construction, and the motion controller was already built
to tolerate a noisy rate estimate. This firmware is fine. The effort belongs on
the Pi side — measuring `tau`, and separating `arrive_tol` from the cut
threshold.

---

## 8. Open items

1. **Measure `tau`.** §6.3, §7 Rank 1. The highest-leverage number in the stack.
2. **§4 is unresolved**, not dismissed. NuttX ESP32-S3 driver coverage —
   especially PCNT decoding mode and ICM-42670-P — was never established.
3. **Jitter magnitude is UNVERIFIED.** §2.4 gives a source-derived *bound*
   (~120 ms worst single gap, self-limiting) and no distribution.
4. **Which cut-band term dominates at the real operating point is UNVERIFIED**
   (§6.2). It decides whether tick jitter reaches the straight-line controller
   at all.
