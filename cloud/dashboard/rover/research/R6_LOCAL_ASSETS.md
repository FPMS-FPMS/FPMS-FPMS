# R6 — LOCAL ASSETS (disk archaeology, read-only)

Scope: `C:/Users/ruchi/**`, `C:/Espressif`, `.platformio`, `Arduino15`, Program Files.
One filesystem drive (`C:`, 224 GB used). No removable/SD volumes mounted.
Nothing was modified, moved, copied or deleted.

## HEADLINE

**We can flash a working ESP32-S3 rover firmware today, from a binary already on this
disk.** A complete, custom, bare-metal Arduino firmware for the **Yahboom
YB-EET01-V2.0** board was written, compiled and (evidently) flashed on this machine on
**2026-05-02**. The build cache still holds a verified 4 MB full-flash image plus its
`sdkconfig`, `partitions.csv` and complete GPIO pinout.

This is not the stock Yahboom micro-ROS firmware — it is a **replacement** that drives
the motors directly via LEDC PWM with **no dead zone at all**, which is precisely the
approach R4 recommended and costed at "real build + flash effort". That work is
already done and sitting on disk.

## 1. FLASHABLE ESP32-S3 IMAGE — verified, not inferred

Directory: `C:/Users/ruchi/AppData/Local/arduino/sketches/6361731294F608069694E6D3860AD966/`

| File | Size | Verified header | Why it matters |
|---|---|---|---|
| `ajhb.ino.merged.bin` | 4,194,304 B | magic `0xE9`, **chip_id `0x0009` = ESP32-S3**; `0x8000` = `0xAA` (partition table), `0x10000` = `0xE9` (app) | **Single-command full flash.** `esptool write_flash 0x0 ajhb.ino.merged.bin`. |
| `ajhb.ino.bin` (app only) | 298,128 B | magic `0xE9`, 5 segments, chip_id 9 | App-only flash to `0x10000`, preserves NVS. |
| `ajhb.ino.bootloader.bin` | 19,504 B | magic `0xE9`, chip_id 9 | Bootloader @ `0x0`. |
| `ajhb.ino.partitions.bin` | 3,072 B | magic `0xAA` | Partition table @ `0x8000`. |
| `sdkconfig` | 110,490 B | `CONFIG_IDF_TARGET="esp32s3"`, `CONFIG_IDF_FIRMWARE_CHIP_ID=0x0009`, `CONFIG_ESPTOOLPY_FLASHMODE_QIO=y`, `CONFIG_SPIRAM_MODE_QUAD=y` | The exact IDF config the image was built against. |
| `partitions.csv` | 305 B | — | `nvs@0x9000`, `otadata@0xe000`, `app0@0x10000 (0x140000)`, `app1@0x150000`, `spiffs@0x290000`, `coredump@0x3F0000`. |
| `ajhb.ino.map` | 16.4 MB | — | Full symbol/address map of the shipped image. |
| `build.options.json` | 855 B | — | FQBN: `esp32:esp32:esp32s3:...,FlashMode=qio,FlashSize=4M,PartitionScheme=default,PSRAM=disabled,CPUFreq=240,UploadSpeed=921600`. Core **3.0.7**. |

**Discrepancy worth flagging before flashing:** the FQBN says `FlashSize=4M` and
`PSRAM=disabled`, the `sdkconfig` says `FLASHSIZE="16MB"`, and the source header
comment says "4MB (32Mb) / OPI PSRAM". The merged image is 4 MB. Confirm the real
flash size with `esptool flash_id` before writing, exactly as R4 §4 already instructs.

## 2. THE SOURCE — pinout and geometry (read directly from the build cache)

`sketch/ajhb.ino.cpp` (11,965 B) — self-identifies as
`FPMS Yahboom YB-EET01-V2.0 — Final Firmware`, `ESP32 Core: 3.0.7 ONLY`.

- **Motor pins:** `M1A 4, M1B 5, M2A 15, M2B 16, M3A 9, M3B 10, M4A 13, M4B 14`
- **Encoder pins:** `E1A 6, E1B 7, E2A 47, E2B 48, E3A 11, E3B 12, E4A 1, E4B 2`
- **Other:** `SPRAY_PIN 8`, `BEEP_PIN 46`, `LED_PIN 45`, `OPI_TX 17`, `OPI_RX 18`
  (Orange Pi UART link)
- **PWM:** `PWM_FREQ 25000`, `PWM_BITS 8`, `PWM_MAX 255`, `DRIVE_SPD 180`,
  `TURN_SPD 150`. Written with raw `ledcWrite()` — **no `PWM_MOTOR_DEAD_ZONE`
  equivalent exists in this firmware.** Full 1–255 duty range is reachable.
- **Geometry:** `WHEEL_DIAM 70.0 mm`, `WHEEL_CIRC 219.9 mm`, `ENCODER_CPR 1170.0`
  (documented as 13 lines × 45 gear ratio × 2× decode), `COUNTS_PER_MM 5.32`,
  `WHEEL_BASE 100.0 mm`, `TURN_90_COUNTS ≈ 418`.
- Quadrature decoding via 4 `IRAM_ATTR` ISRs; note `isr3`/`isr4` use **inverted**
  sign vs `isr1`/`isr2` (right-side motors mounted mirrored).
- Contains ESP-NOW receive (`SensorPacket{sensor_id, command[8], reading, seq}`,
  reacts to `"ALERT"` from sensor_id 1–2), plus hard-coded `runRoute1()` /
  `runRoute2()` open-loop spray routes and a `force_stop` flag.

**Caveat — the cached source is incomplete.** The `.ino.cpp` is 288 lines and stops
mid-file at the `// ── ORANGE PI SERIAL ──` comment; it contains **no `setup()` or
`loop()`**. Its mtime (10:10) is *later* than the binaries (09:52), i.e. it is stale
output from a subsequent compile that did not finish. The **binaries are complete and
consistent; the recovered source is not** and will not compile as-is.

**The original sketchbook is not readable.** `C:/Users/ruchi/OneDrive/Documents/Arduino`
is a OneDrive `ReparsePoint` that enumerates as empty (cloud-only / dehydrated). The
`ajhb`, `FPMS/ajhb_copy_20260502101203`, `hh`, `arduin` and `receiveresp` sketches are
referenced by `.arduinoIDE/recent-sketches.json` but their `.ino` files are **not
present locally**. Hydrating that OneDrive folder is the single highest-value next
action — it would recover the full firmware source. A second ESP32-S3 build dir
(`5FBB08AAFD624918FFE0DCC407B2371F`, `ajhb_copy_...`) holds only a bootloader and the
same truncated cpp — no app binary.

## 3. TOOLCHAIN — already installed, zero setup cost

| What | Where | Why it matters |
|---|---|---|
| `esptool.exe` **4.6** (also 4.5.1) | `…/Arduino15/packages/esp32/tools/esptool_py/4.6/esptool.exe` (6.6 MB) | Backup (`read_flash`) and flash the merged image **now**. |
| Arduino ESP32 core **3.0.7** | `…/Arduino15/packages/esp32/hardware/esp32/3.0.7/` | The exact core the image was built with. `esp32s3-libs` + `esp-xs3` toolchain present — a rebuild needs no downloads. |
| `arduino-cli.exe` | `C:/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe` (37.8 MB) | Headless compile/upload. Not on PATH. |
| ESP-IDF **v5.2.6** | `C:/Espressif/v5.2.6/esp-idf/` (`export.ps1`, `tools/idf.py`) | Full IDF + xtensa/riscv GCC, ninja, cmake, openocd, venv `esptool.exe`. Caveat: micro-ROS wants **5.1.2**; 5.2.6 needs a second install or a version bump. |
| `STM32_Programmer_CLI.exe` | `C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/` | Flashes the STM32 `.hex` below. |
| STM32duino core 2.12.0 | `…/Arduino15/packages/STMicroelectronics/hardware/stm32/2.12.0` | Used for the `banana`/`STM32MAIN` builds. |
| WSL2 distros | `Ubuntu`, `Ubuntu-22.04`, `docker-desktop` (all Stopped) | A plausible micro-ROS/colcon build host. **Contents not scanned** — Linux-side assets remain unchecked. |

## 4. YAHBOOM ROSMASTER STM32 FIRMWARE (different board, still valuable)

`C:/Users/ruchi/OneDrive/Documents/Rosmaster_V3.5.1/V3.5.1/` — 225 files, 5.7 MB.
Yahboom's `ROS-Driver-Board-FW` for the **YB-ERF01 / STM32F103HD** board (Keil MDK +
FreeRTOS). **Not the ESP32-S3 board — do not flash it there.** Contains
`Source/APP/protocol.c` (26 KB wire protocol), `app_motion.c`, `app_pid.c`,
`bsp_motor.c`, `bsp_encoder.c`, ICM20948/MPU9250 drivers, plus prebuilt
`output/rosmaster_V3.5.1.hex` (250 KB) and `Listings/rosmaster.map` (698 KB).
`CHANGELOG.md` V3.1 records "hardware version upgraded to **V2.0**" — a likely source
of the "V2.0" board-name confusion.

Constants read directly from that source:
`MOTOR_MAX_PULSE (3600)`, `MOTOR_IGNORE_PULSE (1600)`,
`MOTOR_SUNRISE_IGNORE_PULSE (2000)` — a ~44 % dead zone, clamped in `app_pid.c:86-96`;
`ENCODER_CIRCLE_205/330/450/550 = 2464.0 / 1320.0 / 1040.0 / 836.0` counts/rev
(`app_motion.h`), selected by car type. **None of these match the ESP32-S3 rover's
1170 CPR** — they belong to a different chassis family.

## 5. OTHER ARTIFACTS

| What | Where | Size | Why it matters |
|---|---|---|---|
| `banana` motor smoke test (STM32) | build cache `7037386821071F2306B4DDBB96934A16/` | `.bin` 19 KB, `.elf`, `.hex`, `.map` | 4-motor bring-up test, FQBN `GENERIC_F103RCTX`. Pins `PA8/PB0/PB1/PA11` + `PC6-PC9` — matches Rosmaster `bsp_motor.h`. Console on `PA10/PA9`. |
| `STM32MAIN` register-level PWM probe | build cache `1953E63699C423081042172496F60905/` | `.bin` 12 KB, `.elf` | Bare-metal `TIM8` diagnostic: `ARR=3599`, `CCR1=1800`, AFIO partial remap, samples `PC6`. Confirms the STM32 PWM path was being debugged. |
| ROSMASTER M1 V1 chassis models (7 × `.3mf`) | `C:/Users/ruchi/3D Objects/3D Builder/` | up to 7.5 MB | Physical hardware documentation — base, top body, chassis w/o wheels, "circular locator mounts". |
| `FPMS_CODE_FOR_REBUILD_20260506_214533.tar.gz` | `C:/Users/ruchi/Desktop/` | 42 KB | 5 entries: `fpms_B6_STRAIGHT_ONLY.py`, `fpms_B7_WITH_SPRAY.py`, 2 start scripts, `test_esp32_spray_from_orangepi.py`. **Not** in `_golden_ref/`. |
| ~30 other Arduino sketch builds | `…/AppData/Local/arduino/sketches/` | — | Mostly `esp32:esp32:esp32` (classic ESP32) FPMS sensor/ESP-NOW nodes: `FPMS`, `ZONE1`, `SCORCH_NODE1`, `Rover_Receiver`, `ESP32_bridge`, `hotspot`, `ESPSENSOR`; plus `arduino:avr:uno` sketches (`rover`, `ROVER3`, `ARM`). |
| `fpms-dashboard` (older copy) | `C:/Users/ruchi/Downloads/fpms-dashboard/fpms-dashboard/` | — | `pi-publishers/lidar_publisher.py`, `frontend/src/pages/Lidar.jsx`. Superseded. |
| `Bluetooth_RC_Car.ino` | `C:/Users/ruchi/Downloads/Bluetooth_RC_Car/` | 4.6 KB | Misleading name — an ESP32 WiFi humidity-alert node (MH-RD → GPIO4 → Pi HTTP). **Contains a hard-coded WiFi SSID and password in plaintext; do not copy this file or its contents into this public repo.** |

## ALREADY CAPTURED vs NEW

- **Already in `_golden_ref/`** (not re-reported): `B6_*`, `B84_lidar_gap.py`,
  `B86_autonomous_driving.py`, `P5_*`, `phase6_*` — 10 Python files, **zero firmware**.
  The eMMC pull was rover-side Python only.
- **New:** everything in sections 1–5 above.

## EXPLICIT NEGATIVES

- No file or directory named `microROS*`, `PLAMSLAM*`, or `yahboomcar*` anywhere.
- No downloaded Yahboom archive, PDF, pinout or schematic document.
- No `Rosmaster_Lib` Python package installed (AppData Local/Roaming, `.local`,
  system Python).
- No standalone ESP-IDF *project* (`CMakeLists.txt` + `sdkconfig` outside the Arduino
  build cache) for any ESP32 target — only the IDF installation itself.
- No micro-ROS source, `colcon.meta`, or `micro_ros_espidf_component` on disk.
- No stock/original Yahboom ESP32-S3 firmware image — **no backup of what is currently
  on the rover exists.** R4 §4's `read_flash` dump is still mandatory before writing.
- PlatformIO has **only** the `ststm32` platform; **no `espressif32`**.
- `spray_rover` (`C:/Users/ruchi/STM32CubeIDE/workspace_2.1.1/spray_rover/`) is an
  empty CubeIDE skeleton — `main.c` is the generated `for(;;);` template. Value is only
  that it targets `STM32F103RCTX` with a working CMake/GCC toolchain file.

## PRIVACY NOTE

The sweep incidentally traversed directories of personal/immigration documents and an
`.ssh` and `.aws` credential directory under `C:/Users/ruchi/`. Their contents were
**not** opened, read or copied, and their paths are deliberately not enumerated. The
one credential actually observed (the WiFi password in `Bluetooth_RC_Car.ino`) is
reported by path and type only.

## BOTTOM LINE

1. **Flashable ESP32-S3 image exists on disk and is header-verified** (chip_id 9).
   Combined with the installed `esptool` 4.6, the rover can be reflashed today.
2. **Back up the rover's current flash first** — no dump of the existing firmware
   exists anywhere on this machine.
3. **Hydrate `OneDrive/Documents/Arduino`** to recover the full `ajhb.ino` source. The
   cached copy is truncated and missing `setup()`/`loop()`.
4. The recovered pinout and geometry (motor/encoder GPIOs, 1170 CPR, 70 mm wheels,
   100 mm wheelbase, 25 kHz 8-bit PWM, **no dead zone**) supersede the Rosmaster STM32
   constants for this rover and should be reconciled against R4 and R2.
