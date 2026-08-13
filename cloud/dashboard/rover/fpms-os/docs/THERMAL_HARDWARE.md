# Thermal camera survey: what FPMS-OS should be prepared to find on a USB port

`fpms_rover_agent.py` detects fire with `detect_fire()` — an HSV heuristic over
the RGB webcam that looks for bright, saturated, red-orange blobs. The comment
at line 119 is explicit about its own limits:

> a hit raises an event that the thermal sensor and the cloud VLM then
> corroborate.

This document is about the first half of that sentence. It surveys what
hardware can actually be plugged into the Orange Pi 5B to provide that
corroboration, and — much more importantly — **which of those devices can
corroborate anything at all.**

It is a **research document**. No repo file was changed to write it and no
image was built. Nothing here was measured; there is no hardware in hand.

**The one-line answer:** an orange traffic cone and a fire are the same colour
and roughly 400 °C apart. Only a *radiometric* camera can see the second
difference. A large fraction of cheap thermal cameras cannot, and if FPMS
corroborates a colour detection with one of those, it has confirmed nothing
while looking like it confirmed something.

---

## 0. How to read the evidence tags

Every load-bearing claim carries one. Nothing in this document was measured.

- **FROM-VENDOR-SPEC** — a manufacturer datasheet, spec sheet, export fact
  sheet, or product page.
- **FROM-DOCS** — project documentation, a reverse-engineered open-source
  driver's README, or the source of such a driver read directly. Community
  knowledge, not a vendor guarantee.
- **UNVERIFIED** — believed, inferred, or reported by one source only. Treat as
  a lead to check against hardware, not a fact.

Where two sources disagree, both are given and the disagreement is named.

---

## 1. The spine: radiometric versus non-radiometric

This is the distinction that decides whether a purchase is useful, and it is
the one that marketing copy is most careless about. Every device in this
document is judged against it first and on resolution second.

### 1.1 What a non-radiometric camera actually gives you

A microbolometer array measures scene flux per pixel. Turning flux into a
*picture* requires choosing a mapping from flux to display value. Almost every
cheap camera does this with **AGC** — automatic gain control — which finds the
minimum and maximum in the current frame and stretches the palette between
them. FROM-DOCS.

The consequences are exactly the ones that break corroboration:

- The colour of a pixel depends on **what else is in the frame**. The same
  campfire is white-hot when it is alone against cold ground and mid-grey when
  a hotter exhaust pipe enters the frame.
- The mapping **changes between consecutive frames**. A thresholded "hot pixel"
  mask is not stable over time.
- There is **no recoverable temperature**. Not "poorly calibrated" — absent.
  The scaling parameters are discarded before the pixel leaves the device.

A non-radiometric thermal camera is a camera that sees a different band of
light. That is genuinely useful for some things (seeing through smoke, seeing
in darkness, seeing a warm body against cold background). It is *not* useful
for the specific job asked of it here.

### 1.2 Why "corroborate with a second picture" is worse than nothing

`detect_fire()` fires on orange-ish saturated blobs. Suppose FPMS confirms with
a non-radiometric thermal camera by thresholding its false-colour output:

- A sunlit orange traffic cone is warmer than the grass around it. Solar-loaded
  plastic reaching 50–60 °C on a hot day is entirely ordinary. UNVERIFIED (no
  measurement, but well within the range of dark/coloured plastic in sun).
- Against grass and sky, AGC will map that cone to the **top of the palette** —
  white/yellow, the same colours a fire gets.
- The thermal image therefore agrees with the RGB detection, on the wrong
  object, for a physically real reason. The two sensors are not independent:
  both are responding to "this object is a bit different from its background".

The system now reports a fire with two-sensor agreement. The confidence is
manufactured. This is strictly worse than reporting a single-sensor colour hit,
because a single-sensor colour hit is honestly labelled as weak, and the
operator treats it accordingly.

**Design rule for the driver layer that follows from this:** the thermal node
must publish °C — a number — or publish nothing and declare itself
non-radiometric. It must never publish a colourised image into a code path that
treats it as evidence. If a device can only give a picture, that picture belongs
on the operator's screen for a human to look at, and nowhere near the
corroboration logic.

### 1.3 The three things that make temperature recoverable

1. **A 16-bit pre-AGC stream leaves the device.** Usually `Y16` /
   `GRAY16_LE` / `V4L2_PIX_FMT_Y16`, sometimes a second image-height's worth of
   raw data appended below the visible image. If only 8-bit or MJPEG leaves the
   device, the game is already over.
2. **A documented (or reverse-engineered) counts→temperature mapping exists.**
   Either the device does the conversion internally (Lepton TLinear: pixel =
   K × 100) or a calibration table is shipped in the stream or in EEPROM
   (MLX90640, InfiRay/HTI family).
3. **Emissivity and ambient/reflected temperature can be supplied.** Any
   radiometric reading is a reading of *apparent* temperature. Getting from
   apparent to actual needs the target's emissivity (ε) and the reflected
   ambient temperature. For fire-vs-cone this matters much less than for
   metrology — the discrimination is hundreds of kelvin, not a few — but the
   parameter must exist in the API or the number is not defensible.

### 1.4 Where flat-field correction / shutter events enter

Microbolometer arrays drift. Correcting for that requires periodically closing
a shutter over the whole array so every pixel sees a uniform temperature, then
computing per-pixel offsets. This is **FFC** (flat-field correction), a form of
NUC. FROM-DOCS.

- The FLIR Lepton has a mechanical shutter and runs FFC automatically on a
  ~3-minute timer and on ambient-temperature change since the last FFC; it can
  also be commanded. FROM-DOCS.
- During and immediately around an FFC the image is invalid or frozen.
- Lepton telemetry exposes FFC state, so a driver can drop those frames.
  FROM-DOCS.
- The InfiRay/HTI-family cameras also have a shutter; the open drivers expose a
  "calibrate" command that triggers it. FROM-DOCS (driver source, §2.3).

**Consequence for FPMS:** a fire alarm raised on a frame captured mid-FFC is
noise. The driver layer must expose an FFC/validity flag, and the corroboration
logic must refuse to act on frames without one. A camera that hides its shutter
events will occasionally produce a whole frame of nonsense with no warning.

---

## 2. Class 1 — USB UVC thermal cameras

The most attractive class for this rover: the Orange Pi 5B has spare USB ports,
already runs an RGB UVC webcam, and needs no header wiring.

### 2.1 FLIR Lepton on PureThermal / GroupGets carrier boards

The Lepton is a bare module (VoSPI + I2C, §3.4). The **PureThermal 1 / 2 /
Mini / Mini Pro** boards from GroupGets put an STM32 in front of it that
converts VoSPI into a standard **UVC** stream, so the whole assembly enumerates
as `/dev/video*` with no vendor driver. FROM-DOCS.

**The variant table is the whole story here:**

| Module | Array | Radiometric? |
|---|---|---|
| Lepton 2.0 | 80 × 60 | No FROM-VENDOR-SPEC |
| Lepton 2.5 | 80 × 60 | **Yes** — radiometric variant FROM-VENDOR-SPEC |
| Lepton 3.0 | 160 × 120 | No FROM-VENDOR-SPEC |
| Lepton 3.5 | 160 × 120 | **Yes** — calibrated across all 19,200 pixels FROM-VENDOR-SPEC |

A "3.0" and a "3.5" look identical, cost differently, and produce data of
completely different value. **Buying a 3.0 by accident is the single most
likely way this project ends up with a picture-only sensor while believing it
bought a thermometer.** Same for 2.0 vs 2.5.

**Formats over UVC.** PureThermal exposes an AGC'd colour/greyscale stream
(BGRA / RGB565 / GRAY8 / UYVY, depending on board and firmware) **and** a raw
16-bit stream (`Y16` / `GRAY16_LE`). FROM-DOCS. Both are standard UVC formats;
no vendor SDK and no magic control transfer is needed to *get* the raw format on
Linux — you select it like any other V4L2 format. The historical pain is on the
host side (§5), not the device side.

**Counts → temperature.** For a radiometric part with **Radiometry enabled and
TLinear enabled** (the factory default for radiometric SKUs), each 16-bit pixel
is **absolute temperature in kelvin at 0.01 K resolution** — divide by 100 for
kelvin, subtract 273.15 for °C. FROM-DOCS. With radiometry or TLinear disabled,
the same 16 bits carry 14-bit *scene flux counts*, which are **not** temperature
and have no fixed mapping to it.

**Accuracy.** High-gain mode: greater of ±5 °C or 5%, typical. Low-gain mode:
greater of ±10 °C or 10%, typical. FROM-VENDOR-SPEC (Lepton engineering
datasheet). These are measured against a blackbody at 25 cm at equilibrium and
corrected for target emissivity; field accuracy will be worse.

**The gain-mode trap, and it matters for fire.** High gain covers roughly
−10…140 °C; low gain extends to roughly −10…450 °C. FROM-VENDOR-SPEC.
A radiometric Lepton left in its default high-gain mode will **saturate on an
actual flame** and report a flat ceiling. It will still say "very hot", which is
enough to distinguish a fire from a cone, but any attempt to report a flame
temperature from high-gain data is wrong. UNVERIFIED whether the PureThermal
firmware exposes gain-mode switching over UVC controls without dropping to the
Lepton CCI.

**FOV.** Lepton 3.5 with the standard lens: ~57° horizontal / ~71° diagonal.
FROM-VENDOR-SPEC. Different from any reasonable RGB webcam — see §7.

**Frame rate.** ~8.7 Hz of *unique* frames (§4).

**Cost.** PureThermal Mini Pro JST-SR bundled with a Lepton 3.5 was listed at
roughly €400 ex-VAT / US$400–500 at several distributors. FROM-VENDOR-SPEC
(retail listings; prices move).

**Verdict: fully radiometric (3.5 / 2.5 only), vendor-documented, the
reference-quality option, and the most expensive per pixel by a wide margin.**

### 2.2 InfiRay / Xtherm — P2 Pro, T2S+, T2L, T3S (and the TOPDON / HTI clones)

This is one hardware family wearing several brands. The same sensor + USB
bridge appears as InfiRay P2 Pro, Xtherm T2S+/T2L/T3S, TOPDON TC001, and HTI
HT-301, with different housings and different bundled Windows/Android apps.
FROM-DOCS. The open-source drivers for one frequently work on the others, which
is the strongest single argument for supporting this family.

**Enumeration.** Standard UVC. The P2 Pro appears as USB `0bda:5830` — a
Realtek bridge VID — and shows up as an ordinary `/dev/video*`. FROM-DOCS
(`LeoDJ/P2Pro-Viewer` source).

**Format — and this is the important structural fact.** The P2 Pro offers a
**256 × 384 YUY2 @ 25 fps** mode. The sensor is 256 × 192. The frame is
*double height*: the **top half is the AGC'd pseudo-colour image**, the
**bottom half is the raw 16-bit thermal data**. FROM-DOCS (driver source).
The driver disables OpenCV's YUV→RGB conversion, grabs the raw buffer, splits it
in half by length, and reinterprets the lower half as `uint16` reshaped to
192 × 256.

So: the picture and the measurement arrive in the same frame, perfectly
registered with each other, at 25 Hz. That is an unusually good deal.

**Counts → temperature.** The community formula is
`T(°C) = raw / 64 − 273.15` — i.e. 1/64 K per count. FROM-DOCS, community-
derived. **UNVERIFIED against vendor documentation**, and note the sources
disagree in emphasis: `P2Pro-Viewer`'s own source, read directly, extracts the
raw `uint16` array and applies *no* conversion (the README lists "switch to
actual raw sensor readings" and "apply pseudo color from raw temperature data"
as future work), while secondary write-ups state the `/64 − 273.15` form with an
environmental correction taken from device metadata. Both can be true — the
divide-by-64 is likely correct as a first-order mapping, with vendor emissivity
and ambient corrections layered on top. **Do not treat the absolute value as
metrology-grade; treat the ~300 K gap between a cone and a flame as far larger
than any plausible error in it.**

**The HTI HT-301 / Xtherm T2S+ path is better documented than the P2 Pro path.**
`stawel/ht301_hacklib` (and the `diminDDL/IR-Py-Thermal` and
`CEAD-group/ht301_t2s_plus_python` derivatives) read, FROM-DOCS from source:

- Resolutions 240×180, 256×192, 384×288, 640×512 depending on model.
- `cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)` — mandatory, raw 16-bit.
- **`cap.set(cv2.CAP_PROP_ZOOM, 0x8004)` enables raw 16-bit mode.** This is the
  "magic control" for this family: the `ZOOM` V4L2 control is hijacked as a
  general command channel, with values encoding a register position and a byte.
  `0x8000` triggers a calibration (shutter/FFC), `0x80ff` persists parameters.
- **4 extra rows** are appended below the image carrying per-device calibration
  coefficients and scene metadata.
- Temperature is computed from those coefficients through a device-specific
  lookup plus a Stefan-Boltzmann-form computation, with **distance, atmospheric
  and emissivity terms** and a range-correction pair (normal `m=1, b=0`;
  high-range `m=1.17, b=−40.9`).

That is a genuinely radiometric pipeline with emissivity exposed, running in
pure Python over OpenCV/V4L2 — no vendor binaries, no x86 dependency.

**TOPDON TC001.** 256×192; `leswright1977/PyThermalCamera` and
`92es/Thermal-Camera-Redux` read it on Linux and Raspberry Pi via `v4l2-ctl`
device enumeration and OpenCV, and PyThermalCamera explicitly credits LeoDJ's
reverse engineering of the image format. FROM-DOCS. **UNVERIFIED** whether
TC001, TC001 Plus and TC002 all expose the raw half identically — they are
different SKUs and the assumption should be checked per unit.

**Measurement ceiling.** The P2 Pro is specified roughly −20…550/600 °C
(listings quote −4 °F to 1112 °F). FROM-VENDOR-SPEC. **This is the single most
fire-relevant number in the document** — it does not saturate on a flame.

**Cost.** Roughly US$200–350 depending on brand and bundle. FROM-VENDOR-SPEC
(retail).

**Verdict: radiometric in practice, via reverse-engineered but well-exercised
open drivers. No vendor guarantee, no vendor support, and the SKU-to-SKU
variation is a real risk. Best value in the survey.**

### 2.3 Seek Thermal — Compact, CompactXR, CompactPRO, Mosaic/Micro Core

**Not UVC.** The Compact family speaks a proprietary USB protocol and does
**not** appear as `/dev/video*`. Access is via `libseek-thermal` (libusb +
OpenCV, userspace) or the official Seek SDK. FROM-DOCS.

**And the open path is not radiometric.** `libseek-thermal` states plainly that
it does not support absolute temperature because the raw-16-bit→temperature
function is not known; contributors have fitted **linear approximations**
claimed accurate to about ±1 °C after warm-up, while acknowledging the result is
"very approximate depending on test conditions" and that more calibration data
is needed. FROM-DOCS (repo issues/PRs). Note the internal disagreement in that
project's own threads — a claimed ±1 °C alongside an admission that the mapping
is unknown cannot both be robust.

**The official SDK** (`seekcamera` / `seekcamera-python`) covers the **Mosaic
Core** and **Micro Core** OEM modules and does offer thermography frame formats;
the Mosaic Core spec sheet lists user-selectable outputs including "16-bit
filtered pre-AGC". FROM-VENDOR-SPEC. But:

- "16-bit pre-AGC" is **not the same claim as "temperature"** — pre-AGC counts
  still need a calibration to become kelvin. Which SKUs unlock true thermography
  output is **UNVERIFIED**.
- The SDK ships **prebuilt binaries**. The documentation reachable for this
  survey says "Linux, Windows, Android" and defers architecture details to a
  developer portal that returned HTTP 403. **arm64/aarch64 availability is
  UNVERIFIED** and is the deciding question for this rover.

**Verdict: treat the consumer Compact family as picture-only. The OEM Mosaic /
Micro Core may be radiometric but is gated behind an SDK whose arm64 support
this survey could not confirm. Not a first choice.**

### 2.4 Generic HTI, and unbranded "thermal camera" USB dongles

HTI sells many SKUs beyond the HT-301 (HT-18, HT-19, HT-102…), and generic
Amazon/AliExpress dongles proliferate. Many of these output **only** an AGC'd
MJPEG or 8-bit YUYV image. There is no raw half, no `Y16`, no metadata rows.

**Verdict: assume picture-only until a specific SKU is proven otherwise by
`v4l2-ctl --list-formats-ext` showing a 16-bit format or a double-height mode.
An unbranded dongle with no named sensor should be treated as a novelty.**

---

## 3. Class 2 — I2C / SPI sensor modules on the 40-pin header

### 3.1 MLX90640 (Melexis) — 32 × 24

The strongest small-sensor option, and the one with the cleanest arm64 story.

- **Bus/address:** I2C, factory default **0x33**, reprogrammable to up to 127
  addresses. FROM-VENDOR-SPEC.
- **Frame rate:** programmable 0.5–64 Hz, but bandwidth-limited in practice —
  **~8 FPS at a 400 kHz bus, up to 32 FPS at 1 MHz**. FROM-DOCS (Melexis driver
  notes). Whether the RK3588S header I2C runs reliably at 1 MHz over dupont
  wiring is **UNVERIFIED**; assume 400 kHz / 8 FPS when planning.
- **FOV variants:** **BAA = 110° × 75°**, **BAB = 55° × 35°**.
  FROM-VENDOR-SPEC. Choose deliberately — this is the parameter that determines
  how badly it mismatches the RGB camera (§7).
- **Target range:** −40…300 °C. FROM-VENDOR-SPEC. Does not saturate on a modest
  fire; will saturate on a large one.
- **Radiometric: yes, and openly so.** The Melexis reference driver reads the
  device EEPROM calibration and computes per-pixel object temperature `To` with
  **emissivity** and **reflected/ambient temperature `Ta`** as explicit
  parameters. The maths is published in the driver document. FROM-VENDOR-SPEC.
- **arm64 support:** excellent. The Melexis C driver builds with a generic Linux
  I2C backend (`make I2C_MODE=LINUX`, no bcm2835, no root). Pimoroni and Adafruit
  Python wrappers, a CircuitPython library, and a pure-Rust `mlx9064x` crate
  (covering 90640 **and** 90641) all exist. FROM-DOCS. Nothing here is
  architecture-specific.

**The subpage caveat.** The MLX90640 reads out in two interleaved subpages
(chessboard pattern); a full frame is two reads. On a **moving** rover the two
subpages sample different scenes and the interleave shows up as a checker
artefact. FROM-VENDOR-SPEC (readout structure); the motion consequence is
UNVERIFIED but follows directly. Read while stopped, or accept the artefact.

### 3.2 MLX90641 — 16 × 12

Same family, same 0x33 default address, same calibration philosophy, quarter
the pixels with correspondingly larger per-pixel IFOV and better per-pixel SNR.
FROM-VENDOR-SPEC. Fewer libraries (the Rust `mlx9064x` crate covers it; Python
support is thinner). **Radiometric: yes.** Choose it over the 90640 only if
range matters more than shape — 16×12 is barely an image.

### 3.3 AMG8833 Grid-EYE (Panasonic) — 8 × 8

- **Bus/address:** I2C, **0x69** default, jumper-selectable to **0x68**.
  FROM-VENDOR-SPEC.
- **Frame rate:** 10 fps or 1 fps (selectable). FROM-VENDOR-SPEC.
- **Accuracy:** ±2.5 °C. **Range: 0…80 °C.** FROM-VENDOR-SPEC.
- **Radiometric: yes** — each of the 64 pixels reports a temperature directly
  (12-bit, 0.25 °C LSB). FROM-VENDOR-SPEC.
- Detects human body heat at roughly ≤7 m. FROM-VENDOR-SPEC.
- **arm64 support:** trivial. Generic `/dev/i2c-N`, Adafruit/SparkFun libraries,
  many independent implementations.

**But read the range line again. 0…80 °C.** A flame, an ember bed, or a hot
exhaust pins this sensor at its ceiling. It can report "something here is at
least 80 °C", which is *not nothing* — a cone in the sun does not reach 80 °C —
but it cannot rank two hot things, cannot measure a fire, and cannot tell a fire
from a car engine.

**What 8×8 can and cannot do.** 64 pixels is **not an image**. It is a
presence-and-bearing detector with 8 columns of angular resolution. It can
answer "is there a warm region, and roughly in which of eight horizontal
sectors". It **cannot**:

- be aligned to an RGB bounding box in any meaningful way — one pixel covers a
  huge solid angle, so a single flame and a single sunlit rock in the same
  sector are one pixel;
- distinguish two adjacent hot objects;
- produce a shape, a boundary, or an area estimate;
- support any kind of tracking.

Do not build a corroboration path that pretends otherwise. Its honest use is a
cheap, always-on "something warm ahead" gate that raises the *priority* of a
colour hit without claiming to localise it.

### 3.4 FLIR Lepton raw, over SPI (VoSPI) on the header

Skipping the PureThermal board and wiring the bare Lepton to the 40-pin header
is possible and is a trap.

- **Video** is **VoSPI** — an output-only slave SPI stream, **max 20 MHz**,
  **SPI mode 3** (CPOL=1, CPHA=1). FROM-DOCS.
- **Packets are 164 bytes** and must be transferred in a single SPI transaction:
  a 16-bit ID word, a 16-bit CRC, then **80 16-bit pixels**. Some modes use
  244-byte packets. FROM-DOCS.
- A **segment** is 60+ packets; the Lepton 3.x sends **4 segments per frame**,
  interleaved with **discard packets** that must be recognised and dropped.
  FROM-DOCS.
- **Control** is a separate I2C **CCI** interface (address 0x2A). FROM-DOCS.
- **The killer:** the host must pull data out **within three line times** of
  generation or the Lepton loses sync and stops producing valid output. FROM-DOCS.
  Recovering sync costs a full resync cycle.

Under a non-realtime Linux userspace with a scheduler that can preempt you at
any moment, meeting a three-line deadline continuously is not a design, it is a
hope. The projects that do this reliably use a PRU core, an FPGA, or a
dedicated MCU (`mixaz/LeptonPRU` uses the BeagleBone PRU with SPI bit-banging;
the PureThermal boards use an STM32). FROM-DOCS.

**Verdict: do not do this. If you want a Lepton, buy the PureThermal board.
The STM32 exists precisely to solve this problem, and it costs less than the
engineering time to fail at it.**

---

## 4. Class 3 — Ethernet / GigE

### 4.1 FLIR Ax5 (A35 / A65) and the A400 series

- **Protocol:** **GigE Vision** transport with a **GenICam** feature interface —
  an industrial-machine-vision standard, not a webcam protocol. FROM-VENDOR-SPEC.
- **Radiometric: yes, properly.** Configuring the camera into
  **TemperatureLinear** mode makes each 16-bit pixel a calibrated temperature.
  FROM-VENDOR-SPEC (FLIR GenICam ICD / knowledge base).
- **Resolution:** A65 is 640 × 512. FROM-VENDOR-SPEC. FLIR's own product page
  marks the A65 **discontinued**; the A400 series is the current radiometric
  GigE line. FROM-VENDOR-SPEC.
- **arm64:** the Spinnaker SDK supports Linux and has published **Ubuntu 18.04
  ARM64** builds. FROM-VENDOR-SPEC. Whether a current Spinnaker release ships a
  **Ubuntu 22.04 arm64** package that installs cleanly on an RK3588S is
  **UNVERIFIED** — and Spinnaker is a prebuilt binary SDK, so if the package
  does not exist, there is no fallback.

**Realism on this rover: low.** Cost is in the thousands of dollars against a
rover whose entire compute is an Orange Pi. Power is PoE or a separate 12 V
supply. It consumes the board's single GbE port. And a GigE Vision stream at
full rate is a meaningful fraction of the link and of the CPU. The data would be
the best in this document; the platform is wrong.

### 4.2 Optris PI 640i / Xi 640

- **Radiometric: yes.** PI 640i: 640 × 480 radiometric, **32 Hz** full-frame
  plus a 125 Hz subframe mode, **−20…1500 °C**. Xi 640: 640 × 480, 32 Hz,
  −20…900 °C. FROM-VENDOR-SPEC.
- **Interface note, because the brief grouped these under Ethernet:** the Optris
  PI series is primarily **USB 2.0** (via an interface box), with Ethernet
  available on some models/packages; the OTC SDK manages cameras "connected via
  USB or Ethernet". FROM-VENDOR-SPEC. Do not assume GigE.
- **SDK:** `libirimager`, a C++ library for **Linux and Windows**, plus LabVIEW
  and MATLAB bindings. FROM-VENDOR-SPEC. **ARM/arm64 support is not documented
  in anything this survey could reach — UNVERIFIED.** Historically Optris has
  shipped ARM builds for Raspberry Pi, but that is UNVERIFIED and should be
  confirmed with the vendor before purchase.

**Verdict for the whole class: technically the best data in the survey, and the
wrong machine. Revisit only if the rover grows a real compute and power budget.**

---

## 5. Frame rate, and what 9 Hz means on a moving rover

**The export rule.** Thermal cameras exceeding ~9 Hz fall under tighter US
export control. Manufacturers deliberately clamp the *unique* frame rate below
9 Hz to stay in the exportable category. FLIR's own export fact sheet places the
Lepton under **EAR ECCN 6A993**, exportable **NLR** to all countries except
Country Group E:1, *because* it operates at 9 Hz. FROM-VENDOR-SPEC.

**The measured-looking detail that is not measured:** the Lepton's VoSPI clocks
out at ~27 Hz but only ~8.7 Hz of those frames are *unique* — each unique frame
is repeated three times. FROM-DOCS. A naive driver reading at 27 Hz will believe
it has a 27 Hz sensor and will triple-count every event.

**Note the asymmetry in this survey:** the InfiRay-family cameras are sold at
**25 Hz** and the Optris at 32 Hz, while the Lepton is clamped to 8.7 Hz. That
is a real capability difference, not marketing.

**What 9 Hz costs on this rover.** 8.7 Hz is 115 ms between unique frames.

- **Translation is fine.** At a rover speed on the order of 0.7 m/s, that is
  ~8 cm of travel per frame. Nothing breaks.
- **Rotation is the problem.** At 1 rad/s yaw, 115 ms is **6.6° of rotation
  between frames**. On a 57° HFOV, 160-pixel-wide Lepton that is ~18 pixels of
  whole-scene image motion — more than 10% of the frame width — every frame.
  The result is smeared thermal frames and, worse, an RGB frame and a thermal
  frame that no longer describe the same scene.
- **Latency compounds it.** A thermal frame is up to 115 ms old before it even
  arrives, plus transport. Any RGB↔thermal association must be done on
  timestamps, not on arrival order.

**The operational rule this implies:** corroborate while near-stationary. When
`detect_fire()` raises a colour hit, the correct next action is to stop (or at
least stop yawing), let a clean thermal frame arrive, check for an FFC flag, and
*then* decide. Corroborating while turning is how a hot object ends up
associated with the wrong bearing.

---

## 6. Reading 16-bit thermal in software: OpenCV vs V4L2

**The trap, stated first:** if the driver layer forgets one flag, everything
still works, a pretty image appears, and every temperature is silently
destroyed. There is no error. This is the same failure class as §1.2 and it is
the most likely way this integration goes wrong in code rather than in
purchasing.

**Can OpenCV `VideoCapture` read a raw 16-bit stream?** On Linux via the V4L2
backend, **yes, conditionally**:

```python
cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"Y16 "))
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)      # <-- without this, everything above is undone
```

- 16-bit single-channel output is produced for `Y16` **only if
  `CAP_PROP_CONVERT_RGB` is explicitly false**. FROM-DOCS (OpenCV PR #7293 and
  the OpenCV Q&A threads).
- Without it, `read()` returns a 3-channel `uint8` array — libv4l has helpfully
  converted the data, discarding the upper bits. FROM-DOCS. The frame looks
  fine. The temperatures are gone.
- The InfiRay/HTI-family drivers set `CAP_PROP_CONVERT_RGB = 0` **and** request
  YUYV at double height, then slice — they do not use `Y16` at all. FROM-DOCS
  (driver source).
- **Known OpenCV rough edges:** a segfault on `release()` after disabling RGB
  conversion (opencv#13697); behaviour that varies with whether the OpenCV build
  is linked against libv4l; and Windows/macOS backends that historically cannot
  do `Y16` at all — which is why GroupGets maintains a **forked libuvc** with
  Y16 support rather than relying on OS capture drivers. FROM-DOCS.

**Recommendation for the driver layer:**

1. **Verify the format is really 16-bit at runtime.** After opening, assert
   `frame.dtype == np.uint16` (or the expected raw byte count) and **refuse to
   start** otherwise. Do not let a silently-converted 8-bit stream reach the
   corroboration path. This single assertion prevents the entire failure class.
2. Prefer direct V4L2 (`v4l2-ctl`, `linuxpy`, or a small ioctl wrapper) over
   OpenCV where the vendor path allows it; use OpenCV where the existing
   open-source driver already does (the InfiRay family), since that path is
   well-exercised.
3. For PureThermal, GroupGets' forked libuvc is the vendor-blessed path and
   sidesteps OpenCV entirely; plain V4L2 `Y16` on Linux is reported to work but
   is **UNVERIFIED** on this board.
4. Log the negotiated fourcc and resolution at startup, every time.

---

## 7. Thermal-to-RGB alignment

A thermal camera has a **different FOV, a different optical axis, a different
resolution and a physical baseline offset** from the RGB webcam. Two examples of
how badly the FOVs mismatch:

- Lepton 3.5: ~57° HFOV at 160 px wide. FROM-VENDOR-SPEC.
- MLX90640-BAA: 110° × 75°. MLX90640-BAB: 55° × 35°. FROM-VENDOR-SPEC.
- A typical USB webcam: ~60–78° HFOV at 640+ px wide. UNVERIFIED for the
  specific unit on this rover — **measure it, do not assume it.**

**Why a single homography is not enough.** A homography maps one image to
another **correctly on one plane only**. Because the two cameras are separated
by a baseline, objects at different depths shift by different amounts —
parallax. The literature handles this by computing **multiple homographies at
different calibration-target distances** and blending them by approximate range,
precisely because one is insufficient. FROM-DOCS (RGB-thermal cross-calibration
literature).

**Calibration is harder than RGB-RGB** because a printed checkerboard is
thermally uniform — it has no features in LWIR. Published approaches use a
heated/backlit target, a cut-out board over a warm background, or a grid of
incandescent bulbs, so the same geometry is visible in both modalities.
FROM-DOCS.

**What getting it wrong looks like:** the thermal hot spot lands on the wrong
RGB box. On a rover that is specifically trying to separate an orange cone from
a fire, and where cones and fires may plausibly be within a few degrees of each
other in bearing, a 5° registration error is enough to attribute the fire's heat
to the cone or the cone's coolness to the fire. **The error mode is not "no
detection" — it is a confident detection of the wrong object.**

**Recommendation, and it is deliberately conservative:** do not fuse at pixel
level in v1.

- Publish thermal as a **region summary**: max °C, mean °C, and the bearing and
  angular extent of the hottest contiguous region, in the thermal camera's own
  frame.
- Associate with the RGB detection **by bearing with a generous tolerance**
  derived from a measured (not assumed) calibration error, plus a margin.
- Report the association's uncertainty rather than hiding it. "Hot region at
  bearing 12° ±8°, 430 °C; colour hit at bearing 9°" is an honest and useful
  statement. A pixel-aligned overlay implies a precision that has not been
  earned.
- Upgrade to a proper homography (or a range-blended set) only after a
  calibration procedure exists and its residual has been measured.

---

## 8. arm64 Linux support, today

Ubuntu 22.04 arm64 on RK3588S. The dividing line is between **source you can
build** and **vendor binaries you must be given**.

| Path | arm64 today | Why |
|---|---|---|
| MLX90640 / MLX90641 | **Yes** | Generic Linux I2C; C/Python/Rust libs, all source FROM-DOCS |
| AMG8833 | **Yes** | Generic Linux I2C, trivial protocol FROM-DOCS |
| PureThermal / Lepton over UVC | **Yes** | Standard UVC + V4L2; GroupGets libuvc fork builds from source FROM-DOCS |
| InfiRay P2 Pro / TC001 / HT-301 / T2S+ | **Yes** | Pure Python + OpenCV + V4L2; drivers are portable source. Community reports of HT-301 on `x86_64`, `aarch64` **and** `armhf` FROM-DOCS |
| FLIR GigE (Spinnaker) | **Probably** | ARM64 builds published for Ubuntu 18.04; **22.04 arm64 UNVERIFIED**, binary-only SDK FROM-VENDOR-SPEC |
| Seek official SDK | **UNVERIFIED** | Prebuilt binaries; platform matrix behind a 403 developer portal |
| Optris `libirimager` | **UNVERIFIED** | Linux/Windows documented; ARM not mentioned in reachable docs |
| Vendor desktop apps (Topdon, HTI, InfiRay, Xtherm) | **No** | Windows/Android only FROM-VENDOR-SPEC. Irrelevant — the open drivers replace them |

**The pattern:** every device with an **open, source-available** driver works on
arm64 without drama. Every device with a **binary vendor SDK** is a coin flip
this survey cannot resolve from a desk. On an RK3588S that is a first-order
selection criterion, not a footnote.

---

## 9. Recommendation table

Ranked for the actual job: **distinguishing a real heat source from an orange
traffic cone, on this rover, this season.** Scored on radiometric capability ×
arm64 support × cost × integration effort.

| # | Device class | Radiometric | arm64 | Approx cost | Integration | Ceiling | Verdict |
|---|---|---|---|---|---|---|---|
| **1** | **InfiRay-family USB UVC** (P2 Pro, T2S+, TC001, HT-301) | **Yes** (open drivers, ε exposed) | **Yes** — pure Python | ~$200–350 | Low — UVC, one flag, slice the frame | **~550–600 °C** | **Support first.** Best ratio in the survey. 25 Hz, 256×192, doesn't saturate on flame, no vendor binaries |
| **2** | **MLX90640** (I2C) | **Yes** — vendor-documented maths | **Yes** — flawless | ~$40–60 | Medium — header wiring, 8 FPS at 400 kHz | 300 °C | **Support second, as the independent check.** Different physics, different bus, different failure modes from #1. Cheap enough to fit both |
| **3** | **FLIR Lepton 3.5 on PureThermal** | **Yes** — TLinear, 0.01 K, telemetry | **Yes** | ~$400–500 | Low — UVC | 140 °C high gain / **450 °C low gain** | The quality answer. Loses on price and 8.7 Hz. **Buy the 3.5, never the 3.0** |
| **4** | **AMG8833** (I2C) | **Yes**, but **0–80 °C** | **Yes** | ~$20–40 | Very low | **80 °C** | Presence gate only. 8×8 is not an image. Cannot localise into an RGB box. Fine as a cheap always-on tripwire, never as the verdict |
| **5** | **Seek Compact family** | **No** (open path); OEM cores UNVERIFIED | **UNVERIFIED** | ~$200–400 | High — non-UVC, libusb | n/a | **Avoid.** Not UVC, no open temperature mapping, arm64 unconfirmed. Highest effort for the weakest guarantee |
| **6** | **GigE / Ethernet** (FLIR A400/Ax5, Optris) | **Yes** — best in survey | Partial / UNVERIFIED | $3,000–10,000+ | High — GenICam, PoE, the GbE port | 900–1500 °C | Right data, wrong machine. Revisit if the platform grows |
| **7** | **Unbranded USB "thermal" dongles** | **No** | — | ~$50–150 | — | — | **Do not.** Picture-only. Buying one is how §1.2 happens |

**Suggested first purchase: one InfiRay-family USB camera (#1) and one
MLX90640 (#2).** Together they cost less than a single PureThermal+Lepton 3.5,
they occupy different buses, and they fail differently — which is the only kind
of redundancy worth having. If exactly one device can be bought, buy #1.

---

## 10. What this actually means for `detect_fire()`

The corroboration contract that follows from all of the above:

1. **The thermal node publishes numbers, not pictures.** Minimum useful message:
   `max_c`, `mean_c`, the hottest region's bearing and angular extent, a
   `radiometric: true/false` flag, and a `frame_valid` flag that is false during
   FFC/shutter events.
2. **`radiometric: false` must disqualify the device from corroboration
   entirely**, at the message level, not by convention. If a picture-only camera
   is plugged in, the correct behaviour is for corroboration to become
   *unavailable*, not to become *easy*.
3. **The discriminating threshold is large, and should be stated as such.** A
   sunlit orange cone plausibly reaches 50–60 °C (UNVERIFIED). A flame or ember
   bed is 400 °C+. Any threshold placed in the 100–200 °C band separates them
   with enormous margin — far more margin than the ±5 °C class accuracy of any
   device here, and more than enough to absorb the uncertainty in the InfiRay
   family's community-derived conversion formula. **This is why a
   reverse-engineered driver is acceptable for this job and would not be for
   metrology.**
4. **Check the ceiling before choosing the threshold.** A 150 °C threshold is
   unreachable on an AMG8833 (80 °C ceiling) and marginal on a high-gain Lepton
   (140 °C). The threshold and the device must be chosen together.
5. **Corroborate stopped, not while turning** (§5), and associate by timestamp
   and bearing with declared uncertainty, not by pixel overlay (§7).
6. **Assert the raw format at startup** (§6) and refuse to run degraded.

---

## 11. Open questions this survey could not close

Each of these needs hardware or a vendor answer, and each could change a
recommendation:

- **UNVERIFIED:** the exact InfiRay/Topdon counts→°C conversion, and whether the
  `/64 − 273.15` form is complete or needs the metadata-borne environmental
  correction. Sources emphasise this differently (§2.2).
- **UNVERIFIED:** whether TC001, TC001 Plus and TC002 all expose the raw half.
- **UNVERIFIED:** Seek SDK arm64 availability (developer portal returned 403).
- **UNVERIFIED:** Optris `libirimager` ARM availability.
- **UNVERIFIED:** Spinnaker on Ubuntu 22.04 arm64 / RK3588S.
- **UNVERIFIED:** whether PureThermal firmware exposes Lepton gain-mode
  switching over UVC, which decides whether a Lepton saturates on a real fire.
- **UNVERIFIED:** reliable I2C clock rate on the RK3588S 40-pin header, which
  sets the MLX90640 frame rate at 8 vs 32 FPS.
- **UNVERIFIED:** the RGB webcam's actual FOV, which every alignment number in
  §7 depends on. Measure this first — it costs nothing and gates the rest.
