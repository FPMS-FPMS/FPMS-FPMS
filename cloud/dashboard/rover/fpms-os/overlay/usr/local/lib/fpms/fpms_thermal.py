"""fpms_thermal - device-abstraction layer for ANY thermal camera on FPMS.

Pure functions and frozen descriptors. NO cv2, NO ROS, NO serial, NO clock, NO
globals, NO file I/O. Frames arrive as bytes or numpy arrays and values come
back. That is the same contract fpms_motion.py and fpms_scanmatch.py already
keep in this directory, and it is the only reason any of this is testable when
NO THERMAL CAMERA HAS EVER BEEN CONNECTED TO THIS ROVER - which is the state
today and the state every default in this file assumes.

WHY THIS EXISTS
===============
`fpms_rover_agent.py` screens for fire with a colour heuristic (`detect_fire`,
HSV: bright, saturated, red/orange). Its own comment is exact about the status
of that screen:

    "deliberately a *screen*, not a verdict - a hit raises an event that the
     thermal sensor and the cloud VLM then corroborate."

docs/THERMAL_HARDWARE.md puts the problem in one line: an orange traffic cone
and a fire are the same colour and roughly 400 degC apart. Only a RADIOMETRIC
camera sees the second difference. So the thermal camera's entire job here is
CORROBORATION, and this module exists to make sure that corroboration is real,
because there are several ways to make it fake and every one of them is silent.

    THE FIVE THINGS THIS FILE IS BUILT AROUND
    =========================================

1 - NEVER BIND A CAMERA BY INTEGER INDEX.
-----------------------------------------
`fpms_rover_agent.py:474` does `cv2.VideoCapture(idx)` from
`FPMS_CAMERA_INDEX=0`, then scans /dev/video* for any node that yields a frame.
With ONE camera that is merely fragile; it has already failed once in the field
when a USB re-enumeration (error -71) turned /dev/video0 into /dev/video1
mid-run. With TWO cameras it is a correctness bug with no error path at all:

  * thermal enumerates first -> the fire detector hunts for flame HUE in a
    greyscale or false-colour thermal image. `detect_fire` does not raise. It
    returns `(False, 0.0, [])` forever, on a rover whose product thesis is
    fire detection.
  * rgb enumerates into the thermal slot -> colour webcam bytes are read as
    16-bit counts and every "temperature" is a reinterpretation of packed YUYV.
    Those numbers are plausible, stable, and entirely fictional.

This project has already been bitten by exactly this class of ambiguity: BOTH
onboard CP2102 serial adapters report an identical `ID_SERIAL` of "0001", so
/dev/serial/by-id can only ever name one of the pair and it is a coin flip
which. 99-fpms-serial.rules therefore keys on USB TOPOLOGY (`KERNELS==`), and
`fpms_duty_driver.validate_port()` actively REFUSES any path that is not under
/dev/serial/by-path/. The cameras get the same treatment: two distinct
persistent symlinks (`FPMS_CAM_RGB` / `FPMS_CAM_THERMAL`), created by
98-fpms-thermal.rules, and `require_distinct_bindings()` here refuses an index,
a bare /dev/videoN, or the same path used twice.

2 - RADIOMETRIC vs NOT IS IN THE TYPE SYSTEM, AND IT IS NOT A BOOLEAN.
----------------------------------------------------------------------
Most cheap "thermal cameras" emit only an AGC'd image. AGC rescales the frame
to the CURRENT SCENE every frame; the scaling parameters are discarded before
the pixel leaves the device. There is no temperature in that image and none can
be recovered. Worse, the survey's §1.2 shows the two channels are not even
independent: a sunlit orange cone at 50-60 degC is mapped to the top of the AGC
palette against grass, so the thermal image AGREES with the colour detection,
on the wrong object, for a physically real reason. Two-sensor agreement,
manufactured.

    A device that cannot measure temperature must be STRUCTURALLY INCAPABLE of
    reporting one. Not documented as meaningless. Incapable.

AND IT IS A TRI-STATE, NOT A BOOLEAN, because of the finding that breaks the
obvious design: FLIR Lepton 3.5 and 2.5 are radiometric; 3.0 and 2.0 are
picture-only - and on a PureThermal carrier ALL FOUR PRESENT THE SAME USB
IDENTITY and look identical on the bench. A `thermal_devices.json` entry keyed
on VID:PID that asserts radiometry is therefore a claim the OS cannot verify,
on exactly the device class where being wrong manufactures false confidence.
So `ThermalDevice.radiometry` is one of:

    RADIOMETRY_NO       - proven picture-only. Never measures.
    RADIOMETRY_POSSIBLE - the hardware MIGHT measure; the identity cannot say.
    RADIOMETRY_YES      - the identity alone establishes it.

    *** A `possible` DEVICE BEHAVES EXACTLY LIKE A `no` DEVICE UNTIL PROVEN. ***

Proof is a `RadiometryAssertion`: either an operator naming the variant they
physically read off the module, or `evaluate_radiometry_probe()` checking the
counts against a known reference scene. Absent that, `normalise()` hands back
an `AgcFrame` - the same object a picture-only camera produces - and says so.

The enforcement layers, so defeating this takes deliberate work not a typo:

  L1. TWO FRAME TYPES, NOT ONE FLAG. `AgcFrame` HAS NO `temperature_at`, no
      `max_celsius`, no `hotspot`. There is no method to call and no flag to
      get wrong. `hasattr(frame, "temperature_at")` is False.
  L2. THE CONVERSION IS THE CAPABILITY. `RadiometricFrame` cannot be built
      without a `RadiometricConversion`. There is no default conversion.
  L3. A `no` DEVICE MAY NOT CARRY A CONVERSION. Refused as hard as a `yes`
      device without one. That is the rule that stops somebody attaching a
      plausible slope to an AGC camera and calling it corroboration.
  L4. THE PIXEL FORMAT MUST AGREE. A YUYV luma plane and an 8-bit GREY frame
      have both been through AGC by definition, so radiometry on either is
      refused. BIT DEPTH IS NOT RADIOMETRICITY EITHER: the Seek Compact
      streams 16-bit frames that are uncalibrated counts (libseek-thermal says
      so itself), so it is declared `no` despite being 16-bit.
  L5. THE ASSERTION GATE. `possible` yields an `AgcFrame` until asserted.

3 - ONE MISSING CAPTURE FLAG SILENTLY DESTROYS ALL TEMPERATURE DATA.
---------------------------------------------------------------------
Without `cv2.CAP_PROP_CONVERT_RGB = 0`, OpenCV/libv4l helpfully converts the
stream and `read()` returns a 3-channel uint8 array. The upper bits are gone.
The image looks perfect. No error is raised anywhere, every downstream stage
keeps working, and every temperature is wrong. (docs/THERMAL_HARDWARE.md §6.)

`require_raw_frame_form()` is the single assertion that kills the whole class:
a 3-dimensional frame, or an 8-bit frame from a device that declares a 16-bit
raw form, is REFUSED with that flag named in the message. Call it immediately
after `read()`, before anything else touches the frame. `capture_contract()`
exists so the capture layer can be built from the descriptor rather than from
memory - it carries the fourcc, the geometry, the mandatory CONVERT_RGB=0, and
the vendor "magic" controls (the InfiRay/HT-301 family hijacks
`CAP_PROP_ZOOM`: 0x8004 unlocks raw 16-bit, 0x8000 fires the shutter).

4 - THE MEASUREMENT CEILING IS A REQUIRED FIELD, AND IT OUTRANKS RESOLUTION.
----------------------------------------------------------------------------
An AMG8833 saturates at 80 degC. A Lepton left in default HIGH GAIN saturates
at 140 degC (low gain reaches 450; whether PureThermal exposes gain switching
over UVC is UNVERIFIED). A fire is 400 degC+. A sensor that saturates below
fire temperature reports "hot" and cannot say how hot - and a saturated field
looks IDENTICAL to a merely-warm one sitting at the top of its range.

So `max_measurable_c` is required on every conversion, and a reading at or
above it reports SATURATED with `celsius = None`. Never a number. `celsius`
being None rather than a clamped ceiling value is the point: a clamped 80.0
flows through a threshold comparison and a JSON payload without a murmur.
`check_threshold()` refuses a corroboration threshold the device cannot reach -
a 150 degC threshold is unreachable on an AMG8833 and marginal on a high-gain
Lepton, and choosing threshold and device together is survey §10.4.

5 - ONE FRAME CAN CONTAIN SEVERAL CO-REGISTERED PLANES PLUS METADATA.
----------------------------------------------------------------------
The most likely device to actually be attached (survey ranks it #1) is the
InfiRay/Xtherm family. The P2 Pro delivers a 256x384 YUY2 frame in which the
TOP HALF is the AGC picture and the BOTTOM HALF is the raw 16-bit data,
perfectly registered, at 25 Hz. The HT-301/T2S+ path adds FOUR METADATA ROWS
below the image carrying per-device calibration coefficients (including
emissivity terms). So the frame model here is a SPLITTER (`FrameLayout`), not
a format tag: named `agc`, `raw` and `metadata` slices, each validated against
the delivered geometry. Without that split, half the reported temperatures
would be reinterpreted AGC pixels - numbers with the right shape, the right
dtype and no meaning.

MISSING NUMBERS ARE REFUSALS, NOT DEFAULTS
==========================================
Same rule as fpms_motion.Calibration, for the same reason.

  * No default emissivity. `EmissivityCompensation` requires `emissivity` and
    `ambient_c` explicitly. 0.95 is what everyone assumes; on bare aluminium
    (~0.05-0.10) it reads a surface HUNDREDS of degrees wrong, in the direction
    that invents a fire. The selftest asserts that magnitude.
  * No default counts->kelvin. The Lepton's TLinear scale (0.01 vs 0.1 K/count)
    is a runtime register invisible to USB, so the database carries both and
    `resolve()` REFUSES an ambiguous match rather than picking one - a silent
    wrong pick is a factor of ten on every temperature.
  * Some conversions cannot be done here at all. The HT-301/T2S+ maths is a
    device-specific lookup over the metadata rows plus a Stefan-Boltzmann
    computation with distance and atmospheric terms. That is expressible
    (`CONVERSION_DEVICE_METADATA`) and it REFUSES rather than approximating,
    because a linear slope smuggled in beside "the real conversion lives in the
    metadata" is exactly the dishonest descriptor L3 exists to stop.
  * Every shipped entry is UNVERIFIED and every conversion carries
    `verified: false`. Every `TemperatureReading` carries that outward, so a
    consumer that publishes an unverified number as fact does so knowingly.

WHAT THIS MODULE IS NOT
=======================
  * NOT a ROS node and NOT a capture loop. It opens nothing. The transport,
    the systemd unit and the topic are deliberately somebody else's file;
    `capture_contract()` is how this file talks to that one.
  * NOT detection. `hotspot()` reports what the pixels say; whether that is a
    fire is a decision for the layer that also holds the RGB detection, the
    range, the FFC/validity flag and the mission state. Survey §5 adds the
    operational rule this module cannot enforce: corroborate STOPPED. At
    1 rad/s yaw an 8.7 Hz Lepton smears 6.6 deg between frames.
  * NOT an aligner. Survey §7: do not fuse at pixel level in v1. Publish a
    region summary in the thermal camera's own frame and associate by bearing
    with declared uncertainty.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

__all__ = [
    # refusals
    "ThermalRefusal",
    "REFUSE_DESCRIPTOR", "REFUSE_NOT_RADIOMETRIC", "REFUSE_UNASSERTED",
    "REFUSE_NO_CONVERSION", "REFUSE_CONVERSION_NOT_HERE",
    "REFUSE_UNKNOWN_DEVICE", "REFUSE_AMBIGUOUS_DEVICE", "REFUSE_FRAME_SIZE",
    "REFUSE_FRAME_FORM", "REFUSE_COMPENSATION", "REFUSE_UNPHYSICAL",
    "REFUSE_BINDING", "REFUSE_CEILING", "REFUSE_NUMPY",
    # radiometry tri-state
    "RADIOMETRY_YES", "RADIOMETRY_POSSIBLE", "RADIOMETRY_NO", "RADIOMETRY_STATES",
    "RadiometryAssertion", "evaluate_radiometry_probe",
    # transports and formats
    "TRANSPORT_UVC", "TRANSPORT_V4L2_RAW", "TRANSPORT_LIBUSB", "TRANSPORT_I2C",
    "TRANSPORT_SPI", "TRANSPORT_NET", "TRANSPORTS",
    "FMT_Y16", "FMT_GREY", "FMT_YUYV", "FMT_YUYV_RAW16", "FMT_RAW_COUNTS",
    "PIXEL_FORMATS", "BYTES_PER_PIXEL", "AGC_ONLY_FORMATS", "SIXTEEN_BIT_FORMATS",
    # descriptors
    "CONVERSION_LINEAR", "CONVERSION_DEVICE_METADATA",
    "RadiometricConversion", "PlaneSlice", "FrameLayout", "ControlHint",
    "ThermalDevice", "Capabilities", "capabilities",
    "CaptureContract", "capture_contract", "require_raw_frame_form",
    # database
    "DeviceDatabase", "load_device_database", "resolve",
    # compensation and conversion
    "EmissivityCompensation", "TemperatureReading",
    "counts_to_apparent_kelvin", "counts_to_celsius",
    "KELVIN_AT_ZERO_C",
    # thresholds
    "FLAME_FLOOR_C", "SUNLIT_CONE_MAX_C", "DISCRIMINATION_BAND_C",
    "check_threshold",
    # frames
    "AgcFrame", "RadiometricFrame", "HotspotReport", "normalise",
    # display only
    "DisplayOnlyImage", "colourise_for_display",
    # binding
    "FPMS_CAM_RGB", "FPMS_CAM_THERMAL", "require_distinct_bindings",
]


# ===========================================================================
# SECTION 0 -- CONSTANTS THAT ARE NOT MEASUREMENTS
#
# Definitions, policy, and paths we choose. Nothing here is a measurement of
# a thermal camera; those all live in thermal_devices.json and every one of
# them is UNVERIFIED today.
# ===========================================================================

KELVIN_AT_ZERO_C = 273.15
"""Definition, not a calibration. Named because a stray 273 (or 272.15) buried
in an expression is a 0.15-1.15 K bias nothing downstream can distinguish from
a sensor offset."""

FPMS_CAM_RGB = "/dev/fpms-cam-rgb"
FPMS_CAM_THERMAL = "/dev/fpms-cam-thermal"
"""The two persistent symlinks created by 98-fpms-thermal.rules.

DISTINCT NAMES ARE THE POINT. The pre-existing `/dev/fpms-cam` (from
99-fpms-serial.rules) matches ANY USB video4linux capture node, so with a
second camera present it lands on whichever enumerated last - it is an index
bug wearing a stable-looking name. Consumers must move to these two."""

FLAME_FLOOR_C = 400.0
"""A flame or ember bed is 400 degC+. FROM-DOCS (survey §10.3). Used only to
report whether a device's ceiling can see a fire at all."""

SUNLIT_CONE_MAX_C = 60.0
"""A sunlit orange traffic cone plausibly reaches 50-60 degC. UNVERIFIED in the
survey and believed rather than measured - which is fine, because the whole
argument only needs it to be far below FLAME_FLOOR_C."""

DISCRIMINATION_BAND_C = (100.0, 200.0)
"""Any corroboration threshold in this band separates a sunlit cone from a
flame with enormous margin - far more than the +/-5 degC class accuracy of any
device in the survey, and more than enough to absorb the uncertainty in the
InfiRay family's community-derived conversion.

THAT MARGIN IS WHY A REVERSE-ENGINEERED DRIVER IS ACCEPTABLE FOR THIS JOB and
would not be for metrology. It is not a licence to stop caring about the
conversion; it is the reason an unverified one is still worth having."""


# -- transports --------------------------------------------------------------

TRANSPORT_UVC = "uvc"
"""USB Video Class - /dev/videoN via V4L2 or cv2. PureThermal, InfiRay family."""

TRANSPORT_V4L2_RAW = "v4l2_raw"
"""A V4L2 node whose payload is not a UVC-negotiated image."""

TRANSPORT_LIBUSB = "libusb"
"""A proprietary USB protocol with NO /dev/video node at all. The Seek Compact
family is here: it is not UVC and needs libseek-thermal or the vendor SDK. Kept
as a distinct transport so a descriptor cannot imply a video node that will
never exist, and so the udev rule for it is written as a permissions rule
rather than a symlink rule."""

TRANSPORT_I2C = "i2c"
"""MLX90640/90641, AMG8833. NO USB VID:PID EXISTS, so udev CANNOT bind these
and the symlink scheme does not apply: the binding is (bus, address). Recorded
here so nobody writes a udev rule that can never match."""

TRANSPORT_SPI = "spi"
"""A bare Lepton on the header speaks VoSPI. Expressible, and the survey's §3.4
verdict is DO NOT: the host must pull data within three line times or the
sensor loses sync, which is not achievable from non-realtime userspace. The
PureThermal board's STM32 exists precisely to solve this."""

TRANSPORT_NET = "net"
"""GigE Vision/GenICam or RTSP. Expressible; survey §4 says wrong machine."""

TRANSPORTS = (TRANSPORT_UVC, TRANSPORT_V4L2_RAW, TRANSPORT_LIBUSB,
              TRANSPORT_I2C, TRANSPORT_SPI, TRANSPORT_NET)


# -- pixel formats -----------------------------------------------------------

FMT_Y16 = "Y16"
"""16 bpp unsigned, endianness per the descriptor. The standard UVC format for
raw counts (Y16 / GRAY16_LE / V4L2_PIX_FMT_Y16)."""

FMT_GREY = "GREY"
"""8 bpp. AGC OUTPUT BY DEFINITION - 256 levels cannot span a useful range at a
useful resolution, and every device emitting it has already rescaled."""

FMT_YUYV = "YUYV"
"""4:2:2 packed; the luma plane is the image. ALSO AGC BY DEFINITION: a camera
that produced a YUV image already chose a radiance-to-brightness mapping."""

FMT_YUYV_RAW16 = "YUYV_RAW16"
"""A stream the driver ANNOUNCES as YUYV/YUY2 whose bytes are really packed
16-bit data. Not hypothetical: the InfiRay P2 Pro enumerates as 256x384 YUY2
where the top half is an AGC picture and the bottom half is raw uint16, and
the reference driver disables OpenCV's YUV->RGB conversion, splits the buffer
by length and reinterprets the lower half. A device using this format MUST
carry a `FrameLayout` naming which rows are which."""

FMT_RAW_COUNTS = "RAW_COUNTS"
"""The caller supplies an already-shaped numeric sequence, not a byte buffer.
This is what an I2C driver produces: the Melexis reference driver reads the
device EEPROM and computes per-pixel object temperature itself, so the values
reaching this module are already in the driver's units. That indirection is
declared in `conversion.source` and `emissivity_applied_upstream`, never
assumed."""

PIXEL_FORMATS = (FMT_Y16, FMT_GREY, FMT_YUYV, FMT_YUYV_RAW16, FMT_RAW_COUNTS)

BYTES_PER_PIXEL: Dict[str, Optional[int]] = {
    FMT_Y16: 2, FMT_GREY: 1, FMT_YUYV: 2, FMT_YUYV_RAW16: 2,
    FMT_RAW_COUNTS: None,
}

AGC_ONLY_FORMATS = (FMT_GREY, FMT_YUYV)
"""Formats that CANNOT be radiometric, whatever a descriptor claims (L4).

The cheapest enforcement layer and the one most likely to catch a real
mistake: somebody adds a camera and writes radiometry "yes" because the box
said thermal, and the format they also wrote down proves otherwise."""

SIXTEEN_BIT_FORMATS = (FMT_Y16, FMT_YUYV_RAW16)
"""Formats whose delivered buffer must be reinterpretable as uint16.

`require_raw_frame_form()` uses this for the CONVERT_RGB guard: a device
declaring one of these that hands back 8-bit or 3-channel data has been
silently converted by libv4l and its upper bits are gone."""


# ===========================================================================
# SECTION 1 -- REFUSALS
#
# Named, comparable, carrying the reason in the message - exactly as
# fpms_motion.MotionRefusal does. An operator should read the string and know
# both what happened and what to do about it.
# ===========================================================================

class ThermalRefusal(Exception):
    """This module will not produce a value, and here is exactly why.

    A refusal is a SUCCESSFUL outcome of the design. The alternative - naming a
    temperature that came from an AGC image, from an unproven Lepton variant,
    from a saturated pixel, or from a conversion nobody measured - is a
    confidently wrong number attached to a fire alert.
    """

    def __init__(self, code: str, detail: str = ""):
        self.code = code
        self.detail = detail
        super().__init__(f"{code}: {detail}" if detail else code)


REFUSE_DESCRIPTOR = "the device descriptor is not self-consistent"
REFUSE_NOT_RADIOMETRIC = "this device cannot measure temperature"
REFUSE_UNASSERTED = "this device MIGHT measure, and nobody has proven it does"
REFUSE_NO_CONVERSION = "no counts-to-kelvin conversion"
REFUSE_CONVERSION_NOT_HERE = "this device's conversion cannot be done in this module"
REFUSE_UNKNOWN_DEVICE = "no such device in the database"
REFUSE_AMBIGUOUS_DEVICE = "more than one database entry matches"
REFUSE_FRAME_SIZE = "the frame is not the size this device produces"
REFUSE_FRAME_FORM = "the frame is not the raw form this device produces"
REFUSE_COMPENSATION = "emissivity and ambient must be supplied, not assumed"
REFUSE_UNPHYSICAL = "the compensated radiance is not physical"
REFUSE_CEILING = "the threshold is at or above this sensor's ceiling"
REFUSE_BINDING = "camera bindings are not distinct and stable"
REFUSE_NUMPY = "numpy is required to normalise a frame"


# ===========================================================================
# SECTION 2 -- NUMPY, OPTIONALLY
#
# Descriptors, the database, capabilities, the capture contract and scalar
# conversions need no numpy, so this file imports on a Windows dev machine
# with nothing installed - the same requirement fpms_motion states for
# itself. Only frame work needs arrays, and it refuses clearly rather than
# raising an ImportError from somewhere unhelpful.
# ===========================================================================

try:                                            # pragma: no cover - trivial
    import numpy as _np
except Exception:                               # pragma: no cover
    _np = None


def _require_numpy():
    if _np is None:
        raise ThermalRefusal(
            REFUSE_NUMPY,
            "frame work needs numpy (1.21.5 is what the rover has). "
            "Descriptors, the device database, the capture contract and "
            "scalar conversions all work without it, which is why the import "
            "is optional here.")
    return _np


def _is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _is_array(x) -> bool:
    return _np is not None and isinstance(x, _np.ndarray)


def _any_nonpositive(x) -> bool:
    if _is_array(x):
        return bool((x <= 0.0).any())
    try:
        return bool(x <= 0.0)
    except Exception:                            # pragma: no cover
        return False


def _any_at_or_above(x, limit: float) -> bool:
    if _is_array(x):
        return bool((x >= limit).any())
    return bool(x >= limit)


# ===========================================================================
# SECTION 3 -- RADIOMETRY: A TRI-STATE AND ITS PROOF
#
# THE FINDING THAT BREAKS THE OBVIOUS DESIGN. Lepton 3.5/2.5 are radiometric;
# 3.0/2.0 are picture-only; on a PureThermal carrier all four present the SAME
# USB identity. So a JSON entry keyed on VID:PID cannot assert radiometry, on
# exactly the device class where being wrong manufactures false confidence.
# ===========================================================================

RADIOMETRY_YES = "yes"
"""The device identity alone establishes that temperature is recoverable.
Reserved for devices with no picture-only variant sharing their identity - the
MLX90640 and AMG8833 (a documented per-pixel temperature output), and the
InfiRay P2 Pro (whose raw half is read directly by a driver whose source the
survey read)."""

RADIOMETRY_POSSIBLE = "possible"
"""The hardware might measure and THE IDENTITY CANNOT SAY.

    A `possible` DEVICE IS TREATED EXACTLY AS `no` UNTIL PROVEN.

Not "assume the better variant, warn in a log". `normalise()` returns an
`AgcFrame` - the same object a picture-only camera produces, with no
temperature method on it - and records why in `withheld_reason`. This is the
state of every Lepton-on-PureThermal entry, and of the TOPDON TC001 (whether
TC001/TC001 Plus/TC002 all expose the raw half is UNVERIFIED per SKU), and of
the HT-301/T2S+ (whose raw mode has to be unlocked by a magic control that may
simply not take)."""

RADIOMETRY_NO = "no"
"""Proven picture-only. Includes 16-bit devices: the Seek Compact streams
16 bits that libseek-thermal states plainly are not convertible to temperature
because the mapping is unknown. BIT DEPTH IS NOT RADIOMETRICITY."""

RADIOMETRY_STATES = (RADIOMETRY_YES, RADIOMETRY_POSSIBLE, RADIOMETRY_NO)


@dataclass(frozen=True)
class RadiometryAssertion:
    """Proof that a `possible` device really does measure. Frozen evidence.

    THIS IS NOT A CONFIG FLAG. It carries WHO said so, HOW, and WHAT THE
    EVIDENCE WAS, and it is bound to one `device_key` so an assertion made
    about a Lepton cannot silently authorise an InfiRay entry. A bare boolean
    in config.env would be indistinguishable from a guess six months later,
    which on this rover is how CMD_SCALE = 6.1 survived as a "calibration".
    """

    device_key: str
    confirmed: bool
    method: str
    evidence: str
    asserted_by: str

    def __post_init__(self):
        for name in ("device_key", "method", "evidence", "asserted_by"):
            v = getattr(self, name)
            if not isinstance(v, str) or not v.strip():
                raise ThermalRefusal(
                    REFUSE_UNASSERTED,
                    f"a radiometry assertion needs a non-empty {name}. An "
                    "unattributed assertion is a guess with a timestamp.")

    @classmethod
    def from_operator(cls, device_key: str, variant: str, evidence: str,
                      asserted_by: str) -> "RadiometryAssertion":
        """An operator physically identified the variant.

        `variant` must be a model that IS radiometric. "Lepton 3.5" and
        "Lepton 2.5" are; "Lepton 3.0" and "Lepton 2.0" are not, and asserting
        one of those returns a NEGATIVE assertion rather than raising - an
        operator who correctly identifies a 3.0 has done the right thing and
        should get "confirmed: picture-only", not an error.

        `evidence` must name what was actually read: a part number off the
        module, an order line, a vendor invoice. "I think it's the good one"
        is not evidence and the emptiness check will not catch it, but the
        string is stored and shown, which is the next best thing.
        """
        v = (variant or "").strip().lower().replace(" ", "")
        radiometric_variants = ("lepton3.5", "lepton35", "lepton2.5",
                                "lepton25", "3.5", "2.5")
        picture_only_variants = ("lepton3.0", "lepton30", "lepton2.0",
                                 "lepton20", "3.0", "2.0")
        if v in radiometric_variants:
            confirmed = True
        elif v in picture_only_variants:
            confirmed = False
        else:
            raise ThermalRefusal(
                REFUSE_UNASSERTED,
                f"variant {variant!r} is not one this module knows to be "
                "radiometric or picture-only. Do not assert a variant you "
                "cannot name from the hardware: for the Lepton the whole "
                "problem is that 3.5 and 3.0 are indistinguishable except by "
                "the label. Use evaluate_radiometry_probe() against a known "
                "reference scene instead.")
        return cls(device_key=device_key, confirmed=confirmed,
                   method="operator identified the physical variant",
                   evidence=f"variant={variant}; {evidence}",
                   asserted_by=asserted_by)

    def authorises(self, device: "ThermalDevice") -> bool:
        return self.confirmed and self.device_key == device.key


def evaluate_radiometry_probe(device: "ThermalDevice",
                              observed_counts: float,
                              known_scene_c: float,
                              tolerance_c: float,
                              asserted_by: str) -> RadiometryAssertion:
    """Decide whether observed counts are consistent with this device's
    declared conversion, against a scene of KNOWN temperature. Pure.

    THE PHYSICS THAT MAKES THIS WORK. A radiometric Lepton in TLinear reports
    absolute kelvin at 0.01 K/count, so a 20 degC room is ~29315 counts. A
    picture-only Lepton 3.0 reports 14-bit scene FLUX with no fixed relation
    to temperature - typically some thousands of counts, and crucially NOT
    near 29315 and NOT stable as the room changes. One reading against a
    known reference therefore separates them with a huge margin. It is the
    same measurement that would later justify `verified: true` on the
    conversion, though this function deliberately does NOT set that: one
    reading proves the variant, and a calibration claim needs more than one.

    `known_scene_c` must be a real reference filling the field of view - an
    ice bath, boiling water, a large surface with a contact thermometer on
    it. "The room feels about 20" is not a reference; the tolerance is there
    to absorb sensor accuracy (+/-5 degC class), not the operator's guess.

    A NEGATIVE RESULT IS A RESULT. If the implied temperature is far off, the
    returned assertion has `confirmed=False` and carries the implied number,
    which is exactly what an operator needs to see to conclude "this is a
    3.0". It does not raise.
    """
    conv = device.conversion
    if conv is None:
        raise ThermalRefusal(
            REFUSE_NO_CONVERSION,
            f"{device.key} declares no conversion, so there is nothing for a "
            "probe to test. A device with no conversion is picture-only by "
            "construction.")
    if conv.kind != CONVERSION_LINEAR:
        raise ThermalRefusal(
            REFUSE_CONVERSION_NOT_HERE,
            f"{device.key} uses a {conv.kind} conversion, which this module "
            "cannot evaluate. Probe it through whatever decodes the device "
            "metadata, then record the outcome as an operator assertion.")
    if tolerance_c <= 0:
        raise ThermalRefusal(REFUSE_UNASSERTED,
                             "tolerance_c must be positive")

    implied_c = conv.apparent_kelvin(float(observed_counts)) - KELVIN_AT_ZERO_C
    delta = implied_c - known_scene_c
    confirmed = abs(delta) <= tolerance_c
    return RadiometryAssertion(
        device_key=device.key,
        confirmed=confirmed,
        method="known-reference probe",
        evidence=(f"{observed_counts:g} counts imply {implied_c:.2f} degC "
                  f"against a known {known_scene_c:g} degC reference "
                  f"(delta {delta:+.2f} degC, tolerance {tolerance_c:g}). "
                  + ("CONSISTENT with the declared conversion."
                     if confirmed else
                     "NOT consistent: this is what a picture-only variant "
                     "looks like, or the wrong TLinear scale, or the wrong "
                     "database entry. Do not report temperatures.")),
        asserted_by=asserted_by)


# ===========================================================================
# SECTION 4 -- THE CONVERSION
#
# THE OBJECT THAT IS THE CAPABILITY (L2). Holding one is what it means to be
# able to measure. There is no default instance and no way to synthesise one
# from a device that has none.
# ===========================================================================

CONVERSION_LINEAR = "linear"
"""kelvin = counts * kelvin_per_count + kelvin_at_zero_counts.

LINEAR ONLY WHERE LINEAR IS TRUE. FLIR specifies TLinear as linear in counts,
which is why the shape fits; that is a property of TLinear, not of thermal
cameras. A device whose relation is not linear does not get a fudged slope."""

CONVERSION_DEVICE_METADATA = "device_metadata"
"""The conversion coefficients arrive IN THE FRAME and this module cannot do it.

The HT-301/T2S+ family appends four metadata rows carrying per-device
calibration coefficients; temperature comes from a device-specific lookup plus
a Stefan-Boltzmann-form computation with distance, atmospheric and emissivity
terms and a range-correction pair. That is a decoder, not a slope.

DECLARING THIS IS THE HONEST OPTION AND IT REFUSES. The dishonest option is a
linear approximation sitting beside a comment saying the real conversion is
elsewhere - which is why a `device_metadata` conversion carrying a slope is
REFUSED outright. The frame still hands out `metadata_rows` so whoever writes
the decoder has the bytes."""

CONVERSION_KINDS = (CONVERSION_LINEAR, CONVERSION_DEVICE_METADATA)


@dataclass(frozen=True)
class RadiometricConversion:
    """counts -> kelvin, PARAMETERISED, with provenance and a CEILING.

    WHAT `kelvin_per_count` IS NOT
    ------------------------------
    It is NOT a constant of the Lepton. TLinear resolution is a RUNTIME
    REGISTER: 0.01 K/count or 0.1 K/count, and nothing in the USB descriptor
    reports which. The database carries both and `resolve()` refuses to
    choose; picking wrong scales every reading by ten.
    """

    kind: str
    source: str
    """WHERE THIS NUMBER CAME FROM, in words. Not optional. A slope with no
    provenance is indistinguishable from a slope somebody remembered."""

    max_measurable_c: float
    """THE CEILING, AND IT IS REQUIRED - survey finding 3.

    AMG8833 saturates at 80 degC; a Lepton in default high gain at 140; a
    flame is 400+. A sensor that saturates below fire temperature reports
    "hot" and cannot say how hot, and A SATURATED FIELD LOOKS IDENTICAL TO A
    MERELY-WARM ONE AT THE TOP OF ITS RANGE. Readings at or above this report
    SATURATED with `celsius=None`, never a clamped number - a clamped 80.0
    flows through a threshold comparison and a JSON payload without a
    murmur."""

    kelvin_per_count: Optional[float] = None
    kelvin_at_zero_counts: Optional[float] = None
    min_measurable_c: Optional[float] = None

    verified: bool = False
    """True ONLY after this rover has read a known reference through this exact
    device in this exact mode. Every shipped entry is False. This flag rides
    outward on every `TemperatureReading` so a publisher cannot lose it."""

    emissivity_applied_upstream: bool = False
    """True when the DRIVER already applied an emissivity correction.

    TRUE FOR THE MLX90640: the Melexis reference driver computes per-pixel
    object temperature `To` taking emissivity and reflected ambient `Ta` as
    its own parameters, so the values reaching this module are already
    corrected. Applying `EmissivityCompensation` on top would correct twice -
    a silent, physically-shaped error that no plot distinguishes from a
    miscalibrated sensor. `counts_to_celsius` therefore REFUSES a non-unity
    emissivity on such a device and tells the caller to pass it to the driver
    instead."""

    count_min: Optional[int] = None
    count_max: Optional[int] = None
    """Valid raw range where documented. A SANITY GATE, not a clamp: all-zero
    and all-0xFFFF frames are what a mis-bound or disconnected sensor
    produces, and both convert to a plausible uniform field."""

    gain_mode: Optional[str] = None
    """Which gain/range mode this conversion and ceiling assume. The Lepton's
    high gain (~-10..140) is the default and saturates on a flame; low gain
    reaches ~450. Whether PureThermal exposes the switch over UVC is
    UNVERIFIED, so the two are separate database entries rather than a
    runtime setting this module pretends to control."""

    scene_dependent: bool = False
    """True when counts drift with the SENSOR's own temperature. True for
    uncooled microbolometers: a Lepton warming up under its own power reports
    a rising scene. Vendor accuracy applies only within a stated ambient band
    and after warm-up."""

    requires_shutter_sync: bool = False
    """True when the device periodically closes a shutter to re-reference
    itself (FFC/NUC). True for the Lepton (~3-minute timer plus on ambient
    change) and the InfiRay family. Frames captured DURING an FFC are of the
    SHUTTER - a uniform, plausible, wrong field. Lepton telemetry exposes FFC
    state so a driver can drop them; a camera that hides its shutter events
    will occasionally emit a whole frame of nonsense with no warning. This
    module cannot see the flag; it only insists the caller knows it must."""

    def __post_init__(self):
        if self.kind not in CONVERSION_KINDS:
            raise ThermalRefusal(REFUSE_DESCRIPTOR,
                                 f"conversion kind={self.kind!r} is not one "
                                 f"of {CONVERSION_KINDS}")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                "a conversion must say where its numbers came from. An "
                "unattributed slope cannot be checked, corrected, or blamed.")
        if not _is_number(self.max_measurable_c):
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                "max_measurable_c is REQUIRED on every conversion. A sensor "
                "with no stated ceiling cannot be given a corroboration "
                "threshold honestly, and its saturated frames are "
                "indistinguishable from merely-warm ones.")

        if self.kind == CONVERSION_LINEAR:
            if not _is_number(self.kelvin_per_count):
                raise ThermalRefusal(
                    REFUSE_NO_CONVERSION,
                    "a linear conversion needs kelvin_per_count. There is no "
                    "device-class default and none can be inferred.")
            if self.kelvin_per_count <= 0.0:
                # Zero maps the whole scene onto one plausible temperature;
                # negative reports fires as cold spots. Both look like data.
                raise ThermalRefusal(
                    REFUSE_DESCRIPTOR,
                    f"kelvin_per_count={self.kelvin_per_count!r} must be > 0.")
            if not _is_number(self.kelvin_at_zero_counts):
                raise ThermalRefusal(REFUSE_NO_CONVERSION,
                                     "a linear conversion needs "
                                     "kelvin_at_zero_counts")
        else:
            # THE ANTI-SMUGGLING RULE for device_metadata conversions.
            if (self.kelvin_per_count is not None
                    or self.kelvin_at_zero_counts is not None):
                raise ThermalRefusal(
                    REFUSE_DESCRIPTOR,
                    f"a {CONVERSION_DEVICE_METADATA} conversion must NOT carry "
                    "a linear slope. Declaring that the real coefficients live "
                    "in the frame metadata and then shipping an approximation "
                    "beside it means the approximation is what gets used, "
                    "silently, forever. Write the decoder or refuse.")

        if (self.min_measurable_c is not None
                and self.min_measurable_c >= self.max_measurable_c):
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"measurable range [{self.min_measurable_c}, "
                f"{self.max_measurable_c}] degC is empty or inverted")
        if (self.count_min is not None and self.count_max is not None
                and self.count_min >= self.count_max):
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"count range [{self.count_min}, {self.count_max}] is empty "
                "or inverted")

    # -- the maths -------------------------------------------------------
    def apparent_kelvin(self, counts):
        """counts -> APPARENT kelvin. Scalar or array; no numpy for scalars.

        APPARENT, and the name is the documentation: this is the temperature a
        perfect blackbody would need to be to emit what the sensor saw. A real
        surface with emissivity < 1 emits less AND reflects its surroundings,
        so its true temperature differs - see `EmissivityCompensation`.
        Nothing here hands out a surface temperature without being told what
        the surface is.
        """
        if self.kind != CONVERSION_LINEAR:
            raise ThermalRefusal(
                REFUSE_CONVERSION_NOT_HERE,
                f"this device's conversion is {self.kind}: its coefficients "
                "arrive in the frame's metadata rows and need a device-"
                "specific decoder (lookup + Stefan-Boltzmann with distance, "
                "atmospheric and emissivity terms). This module deliberately "
                "ships no approximation of it. Use `metadata_rows` on the "
                "frame and write the decoder, or treat the device as "
                "picture-only until one exists.")
        return counts * self.kelvin_per_count + self.kelvin_at_zero_counts

    def apparent_celsius(self, counts):
        return self.apparent_kelvin(counts) - KELVIN_AT_ZERO_C

    @property
    def survives_flame(self) -> bool:
        """True if the ceiling is above flame temperature, i.e. the device can
        rank two hot things rather than only reporting "at least this hot"."""
        return self.max_measurable_c >= FLAME_FLOOR_C

    @classmethod
    def from_dict(cls, d: dict) -> "RadiometricConversion":
        """Build from a `conversion` object in thermal_devices.json.

        Raises `ThermalRefusal` on anything missing; the loader catches that
        and REFUSES THE WHOLE ENTRY rather than dropping the conversion and
        keeping the device - a radiometric device with its conversion quietly
        removed is precisely the dishonest descriptor L3 forbids.
        """
        if not isinstance(d, dict):
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"conversion is {type(d).__name__}, not an object")
        for key in ("kind", "source", "max_measurable_c"):
            if key not in d:
                raise ThermalRefusal(
                    REFUSE_NO_CONVERSION,
                    f"conversion is missing {key!r}. There is no default for "
                    "it and none can be inferred from the device class.")
        def opt(name):
            v = d.get(name)
            return None if v is None else float(v)
        def opt_int(name):
            v = d.get(name)
            return None if v is None else int(v)
        return cls(kind=str(d["kind"]),
                   source=str(d["source"]),
                   max_measurable_c=float(d["max_measurable_c"]),
                   kelvin_per_count=opt("kelvin_per_count"),
                   kelvin_at_zero_counts=opt("kelvin_at_zero_counts"),
                   min_measurable_c=opt("min_measurable_c"),
                   verified=bool(d.get("verified", False)),
                   emissivity_applied_upstream=bool(
                       d.get("emissivity_applied_upstream", False)),
                   count_min=opt_int("count_min"),
                   count_max=opt_int("count_max"),
                   gain_mode=(None if d.get("gain_mode") is None
                              else str(d["gain_mode"])),
                   scene_dependent=bool(d.get("scene_dependent", False)),
                   requires_shutter_sync=bool(
                       d.get("requires_shutter_sync", False)))


def _conversion_caveats(conversion: RadiometricConversion) -> Tuple[str, ...]:
    out = []
    if not conversion.verified:
        out.append("UNVERIFIED conversion: this rover has never read a known "
                   "reference through this device. Treat the number as an "
                   "indication, not as evidence.")
    if not conversion.survives_flame:
        out.append(f"ceiling {conversion.max_measurable_c:g} degC is BELOW "
                   f"flame temperature ({FLAME_FLOOR_C:g} degC): this device "
                   "can say 'at least this hot' and cannot rank two hot "
                   "things or measure a fire.")
    if conversion.gain_mode:
        out.append(f"gain/range mode assumed: {conversion.gain_mode}")
    if conversion.scene_dependent:
        out.append("sensor is scene/self-heating dependent: readings drift "
                   "with the camera's own temperature, so a warm-up shifts "
                   "the whole field.")
    if conversion.requires_shutter_sync:
        out.append("device runs a periodic flat-field/shutter correction: a "
                   "frame captured during one is an image of the shutter, "
                   "uniform and plausible and wrong. The caller must carry a "
                   "frame-validity flag; this module cannot see it.")
    return tuple(out)


# ===========================================================================
# SECTION 5 -- EMISSIVITY AND AMBIENT
#
# BOTH REQUIRED, NEITHER DEFAULTED. The direct analogue of
# fpms_motion.Calibration: a number nobody supplied is refused, because the
# wrong one is silent and total.
# ===========================================================================

@dataclass(frozen=True)
class EmissivityCompensation:
    """The two scene facts that turn an apparent temperature into a surface one.

    WHY THERE IS NO DEFAULT EMISSIVITY
    ----------------------------------
    0.95 is the number every tutorial uses and it is right for painted wood,
    soil, skin, cloth and vegetation. It is catastrophically wrong for the
    other half of an arena: bare aluminium is ~0.05-0.10, polished steel
    ~0.1-0.2, galvanised sheet ~0.2-0.3. At an apparent 80 degC over a 25 degC
    room, assuming 0.95 for a surface that is really 0.10 shifts the answer by
    well over a hundred degrees, in the direction that INVENTS A FIRE. The
    selftest asserts that magnitude rather than describing it.

    A shiny surface is also a MIRROR in the thermal band: most of what the
    camera sees off it is the reflected surroundings. That is why `ambient_c`
    is required too, and why it must be the temperature of what the surface is
    REFLECTING - not the air, not the sensor housing.

    THE MODEL, AND WHAT IT IGNORES
    ------------------------------
        T_surface^4 = ( T_apparent^4 - (1 - e) * T_ambient^4 ) / e   [kelvin]

    ASSUMES: the reflected surroundings are one uniform temperature;
    atmospheric transmission 1.0 (true enough at rover ranges of metres, NOT
    across a field); no window/lens attenuation beyond what the vendor
    calibration absorbed; a diffuse (Lambertian) emitter. Every one is listed
    on the resulting `TemperatureReading`, so the assumptions travel with the
    number instead of living in this docstring.

    SURVEY §1.3 IS ALSO TRUE AND WORTH HOLDING ALONGSIDE: for fire-versus-cone
    the discrimination is hundreds of kelvin, so emissivity matters far less
    than it would for metrology. It is required anyway, because the number
    must be defensible when somebody asks how hot, not just whether hot.
    """

    emissivity: float
    ambient_c: float
    source: str = "not stated"
    """Where these two came from. The default says nothing flattering on
    purpose: a reading whose emissivity provenance is "not stated" should look
    exactly as weak as it is."""

    def __post_init__(self):
        if not _is_number(self.emissivity):
            raise ThermalRefusal(
                REFUSE_COMPENSATION,
                f"emissivity={self.emissivity!r} is not a number. It must be "
                "supplied for the surface actually being measured; there is "
                "no default and 0.95 is not a safe one.")
        if not (0.0 < self.emissivity <= 1.0):
            raise ThermalRefusal(
                REFUSE_COMPENSATION,
                f"emissivity={self.emissivity!r} is outside (0, 1]. Zero would "
                "divide by zero (a perfect mirror carries no information about "
                "its own temperature); above 1 is not a surface.")
        if not _is_number(self.ambient_c):
            raise ThermalRefusal(
                REFUSE_COMPENSATION,
                f"ambient_c={self.ambient_c!r} is not a number. It is the "
                "temperature of what the surface REFLECTS, which on a shiny "
                "target dominates the reading.")
        if self.ambient_c <= -KELVIN_AT_ZERO_C:
            raise ThermalRefusal(
                REFUSE_COMPENSATION,
                f"ambient_c={self.ambient_c!r} degC is at or below absolute "
                "zero; this is a unit mistake, most likely kelvin passed as "
                "celsius.")

    @property
    def ambient_k(self) -> float:
        return self.ambient_c + KELVIN_AT_ZERO_C

    @property
    def is_identity(self) -> bool:
        """True when emissivity is exactly 1.0, i.e. apparent == surface.
        Lets a caller say "blackbody assumed" out loud rather than by
        omission, and it is the ONLY compensation accepted by a device whose
        driver already applied one."""
        return self.emissivity == 1.0

    def assumptions(self) -> Tuple[str, ...]:
        return (
            f"emissivity={self.emissivity:g} (source: {self.source})",
            f"reflected ambient={self.ambient_c:g} degC, assumed uniform",
            "atmospheric transmission assumed 1.0 (valid at metres, not "
            "across a field)",
            "surface assumed diffuse; a specular target reflects the "
            "surroundings and this correction cannot recover it",
        )


@dataclass(frozen=True)
class TemperatureReading:
    """A temperature AND everything needed to weigh it.

    `celsius` is a float for a point reading, a numpy array for a field, and
    **None when the sensor SATURATED**. None rather than a clamped ceiling is
    deliberate: a clamped 80.0 flows through a threshold comparison, a JSON
    payload and an operator's eye without a murmur, and the whole point of
    survey finding 3 is that a saturated field is indistinguishable from a
    merely-warm one unless something says so.
    """

    celsius: object
    verified: bool
    device_key: str
    saturated: bool = False
    ceiling_c: Optional[float] = None
    saturated_fraction: float = 0.0
    assumptions: Tuple[str, ...] = ()
    caveats: Tuple[str, ...] = ()

    @property
    def is_evidence(self) -> bool:
        """True only for a reading a fire alert may lean on unaided.

        False while the conversion is unverified - which is EVERY device in
        the shipped database - and False when saturated. That is not
        pessimism, it is the state of this rover.
        """
        return bool(self.verified) and not self.saturated

    def celsius_or_refuse(self) -> float:
        """The number, or a refusal. For callers that want the value and must
        not be allowed to accidentally format a None into a payload."""
        if self.celsius is None:
            raise ThermalRefusal(
                REFUSE_CEILING,
                f"{self.device_key} SATURATED at its {self.ceiling_c:g} degC "
                "ceiling, so there is no temperature to report - only 'at "
                "least this hot'. A clamped ceiling value would be "
                "indistinguishable from a real reading at that temperature.")
        return self.celsius


def counts_to_apparent_kelvin(counts, conversion: RadiometricConversion):
    """counts -> apparent kelvin. Refuses a None conversion loudly rather than
    treating it as identity: a device with no conversion cannot measure, and
    returning raw counts labelled "kelvin" is the exact confusion this module
    prevents."""
    if conversion is None:
        raise ThermalRefusal(
            REFUSE_NO_CONVERSION,
            "no conversion supplied, so counts are just counts. They are not "
            "kelvin, they are not scaled kelvin, and there is no device-class "
            "default to fall back on.")
    return conversion.apparent_kelvin(counts)


def counts_to_celsius(counts,
                      conversion: RadiometricConversion,
                      compensation: EmissivityCompensation,
                      *,
                      device_key: str = "unnamed") -> TemperatureReading:
    """counts -> SURFACE temperature in celsius, emissivity-corrected,
    saturation-aware.

    ALL THREE ARGUMENTS ARE POSITIONAL AND REQUIRED. There is deliberately no
    overload that omits `compensation`: the failure this guards against is a
    caller who wanted "just the temperature" and silently got a blackbody
    assumption. If a blackbody really is what you want, say so by passing
    `EmissivityCompensation(emissivity=1.0, ambient_c=..., source="...")`,
    which records the choice in the reading's assumptions.

    SATURATION IS CHECKED IN THE SENSOR'S OWN DOMAIN - against the APPARENT
    temperature, before emissivity - because the ceiling is a property of the
    detector, not of the surface being looked at.

    Works on scalars without numpy and on arrays with it.
    """
    if compensation is None:
        raise ThermalRefusal(
            REFUSE_COMPENSATION,
            "emissivity and ambient are required. Assuming 0.95 for a shiny "
            "metal surface reads over a hundred degrees high, in the "
            "direction that invents a fire.")
    if conversion is None:
        raise ThermalRefusal(REFUSE_NO_CONVERSION, "no conversion supplied")

    # -- the double-correction guard -----------------------------------
    if conversion.emissivity_applied_upstream and not compensation.is_identity:
        raise ThermalRefusal(
            REFUSE_COMPENSATION,
            f"{device_key}'s driver ALREADY applied an emissivity correction "
            f"(source: {conversion.source}), so the values arriving here are "
            "object temperatures, not apparent ones. Correcting again applies "
            "the same physics twice - a silent, physically-shaped error that "
            "no residual plot distinguishes from a miscalibrated sensor. Pass "
            "the emissivity to the DRIVER, and use emissivity=1.0 here to say "
            "so explicitly.")

    t_app_k = counts_to_apparent_kelvin(counts, conversion)
    t_app_c = t_app_k - KELVIN_AT_ZERO_C
    ceiling = conversion.max_measurable_c

    # -- saturation, in the sensor's domain ----------------------------
    saturated_any = _any_at_or_above(t_app_c, ceiling)
    sat_fraction = 0.0
    if _is_array(t_app_c):
        np = _require_numpy()
        sat_fraction = float(np.count_nonzero(t_app_c >= ceiling)
                             / max(t_app_c.size, 1))
    elif saturated_any:
        sat_fraction = 1.0

    caveats = list(_conversion_caveats(conversion))
    if saturated_any:
        caveats.append(
            f"SATURATED: {sat_fraction * 100.0:.1f}% of the reading is at or "
            f"above the {ceiling:g} degC ceiling. Those pixels carry no "
            "temperature - only 'at least this hot'. They are NOT clamped to "
            "the ceiling, because a clamped value is indistinguishable from a "
            "real reading there.")

    def _finish(cels):
        return TemperatureReading(
            celsius=cels,
            verified=conversion.verified,
            device_key=device_key,
            saturated=bool(saturated_any),
            ceiling_c=ceiling,
            saturated_fraction=sat_fraction,
            assumptions=compensation.assumptions() + (
                f"counts->kelvin: {conversion.kelvin_per_count:g} K/count "
                f"+ {conversion.kelvin_at_zero_counts:g} K "
                f"(source: {conversion.source})",),
            caveats=tuple(caveats))

    # A saturated SCALAR has no number at all.
    if saturated_any and not _is_array(t_app_c):
        return _finish(None)

    if compensation.is_identity:
        # e == 1 makes the correction the identity exactly. Short-circuiting
        # avoids a fourth power and a fourth root for nothing and keeps an
        # exact-equality test in the selftest from being flaky for no reason.
        t_surf_k = t_app_k
    else:
        e = compensation.emissivity
        bracket = (t_app_k ** 4 - (1.0 - e) * compensation.ambient_k ** 4) / e
        if _any_nonpositive(bracket):
            # The apparent radiance is LESS than what the reflected
            # surroundings alone would contribute. Not a cold object - an
            # impossible one. A nan or a clamped floor here would hand a
            # plausible number to a fire alert.
            raise ThermalRefusal(
                REFUSE_UNPHYSICAL,
                f"with emissivity={e:g} and ambient={compensation.ambient_c:g} "
                "degC the reflected component alone exceeds the measured "
                "radiance, so no surface temperature exists. Check that "
                "ambient is the REFLECTED background (not the air, not the "
                "sensor housing) and that the emissivity belongs to this "
                "surface.")
        t_surf_k = bracket ** 0.25

    cels = t_surf_k - KELVIN_AT_ZERO_C

    # A saturated ARRAY keeps its unsaturated pixels and NaNs the rest. NaN
    # and not the ceiling: NaN propagates through a mean and refuses to be
    # compared, which is exactly the behaviour wanted from "no temperature
    # exists here".
    if saturated_any and _is_array(cels):
        np = _require_numpy()
        cels = np.where(t_app_c >= ceiling, np.nan, cels)

    return _finish(cels)


def check_threshold(device: "ThermalDevice", threshold_c: float,
                    assertion: Optional[RadiometryAssertion] = None) -> None:
    """Refuse a corroboration threshold this device cannot reach. Survey §10.4.

    "The threshold and the device must be chosen together." A 150 degC
    threshold is UNREACHABLE on an AMG8833 (80 degC ceiling) and marginal on a
    high-gain Lepton (140), and a threshold above the ceiling is not
    conservative - it is a detector that can never fire, silently, while every
    unit reports healthy. That is the same shape as FPMS_NPU_REQUIRED existing
    at all: blind must be loud.
    """
    conv = device.require_measurement(assertion)
    if threshold_c >= conv.max_measurable_c:
        raise ThermalRefusal(
            REFUSE_CEILING,
            f"threshold {threshold_c:g} degC is at or above "
            f"{device.display_name}'s {conv.max_measurable_c:g} degC ceiling, "
            "so it can NEVER be crossed: every reading saturates below it and "
            "the corroboration silently never fires. Either pick a threshold "
            f"inside the sensor's range (the {DISCRIMINATION_BAND_C[0]:g}-"
            f"{DISCRIMINATION_BAND_C[1]:g} degC band separates a sunlit cone "
            "from a flame with huge margin) or pick a sensor with a higher "
            "ceiling.")
    if threshold_c <= SUNLIT_CONE_MAX_C:
        raise ThermalRefusal(
            REFUSE_CEILING,
            f"threshold {threshold_c:g} degC is at or below the {SUNLIT_CONE_MAX_C:g} "
            "degC a sunlit orange traffic cone plausibly reaches, so it would "
            "confirm exactly the false positive the colour screen already "
            "makes. That is survey §1.2 with extra steps: two sensors "
            "agreeing on the wrong object because both are responding to "
            "'this is a bit different from its background'.")


# ===========================================================================
# SECTION 6 -- FRAME LAYOUT: THE SPLITTER
#
# One delivered frame can hold several co-registered planes plus metadata
# rows. This is a splitter, not a format tag - survey finding 5.
# ===========================================================================

@dataclass(frozen=True)
class PlaneSlice:
    """A named band of rows within the delivered frame."""

    row_offset: int
    rows: int

    def __post_init__(self):
        if self.row_offset < 0 or self.rows <= 0:
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"plane row_offset={self.row_offset} rows={self.rows} is not "
                "a real slice")

    @property
    def end(self) -> int:
        return self.row_offset + self.rows


@dataclass(frozen=True)
class FrameLayout:
    """How one delivered frame splits into planes.

    THE SHAPE THIS EXISTS FOR: the InfiRay P2 Pro delivers 256x384 where rows
    0-191 are the AGC picture and rows 192-383 are the raw 16-bit data,
    perfectly co-registered, at 25 Hz - picture and measurement in the same
    frame. The HT-301/T2S+ adds FOUR METADATA ROWS below the image carrying
    per-device calibration coefficients including emissivity terms.

    WITHOUT THE SPLIT the halves are indistinguishable by size and a reader
    that took the whole frame as counts would report temperatures for 192 rows
    of AGC pixels: right shape, right dtype, no meaning. The metadata rows are
    worse - they are coefficients, so converting them yields wild
    temperatures that look like a fire.
    """

    raw: Optional[PlaneSlice] = None
    """Rows carrying pre-AGC counts. Absent means the whole frame is the AGC
    picture (an ordinary single-plane camera)."""

    agc: Optional[PlaneSlice] = None
    """Rows carrying the camera's own AGC picture, co-registered with `raw`
    where both exist. Absent means the whole frame is data."""

    metadata: Optional[PlaneSlice] = None
    """Rows carrying device calibration coefficients and scene metadata.
    EXPOSED RAW AND NEVER DECODED HERE - see CONVERSION_DEVICE_METADATA."""

    def validate(self, height: int) -> None:
        seen = []
        for name in ("raw", "agc", "metadata"):
            p = getattr(self, name)
            if p is None:
                continue
            if p.end > height:
                raise ThermalRefusal(
                    REFUSE_DESCRIPTOR,
                    f"frame layout plane {name!r} ends at row {p.end} but the "
                    f"delivered frame is only {height} rows tall")
            seen.append((name, p))
        for i in range(len(seen)):
            for j in range(i + 1, len(seen)):
                (n1, p1), (n2, p2) = seen[i], seen[j]
                if p1.row_offset < p2.end and p2.row_offset < p1.end:
                    # Overlapping planes mean one of them is being read twice
                    # under two different meanings - the metadata rows read as
                    # counts is the case that invents a fire.
                    raise ThermalRefusal(
                        REFUSE_DESCRIPTOR,
                        f"frame layout planes {n1!r} and {n2!r} overlap. Rows "
                        "cannot be both a picture and a measurement.")

    @property
    def is_split(self) -> bool:
        return sum(1 for p in (self.raw, self.agc, self.metadata)
                   if p is not None) > 1

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> Optional["FrameLayout"]:
        if d is None:
            return None
        if not isinstance(d, dict):
            raise ThermalRefusal(REFUSE_DESCRIPTOR,
                                 "frame_layout must be an object")
        def plane(name):
            p = d.get(name)
            if p is None:
                return None
            if not isinstance(p, dict) or "rows" not in p:
                raise ThermalRefusal(
                    REFUSE_DESCRIPTOR,
                    f"frame_layout.{name} must be an object with `rows`")
            return PlaneSlice(row_offset=int(p.get("row_offset", 0)),
                              rows=int(p["rows"]))
        return cls(raw=plane("raw"), agc=plane("agc"),
                   metadata=plane("metadata"))


@dataclass(frozen=True)
class ControlHint:
    """A vendor control the capture layer must set. Descriptor data, no I/O.

    THE INFIRAY/HT-301 FAMILY HIJACKS `CAP_PROP_ZOOM` AS A COMMAND CHANNEL:
    0x8004 unlocks raw 16-bit mode, 0x8000 triggers a calibration
    (shutter/FFC), 0x80ff persists parameters. Without the unlock the camera
    streams a perfectly good picture and no data, which is exactly the
    "possible" radiometry state - the device might measure and this control is
    what decides. Carried in the JSON so a new SKU's magic number is a data
    edit, not a code change.
    """

    purpose: str
    property_name: str
    value: int
    note: str = ""


# ===========================================================================
# SECTION 7 -- THE DEVICE DESCRIPTOR
#
# Frozen, self-checking, and the place where a dishonest camera entry dies.
# ===========================================================================

@dataclass(frozen=True)
class ThermalDevice:
    """Everything needed to interpret one thermal camera's bytes.

    THE CONSISTENCY RULES ENFORCED HERE, EACH WITH THE FAILURE IT STOPS
    -------------------------------------------------------------------
    R1. radiometry yes/possible REQUIRES a conversion (and therefore a
        ceiling). Otherwise a device claims it can measure and there is
        nothing to measure with; whatever came back would be counts wearing a
        degree sign.
    R2. radiometry `no` FORBIDS a conversion. Otherwise somebody attaches a
        plausible slope to an AGC camera and every layer above is bypassed by
        a one-word JSON edit. This is the rule that makes constraint 2
        structural rather than procedural.
    R3. radiometry yes/possible is refused on an AGC-only pixel format. An
        8-bit GREY or a YUYV luma plane were both rescaled to the scene by the
        camera before the bytes left it.
    R4. A split frame (YUYV_RAW16, or any layout with a metadata plane)
        REQUIRES a `frame_layout` with a `raw` slice. Otherwise the human-
        facing half or the coefficient rows are read as measurements.
    R5. An I2C device may not carry a USB id. A udev rule written from such an
        entry could never fire and would look like a broken rule rather than
        an impossible one.
    """

    key: str
    display_name: str
    transport: str
    pixel_format: str
    width: int
    height: int
    """DELIVERED height, including every plane. For the P2 Pro this is 384,
    not the 192-row sensor: it is what a buffer length must match."""
    fps: float
    radiometry: str
    conversion: Optional[RadiometricConversion] = None
    byte_order: str = "<"
    frame_layout: Optional[FrameLayout] = None
    controls: Tuple[ControlHint, ...] = ()
    usb_vendor_id: Optional[str] = None
    usb_product_id: Optional[str] = None
    i2c_address: Optional[str] = None
    status: str = "UNVERIFIED"
    """UNVERIFIED until this exact model has streamed into this rover. Every
    shipped entry is UNVERIFIED; no thermal camera is in hand."""
    notes: Tuple[str, ...] = ()

    def __post_init__(self):
        if self.radiometry not in RADIOMETRY_STATES:
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"radiometry={self.radiometry!r} must be one of "
                f"{RADIOMETRY_STATES}. It is deliberately NOT a boolean: for "
                "the Lepton family a radiometric 3.5 and a picture-only 3.0 "
                "present the same USB identity, so 'we do not know' is a real "
                "state that must be expressible and must behave as 'no'.")
        if self.transport not in TRANSPORTS:
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"transport={self.transport!r} is not one of {TRANSPORTS}")
        if self.pixel_format not in PIXEL_FORMATS:
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"pixel_format={self.pixel_format!r} is not one of "
                f"{PIXEL_FORMATS}")
        if self.width <= 0 or self.height <= 0:
            raise ThermalRefusal(REFUSE_DESCRIPTOR,
                                 "width and height must be positive")
        if self.byte_order not in ("<", ">"):
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"byte_order={self.byte_order!r} must be '<' or '>'. Getting "
                "it wrong on a 16-bit stream does not fail - it byte-swaps "
                "every count, and a swapped 30000 (0x7530 -> 0x3075 = 12405) "
                "is a perfectly plausible temperature.")

        claims_measurement = self.radiometry in (RADIOMETRY_YES,
                                                 RADIOMETRY_POSSIBLE)

        # -- R1 ---------------------------------------------------------
        if claims_measurement and self.conversion is None:
            raise ThermalRefusal(
                REFUSE_NO_CONVERSION,
                f"device {self.key!r} declares radiometry={self.radiometry!r} "
                "but carries no conversion. A claim to measure IS the "
                "conversion (and its ceiling); without one there is nothing "
                'to report and the honest descriptor is radiometry "no".')

        # -- R2, and the important one ----------------------------------
        if not claims_measurement and self.conversion is not None:
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f'device {self.key!r} says radiometry "no" but carries a '
                "conversion. That combination has exactly one use: smuggling "
                "a temperature out of an AGC image. An AGC frame is rescaled "
                "to the current scene every frame, so no fixed slope can mean "
                "anything on it. Delete the conversion, or - if the hardware "
                'might really measure - declare radiometry "possible" and '
                "prove it with an assertion.")

        # -- R3 ---------------------------------------------------------
        if claims_measurement and self.pixel_format in AGC_ONLY_FORMATS:
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"device {self.key!r} declares radiometry="
                f"{self.radiometry!r} with pixel_format={self.pixel_format}. "
                "That format is AGC output by construction: the camera "
                "already mapped radiance to brightness against the current "
                "scene and threw the reference away. If the stream is really "
                f"packed 16-bit counts announced as YUYV, declare "
                f"{FMT_YUYV_RAW16} and a frame_layout.")

        # -- R4 ---------------------------------------------------------
        if self.frame_layout is not None:
            self.frame_layout.validate(self.height)
        needs_split = (self.pixel_format == FMT_YUYV_RAW16
                       or (self.frame_layout is not None
                           and self.frame_layout.metadata is not None))
        if needs_split and (self.frame_layout is None
                            or self.frame_layout.raw is None):
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"device {self.key!r} delivers a split frame but no "
                "frame_layout.raw slice names which rows are the data. These "
                "streams interleave a human AGC image, the measurement, and "
                "sometimes calibration coefficient rows in ONE buffer; "
                "without the slice, half the reported temperatures are "
                "reinterpreted AGC pixels and the coefficient rows convert to "
                "wild values that look like a fire.")

        # -- R5 ---------------------------------------------------------
        if self.transport == TRANSPORT_I2C and (self.usb_vendor_id
                                                or self.usb_product_id):
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"device {self.key!r} is on I2C but carries a USB id. There "
                "is no USB device to match, so a udev rule written from this "
                "entry could never fire and would look like a broken rule "
                "rather than an impossible one.")

    # -- derived shape ---------------------------------------------------
    @property
    def raw_plane(self) -> Optional[PlaneSlice]:
        """Rows of measurement data. For a single-plane radiometric camera
        (a Lepton streaming Y16) that is the whole frame."""
        if self.frame_layout is not None and self.frame_layout.raw is not None:
            return self.frame_layout.raw
        if self.radiometry == RADIOMETRY_NO:
            return None
        return PlaneSlice(row_offset=0, rows=self.height)

    @property
    def agc_plane(self) -> Optional[PlaneSlice]:
        if self.frame_layout is not None and self.frame_layout.agc is not None:
            return self.frame_layout.agc
        if self.frame_layout is not None and self.frame_layout.raw is not None:
            return None                       # split frame with no picture half
        return PlaneSlice(row_offset=0, rows=self.height)

    @property
    def data_rows(self) -> int:
        p = self.raw_plane
        return self.height if p is None else p.rows

    @property
    def frame_bytes(self) -> Optional[int]:
        """Exact byte count of one delivered frame, or None for RAW_COUNTS.

        THE CHEAPEST GUARD AGAINST CONSTRAINT 1's FAILURE. A 640x480 YUYV RGB
        webcam frame is 614400 bytes; a 160x120 Y16 Lepton frame is 38400. If
        the thermal binding drifted onto the RGB camera the buffer is wrong by
        a factor of sixteen and `normalise()` refuses immediately, instead of
        reinterpreting colour pixels as counts.
        """
        bpp = BYTES_PER_PIXEL[self.pixel_format]
        return None if bpp is None else self.width * self.height * bpp

    @property
    def usb_id(self) -> Optional[str]:
        if self.usb_vendor_id is None or self.usb_product_id is None:
            return None
        return f"{self.usb_vendor_id.lower()}:{self.usb_product_id.lower()}"

    @property
    def raw_is_16bit(self) -> bool:
        return self.pixel_format in SIXTEEN_BIT_FORMATS

    # -- THE GATE ---------------------------------------------------------
    def require_measurement(self,
                            assertion: Optional[RadiometryAssertion] = None
                            ) -> RadiometricConversion:
        """Return the conversion, or REFUSE. The only sanctioned way to get one.

        Call this before promising an operator a temperature. The refusal text
        is written to be read on a dashboard.

        THE THREE REFUSALS:
          * radiometry "no"       -> it cannot measure, ever.
          * radiometry "possible" -> it MIGHT, and nobody proved it. Treated
            exactly as "no" until a `RadiometryAssertion` for THIS key says
            otherwise. Assuming the better variant is the failure the survey
            named: a 3.0 and a 3.5 are indistinguishable over USB, and
            guessing right most of the time is worse than refusing, because
            the times it guesses wrong produce a confident fire confirmation
            from a picture.
          * a device_metadata conversion -> nothing here can perform it.
        """
        if self.radiometry == RADIOMETRY_NO:
            raise ThermalRefusal(
                REFUSE_NOT_RADIOMETRIC,
                f"{self.display_name} ({self.key}) emits an AGC image only. "
                "Its brightness is rescaled to the current scene every frame, "
                "so the same flame is white in a cold room and grey once the "
                "room warms - no per-pixel temperature exists to recover. "
                "Using it to CORROBORATE the RGB colour detector adds zero "
                "information while manufacturing confidence: a sunlit orange "
                "cone at 50-60 degC is mapped to the top of the AGC palette "
                "against grass, so it AGREES with the colour hit, on the "
                "wrong object, for a physically real reason. Show it to the "
                "operator; keep it out of the corroboration path.")

        if self.radiometry == RADIOMETRY_POSSIBLE:
            if assertion is None or not assertion.authorises(self):
                who = ("no assertion supplied" if assertion is None else
                       (f"the assertion names {assertion.device_key!r}, not "
                        f"{self.key!r}" if assertion.device_key != self.key
                        else "the assertion says NOT confirmed: "
                             + assertion.evidence))
                raise ThermalRefusal(
                    REFUSE_UNASSERTED,
                    f"{self.display_name} ({self.key}) MIGHT be radiometric "
                    f"and nobody has proven it is - {who}. This device class "
                    "has a picture-only variant that presents the SAME USB "
                    "identity (Lepton 3.5 vs 3.0, 2.5 vs 2.0), or needs a "
                    "vendor control to unlock its raw plane that may simply "
                    "not take. Until it is proven, it behaves exactly as a "
                    "picture-only camera - which is the only safe default, "
                    "because the alternative is a confident fire confirmation "
                    "produced by a camera that cannot measure. Prove it with "
                    "an operator assertion naming the variant read off the "
                    "hardware, or with evaluate_radiometry_probe() against a "
                    "known reference scene.")

        if self.conversion is None:                # unreachable via R1
            raise ThermalRefusal(REFUSE_NO_CONVERSION,
                                 f"{self.key} has no conversion")
        if self.conversion.kind != CONVERSION_LINEAR:
            raise ThermalRefusal(
                REFUSE_CONVERSION_NOT_HERE,
                f"{self.display_name} ({self.key}) is genuinely radiometric, "
                f"but its conversion is {self.conversion.kind}: the "
                "coefficients arrive in the frame's metadata rows and need a "
                "device-specific decoder. This module ships no approximation "
                "of it on purpose. The frame still exposes `metadata_rows`; "
                "until a decoder exists, treat the device as picture-only.")
        return self.conversion

    def can_measure(self,
                    assertion: Optional[RadiometryAssertion] = None) -> bool:
        """Non-raising form of `require_measurement`."""
        try:
            self.require_measurement(assertion)
            return True
        except ThermalRefusal:
            return False


# ===========================================================================
# SECTION 8 -- THE CAPTURE CONTRACT
#
# How this descriptor-only module talks to the capture layer that another
# file owns. Survey finding 2: one missing flag destroys every temperature
# and raises nothing.
# ===========================================================================

@dataclass(frozen=True)
class CaptureContract:
    """Exactly what the capture layer must do, derived from the descriptor.

    Built so the node author does not have to remember any of it. Every field
    corresponds to a way this integration silently fails.
    """

    device_key: str
    transport: str
    fourcc: Optional[str]
    width: int
    height: int
    fps: float
    convert_rgb_must_be_disabled: bool
    """`cv2.CAP_PROP_CONVERT_RGB = 0`, and it is the whole ballgame.

    Without it, libv4l converts the stream and `read()` returns 3-channel
    uint8. The upper bits are gone, the image looks perfect, and NO ERROR IS
    RAISED ANYWHERE. Every downstream stage keeps working and every
    temperature is wrong. `require_raw_frame_form()` is the assertion that
    catches it; this flag is the instruction that prevents it."""

    expected_dtype: str
    expected_shape: Tuple[int, int]
    expected_bytes: Optional[int]
    controls: Tuple[ControlHint, ...]
    notes: Tuple[str, ...]


def capture_contract(device: ThermalDevice) -> CaptureContract:
    """Everything the capture layer must set and then verify. No I/O."""
    fourcc = {FMT_Y16: "Y16 ", FMT_YUYV: "YUYV", FMT_YUYV_RAW16: "YUYV",
              FMT_GREY: "GREY"}.get(device.pixel_format)
    notes = [
        "Log the NEGOTIATED fourcc and resolution at startup, every time. A "
        "camera that silently fell back to another format is otherwise "
        "indistinguishable from one that honoured the request.",
        "Immediately after read(), call require_raw_frame_form(device, frame) "
        "and refuse to run if it raises. Do not log-and-continue: a degraded "
        "8-bit stream reaching the corroboration path is the failure this "
        "assertion exists for.",
    ]
    if device.raw_is_16bit:
        notes.append(
            "Prefer direct V4L2 where the vendor path allows it; use OpenCV "
            "where the existing open driver already does (the InfiRay "
            "family), since that path is well exercised. Known OpenCV rough "
            "edges with CONVERT_RGB disabled include a segfault on release() "
            "and behaviour that varies with whether the build links libv4l.")
    if device.frame_layout is not None and device.frame_layout.is_split:
        notes.append(
            "This device delivers ONE buffer holding several planes. Do not "
            "let any resize, colour conversion or codec touch it before "
            "normalise() splits it - a resample across the plane boundary "
            "mixes picture pixels into the measurement.")
    if device.conversion is not None and device.conversion.requires_shutter_sync:
        notes.append(
            "This device runs a periodic shutter/FFC. Carry a frame-validity "
            "flag from the device telemetry and drop frames captured during "
            "one; a fire alarm raised on a shutter frame is noise.")
    if device.radiometry == RADIOMETRY_POSSIBLE:
        notes.append(
            "Radiometry is UNPROVEN for this entry. Until an assertion "
            "exists, normalise() will return an AgcFrame and the node must "
            "publish radiometric=false rather than publishing temperatures.")
    return CaptureContract(
        device_key=device.key,
        transport=device.transport,
        fourcc=fourcc,
        width=device.width,
        height=device.height,
        fps=device.fps,
        convert_rgb_must_be_disabled=device.raw_is_16bit,
        expected_dtype=("uint16" if device.raw_is_16bit else
                        ("float" if device.pixel_format == FMT_RAW_COUNTS
                         else "uint8")),
        expected_shape=(device.height, device.width),
        expected_bytes=device.frame_bytes,
        controls=device.controls,
        notes=tuple(notes))


def require_raw_frame_form(device: ThermalDevice, frame) -> None:
    """THE ONE ASSERTION THAT KILLS THE WHOLE CONVERT_RGB FAILURE CLASS.

    Call it immediately after `cap.read()`, before anything else touches the
    frame, and refuse to run if it raises. Returns None on success.

    WHAT IT CATCHES, AND WHY NOTHING ELSE WOULD
    -------------------------------------------
    Without `cv2.CAP_PROP_CONVERT_RGB = 0`, OpenCV/libv4l converts a 16-bit
    stream into 3-channel uint8. The frame is a perfectly pretty picture. The
    upper bits are gone. `isOpened()` is True, `read()` returns True, the
    shape is sensible, the display looks right, and every temperature computed
    from it is wrong - by an amount that varies with the scene, so it does not
    even look like a constant offset. There is no exception anywhere in the
    stack. This function is the only thing standing between that and a fire
    alert.

    It checks THREE things:
      1. ndim - a 3-dimensional frame means the conversion happened.
      2. dtype - 8-bit data from a device that declares a 16-bit raw form
         means the conversion happened even if the array is 2-D (some paths
         return single-channel 8-bit).
      3. element count - the geometry is what the descriptor says, so a
         silent fallback to another resolution is caught too.
    """
    np = _require_numpy()
    if frame is None:
        raise ThermalRefusal(
            REFUSE_FRAME_FORM,
            f"{device.key}: read() returned None. Note that several V4L2 "
            "nodes on one physical device are METADATA ONLY - they open, "
            "report isOpened(), and never yield a frame - which is why "
            "find_camera() already insists on a successful non-empty read.")
    arr = np.asarray(frame)

    if arr.ndim == 3:
        raise ThermalRefusal(
            REFUSE_FRAME_FORM,
            f"{device.key} delivered a {arr.ndim}-dimensional "
            f"{arr.dtype} frame of shape {tuple(arr.shape)}. THE CAPTURE "
            "LAYER FORGOT cv2.CAP_PROP_CONVERT_RGB = 0. libv4l has converted "
            "the stream to 3-channel 8-bit and DISCARDED THE UPPER BITS. "
            "Nothing raised, the picture looks perfect, and every temperature "
            "derived from it is wrong. Set CAP_PROP_CONVERT_RGB to 0 before "
            "the first read and re-open the device; do not attempt to "
            "recover temperatures from this frame.")

    if device.raw_is_16bit and arr.dtype == np.uint8:
        raise ThermalRefusal(
            REFUSE_FRAME_FORM,
            f"{device.key} declares a 16-bit raw form "
            f"({device.pixel_format}) but delivered uint8 data. Either "
            "cv2.CAP_PROP_CONVERT_RGB was not set to 0, or the vendor "
            "control that unlocks raw mode did not take "
            + (f"({device.controls[0].property_name} = "
               f"0x{device.controls[0].value:04x})"
               if device.controls else "(see the device's controls)")
            + ". A converted 8-bit stream is a picture, not a measurement, "
            "and must not reach the corroboration path.")

    if device.pixel_format != FMT_RAW_COUNTS:
        expect_elems = device.width * device.height
        # A uint16 view of a Y16/YUYV_RAW16 buffer has one element per pixel;
        # a uint8 view has two. Accept either, since the caller may hand us
        # the buffer before or after the dtype reinterpretation - but nothing
        # else, so a silent resolution fallback still fails here.
        if arr.size not in (expect_elems, expect_elems * 2):
            raise ThermalRefusal(
                REFUSE_FRAME_FORM,
                f"{device.key} expects {device.width}x{device.height} = "
                f"{expect_elems} pixels but the frame carries {arr.size} "
                "elements. The camera negotiated a different resolution than "
                "requested (log the negotiated fourcc and geometry every "
                "time), or this is not the device you think it is - check "
                f"that {FPMS_CAM_THERMAL} points at the thermal camera.")


# ===========================================================================
# SECTION 9 -- CAPABILITIES
#
# What a caller may branch on, with the reasons attached so a UI can say WHY
# rather than just greying a button out.
# ===========================================================================

@dataclass(frozen=True)
class Capabilities:
    """An honest answer to "what can this camera actually do for us"."""

    key: str
    display_name: str
    transport: str
    pixel_format: str
    width: int
    height: int
    fps: float
    frame_bytes: Optional[int]

    can_render_image: bool
    """True for every thermal device: they all produce something a human can
    look at. This is the capability an AGC-only camera really has, and it is
    genuinely useful - a person can spot a hot region an RGB camera misses."""

    radiometry_declared: str
    radiometry_confirmed: bool
    awaiting_assertion: bool
    """True for a `possible` device with no assertion: the hardware might
    measure and nobody has proven it. The UI should ask for the proof, not
    hide the device."""

    can_measure_temperature: bool
    """True iff `require_measurement()` would not raise. Requires the
    assertion for a `possible` device."""

    temperature_is_verified: bool
    """True only when the conversion has been checked against a known
    reference ON THIS ROVER. False for every shipped entry."""

    max_measurable_c: Optional[float]
    survives_flame: bool
    """True iff the ceiling is above flame temperature. False means the device
    can say "at least this hot" and cannot rank two hot things - which is not
    nothing (a cone in the sun does not reach 80 degC) but is not a
    measurement of a fire either."""

    may_corroborate_colour_detection: bool
    """THE BRANCH THIS WHOLE MODULE EXISTS FOR.

    `detect_fire` is an HSV colour screen. Corroborating it with a second
    brightness image of the same scene adds NO independent information: both
    channels respond to "this object differs from its background", they fail
    together on a sunlit cone, and an alert saying "confirmed by thermal" on
    that basis is more misleading than an unconfirmed one. Only a device that
    MEASURES is an independent witness, so this is False for every AGC-only
    camera AND for every unproven one - and a caller must not fall back to
    "well, it's a thermal image, close enough"."""

    reasons: Tuple[str, ...] = ()
    caveats: Tuple[str, ...] = ()


def capabilities(device: ThermalDevice,
                 assertion: Optional[RadiometryAssertion] = None
                 ) -> Capabilities:
    """Describe what `device` can honestly be used for, given what has been
    proven about it."""
    reasons = []
    caveats = []
    conv = device.conversion
    can_measure = device.can_measure(assertion)
    awaiting = (device.radiometry == RADIOMETRY_POSSIBLE and not can_measure)

    if can_measure:
        reasons.append(
            f"radiometric: counts convert to kelvin by "
            f"{conv.kelvin_per_count:g} K/count (source: {conv.source})")
        caveats.extend(_conversion_caveats(conv))
    elif device.radiometry == RADIOMETRY_POSSIBLE:
        reasons.append(
            "radiometry UNPROVEN: this hardware might measure, and its USB "
            "identity cannot say (the picture-only variant is identical over "
            "USB, or the raw plane needs a vendor control that may not take). "
            "Treated exactly as picture-only until asserted or probed.")
    else:
        reasons.append(
            "AGC-only: the image is rescaled to the current scene every "
            "frame, so no per-pixel temperature exists in it. Good for an "
            "operator's eyes; useless as corroboration for a colour detector.")

    if conv is not None and conv.kind != CONVERSION_LINEAR:
        caveats.append(
            f"conversion kind is {conv.kind}: the coefficients arrive in the "
            "frame metadata and this module ships no decoder for them, so no "
            "temperature can be produced here even though the hardware "
            "measures.")

    if device.status != "VERIFIED":
        caveats.append(
            f"device entry status is {device.status}: this model has never "
            "streamed into this rover, so the resolution, format, frame size "
            "and ceiling are from a datasheet or a community driver, not from "
            "a capture.")

    if device.transport == TRANSPORT_I2C:
        caveats.append(
            "I2C transport: no USB id and no udev symlink. Bind by (bus, "
            f"address={device.i2c_address}); the {FPMS_CAM_THERMAL} scheme "
            "does not apply.")
    if device.transport == TRANSPORT_LIBUSB:
        caveats.append(
            "libusb transport: this device is NOT UVC and never appears as "
            "/dev/video*. It needs a userspace driver, and the udev rule for "
            "it grants access rather than creating a video symlink.")

    if device.fps and device.fps < 10.0:
        caveats.append(
            f"{device.fps:g} Hz unique frames: at 1 rad/s yaw that is "
            f"{57.3 / device.fps:.1f} deg of rotation between frames, so an "
            "RGB frame and a thermal frame no longer describe the same scene. "
            "Corroborate stopped, and associate by timestamp, never by "
            "arrival order.")

    return Capabilities(
        key=device.key,
        display_name=device.display_name,
        transport=device.transport,
        pixel_format=device.pixel_format,
        width=device.width,
        height=device.height,
        fps=device.fps,
        frame_bytes=device.frame_bytes,
        can_render_image=True,
        radiometry_declared=device.radiometry,
        radiometry_confirmed=can_measure,
        awaiting_assertion=awaiting,
        can_measure_temperature=can_measure,
        temperature_is_verified=bool(can_measure and conv.verified),
        max_measurable_c=(None if conv is None else conv.max_measurable_c),
        survives_flame=bool(conv is not None and conv.survives_flame
                            and can_measure),
        may_corroborate_colour_detection=can_measure,
        reasons=tuple(reasons),
        caveats=tuple(caveats))


# ===========================================================================
# SECTION 10 -- THE DEVICE DATABASE
#
# Data-driven so an unknown camera is added by editing JSON, not code. The
# CALLER does the file I/O (this module opens nothing); it hands in an
# already-parsed dict, exactly as fpms_motion.Calibration.from_profile does.
# ===========================================================================

@dataclass(frozen=True)
class DeviceDatabase:
    """Loaded devices, refused devices, and the notes explaining both.

    A REFUSED ENTRY IS REMEMBERED, NOT DROPPED. If a dishonest entry simply
    vanished, `resolve()` would report "no such device" and an operator would
    go looking for a typo in the key. Keeping the reason means the refusal is
    what they read instead.
    """

    devices: Dict[str, ThermalDevice] = field(default_factory=dict)
    refused: Dict[str, str] = field(default_factory=dict)
    notes: Tuple[str, ...] = ()

    def get(self, key: str) -> ThermalDevice:
        if key in self.devices:
            return self.devices[key]
        if key in self.refused:
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"device {key!r} is in the database but was REFUSED at load: "
                f"{self.refused[key]}")
        raise ThermalRefusal(
            REFUSE_UNKNOWN_DEVICE,
            f"no device {key!r} in the database. An unknown camera is added "
            "by editing /etc/fpms/thermal_devices.json - VID:PID, transport, "
            "format, geometry, radiometry, and (if it may measure) the "
            "conversion and its ceiling. Nothing here guesses a descriptor "
            f"for a device that is not described. Known keys: "
            f"{sorted(self.devices)}")

    def match_usb(self, vendor_id: str, product_id: str) -> Tuple[str, ...]:
        """Keys whose USB id equals vid:pid. MAY RETURN MORE THAN ONE.

        More than one is the NORMAL case for the Lepton, whose variant AND
        TLinear scale are both invisible over USB. `resolve()` treats that as
        a refusal, not as a menu to pick the first item from."""
        want = f"{str(vendor_id).lower()}:{str(product_id).lower()}"
        return tuple(sorted(k for k, d in self.devices.items()
                            if d.usb_id == want))

    @property
    def measuring_keys(self) -> Tuple[str, ...]:
        """Keys that could measure if proven. NOT the same as "will measure"."""
        return tuple(sorted(k for k, d in self.devices.items()
                            if d.radiometry != RADIOMETRY_NO))


def load_device_database(doc: Optional[dict]) -> DeviceDatabase:
    """Build a database from an already-parsed thermal_devices.json.

    ONE BAD ENTRY DOES NOT POISON THE FILE. Each device is validated on its
    own; a refused one is recorded with its reason and the rest still load. An
    exception here would mean a typo in an aspirational MLX90640 entry takes
    down the camera that is actually plugged in.

    THE REFUSALS THAT MATTER, all from `ThermalDevice.__post_init__`:
      * radiometry yes/possible with no conversion -> REFUSED,
      * radiometry "no" WITH a conversion -> REFUSED (the dishonest
        direction: it would let an AGC camera report temperatures),
      * radiometry yes/possible on GREY/YUYV -> REFUSED,
      * a conversion with no `max_measurable_c` -> REFUSED,
      * a split frame with no `raw` slice -> REFUSED.
    """
    notes = []

    if doc is None:
        notes.append("thermal: no device database supplied. No camera can be "
                     "identified and every temperature request will be "
                     "REFUSED. This is the shipped-with-no-file state.")
        return DeviceDatabase(notes=tuple(notes))

    if not isinstance(doc, dict):
        notes.append(f"thermal: database is {type(doc).__name__}, not a JSON "
                     "object; IGNORED entirely")
        return DeviceDatabase(notes=tuple(notes))

    schema = doc.get("schema")
    if schema != 1:
        # Refuse rather than best-effort parse. A future schema that renamed
        # `radiometry` would parse as "absent" here, and any default for that
        # field is wrong in one direction or the other.
        notes.append(f"thermal: database schema={schema!r}, expected 1; "
                     "IGNORED entirely rather than parsed on a guess")
        return DeviceDatabase(notes=tuple(notes))

    raw_devices = doc.get("devices")
    if not isinstance(raw_devices, dict):
        notes.append("thermal: database has no `devices` object; nothing "
                     "loaded")
        return DeviceDatabase(notes=tuple(notes))

    devices: Dict[str, ThermalDevice] = {}
    refused: Dict[str, str] = {}

    for key in sorted(raw_devices):
        entry = raw_devices[key]
        if not isinstance(entry, dict):
            refused[key] = f"entry is {type(entry).__name__}, not an object"
            notes.append(f"thermal {key}: REFUSED ({refused[key]})")
            continue
        try:
            devices[key] = _device_from_entry(key, entry)
        except ThermalRefusal as exc:
            refused[key] = str(exc)
            notes.append(f"thermal {key}: REFUSED - {exc}")
            continue
        except (TypeError, ValueError, KeyError) as exc:
            refused[key] = f"malformed entry: {exc}"
            notes.append(f"thermal {key}: REFUSED - malformed entry: {exc}")
            continue
        d = devices[key]
        notes.append(f"thermal {key}: loaded ({d.status}, "
                     f"radiometry={d.radiometry})")

    return DeviceDatabase(devices=devices, refused=refused, notes=tuple(notes))


def _device_from_entry(key: str, entry: dict) -> ThermalDevice:
    """One JSON object -> one ThermalDevice. Raises ThermalRefusal."""
    if "radiometric" in entry:
        # The old boolean field. Refuse loudly rather than migrating silently:
        # a bool cannot express "possible", and reading `true` from an old
        # file would restore exactly the assume-the-better-variant behaviour
        # the tri-state exists to remove.
        raise ThermalRefusal(
            REFUSE_DESCRIPTOR,
            "entry uses the old boolean field `radiometric`. Use "
            f'`radiometry`: one of {RADIOMETRY_STATES}. A boolean cannot say '
            '"this hardware might measure and its USB identity cannot tell '
            'us", which is the true state of every Lepton-on-PureThermal.')

    for req in ("display_name", "transport", "pixel_format", "width",
                "height", "radiometry"):
        if req not in entry:
            raise ThermalRefusal(
                REFUSE_DESCRIPTOR,
                f"entry is missing required field {req!r}. Nothing is "
                "defaulted here: a missing `radiometry` in particular must "
                "never read as no-by-omission (it would silently downgrade a "
                "real sensor) nor as yes, which is worse.")

    radiometry = entry["radiometry"]
    if not isinstance(radiometry, str):
        raise ThermalRefusal(
            REFUSE_DESCRIPTOR,
            f"radiometry={radiometry!r} must be one of the strings "
            f"{RADIOMETRY_STATES}. A JSON boolean here is the old schema; a "
            'string "false" is truthy in Python and would turn an AGC camera '
            "into a temperature sensor.")

    conv_raw = entry.get("conversion")
    conversion = (None if conv_raw is None
                  else RadiometricConversion.from_dict(conv_raw))

    layout = FrameLayout.from_dict(entry.get("frame_layout"))

    controls = []
    for c in entry.get("controls", ()) or ():
        if not isinstance(c, dict):
            raise ThermalRefusal(REFUSE_DESCRIPTOR,
                                 "controls entries must be objects")
        controls.append(ControlHint(purpose=str(c["purpose"]),
                                    property_name=str(c["property"]),
                                    value=int(c["value"]),
                                    note=str(c.get("note", ""))))

    usb = entry.get("usb") or {}
    if not isinstance(usb, dict):
        raise ThermalRefusal(REFUSE_DESCRIPTOR, "usb must be an object")

    return ThermalDevice(
        key=key,
        display_name=str(entry["display_name"]),
        transport=str(entry["transport"]),
        pixel_format=str(entry["pixel_format"]),
        width=int(entry["width"]),
        height=int(entry["height"]),
        fps=float(entry.get("fps", 0.0)),
        radiometry=radiometry,
        conversion=conversion,
        byte_order=str(entry.get("byte_order", "<")),
        frame_layout=layout,
        controls=tuple(controls),
        usb_vendor_id=(None if usb.get("vendor_id") is None
                       else str(usb["vendor_id"]).lower()),
        usb_product_id=(None if usb.get("product_id") is None
                        else str(usb["product_id"]).lower()),
        i2c_address=(None if entry.get("i2c_address") is None
                     else str(entry["i2c_address"])),
        status=str(entry.get("status", "UNVERIFIED")),
        notes=tuple(str(n) for n in entry.get("notes", ())))


def resolve(db: DeviceDatabase,
            key: Optional[str] = None,
            usb: Optional[Tuple[str, str]] = None) -> ThermalDevice:
    """Pick exactly one device, or REFUSE. Never picks "the first match".

    `key` is authoritative when supplied - it is what the udev rule's
    `ENV{FPMS_THERMAL_KEY}` and `/etc/fpms/config.env` set, and it is the ONLY
    way to name a device whose mode is invisible on the wire.

    WHY AN AMBIGUOUS USB MATCH IS A REFUSAL
    ---------------------------------------
    A Lepton on a PureThermal carrier can be a radiometric 3.5 streaming
    TLinear at 0.01 K/count, the same part at 0.1 K/count, or a picture-only
    3.0 - and NONE of that is visible in the USB descriptor, which is
    identical in all three. One VID:PID therefore legitimately matches several
    entries. Choosing one silently means a ten-times temperature error or a
    fabricated one. The refusal names the candidates and tells the operator to
    set the key.
    """
    if key is not None:
        return db.get(key)

    if usb is None:
        raise ThermalRefusal(
            REFUSE_UNKNOWN_DEVICE,
            "no device key and no USB id supplied, so there is nothing to "
            "resolve. A thermal camera is never assumed to be present: with "
            "no device, the fire detector runs on the RGB colour screen "
            "alone, which is a known and stated limitation rather than a "
            "silent one.")

    candidates = db.match_usb(usb[0], usb[1])
    ident = f"{str(usb[0]).lower()}:{str(usb[1]).lower()}"
    if not candidates:
        raise ThermalRefusal(
            REFUSE_UNKNOWN_DEVICE,
            f"USB {ident} matches no entry in "
            "/etc/fpms/thermal_devices.json. Add it there - VID:PID, "
            "transport, pixel format, geometry, and radiometry \"no\" unless "
            "you can name the counts-to-kelvin relation AND the measurement "
            "ceiling. An unknown camera is not assumed to be like a known "
            "one, and an unbranded dongle with no named sensor should be "
            "treated as picture-only until `v4l2-ctl --list-formats-ext` "
            "shows a 16-bit format or a double-height mode.")
    if len(candidates) > 1:
        raise ThermalRefusal(
            REFUSE_AMBIGUOUS_DEVICE,
            f"USB {ident} matches {len(candidates)} entries: "
            f"{list(candidates)}. This is normal and it is NOT a database "
            "bug: for the Lepton family the module variant (3.5 radiometric "
            "vs 3.0 picture-only), the TLinear scale (0.01 vs 0.1 K/count) "
            "and whether TLinear is on at all are all invisible over USB. "
            "Picking one would scale every temperature by ten, or invent "
            "temperatures from a picture. Set the key explicitly "
            "(FPMS_THERMAL_DEVICE in /etc/fpms/config.env, or "
            "ENV{FPMS_THERMAL_KEY} in the udev rule) after confirming what "
            "the camera actually is.")
    return db.get(candidates[0])


# ===========================================================================
# SECTION 11 -- FRAMES
#
# The canonical forms, and the two classes that make constraint 2 structural.
#
#   proven measuring device -> RadiometricFrame, 16-bit counts
#   everything else         -> AgcFrame,         8-bit mono
#
# `AgcFrame` has no temperature method. Not a method that raises - NO METHOD.
# `hasattr(frame, "temperature_at")` is False, `dir(frame)` contains no
# "celsius" and no "kelvin", and the class is frozen so one cannot be attached
# to an instance either. The selftest asserts all of that by inspection rather
# than trusting this comment.
# ===========================================================================

@dataclass(frozen=True)
class AgcFrame:
    """A canonical 8-bit mono frame from a device that is NOT measuring.

    Produced for a picture-only camera AND for an unproven `possible` one -
    deliberately the SAME TYPE, because "we cannot measure" and "we have not
    proven we can measure" must be indistinguishable to every consumer. A
    consumer that could tell them apart would eventually branch on it.

    WHAT THE PIXELS MEAN: relative scene brightness in the thermal band, after
    automatic gain control. Brighter is warmer WITHIN THIS FRAME. Nothing
    more. Two consecutive frames are not comparable and no pixel value maps to
    any temperature.

    THE ONE HONEST USE: showing a human a second view of the scene. That is
    genuinely valuable - a person can spot a hot region an RGB camera misses -
    but it is a HUMAN corroborating, with judgement about what the gain is
    doing, and it must not be laundered into an automatic "thermal confirms".
    """

    mono8: object
    """numpy uint8 array, shape (rows, width). The canonical AGC form."""

    device_key: str
    width: int
    height: int
    raw_plane_withheld: bool = False
    """True when the delivered frame CONTAINED a raw plane that was
    deliberately not interpreted, because radiometry is unproven. The data
    exists; converting it would be the guess this module refuses to make. Run
    `evaluate_radiometry_probe()` on the raw counts (a separate, explicitly
    named entry point) or supply an operator assertion."""

    withheld_reason: str = ""
    metadata_rows: object = None
    notes: Tuple[str, ...] = ()

    @property
    def can_measure_temperature(self) -> bool:
        """Always False, and present so a caller can branch without
        `isinstance`. It is NOT a switch: there is no method to enable."""
        return False

    def why_no_temperature(self) -> str:
        """The sentence to put on a dashboard next to this image."""
        if self.raw_plane_withheld:
            return (
                f"{self.device_key} MIGHT be radiometric and nobody has "
                f"proven it is. {self.withheld_reason} Until it is proven it "
                "behaves exactly as a picture-only camera, because a "
                "confident fire confirmation from a camera that cannot "
                "measure is worse than no confirmation at all.")
        return (
            f"{self.device_key} is AGC-only. Its gain is rescaled to the "
            "current scene every frame, so pixel brightness is relative to "
            "whatever else is in view and carries no temperature. This image "
            "is for a human to look at; it cannot corroborate the fire "
            "detector, because a sunlit orange cone is mapped to the top of "
            "the palette exactly as a flame is.")


@dataclass(frozen=True)
class HotspotReport:
    """What the hot pixels are doing. Measuring devices only, by placement.

    A REPORT, NOT A VERDICT. Whether a hot region is a fire is a decision for
    the layer that also holds the RGB detection, the range, the FFC validity
    flag and the mission state. Survey §7's conservative recommendation stands:
    publish a region summary in the thermal camera's own frame and associate
    by bearing with declared uncertainty - do not fuse at pixel level in v1.
    """

    pixel_count: int
    total_pixels: int
    fraction: float
    max_celsius: Optional[float]
    """None when the hottest pixel SATURATED. There is no number to give."""
    max_xy: Tuple[int, int]
    threshold_c: float
    saturated: bool
    saturated_fraction: float
    ceiling_c: float
    verified: bool
    device_key: str
    assumptions: Tuple[str, ...] = ()
    caveats: Tuple[str, ...] = ()


@dataclass(frozen=True)
class RadiometricFrame:
    """A canonical counts frame from a device PROVEN to measure.

    CANNOT BE CONSTRUCTED WITHOUT A CONVERSION - `__post_init__` refuses a
    None one (L2). Since `ThermalDevice` refuses to let a picture-only device
    carry a conversion (L3), and `normalise()` only builds this type after
    `require_measurement()` has passed (L5), there is no route from an AGC or
    unproven camera to an instance of this class. The absence of a route is
    the guarantee; the docstrings are only the explanation.
    """

    counts: object
    """numpy array, shape (data_rows, width). RAW COUNTS, not kelvin: the
    conversion is applied at the point of asking, so the unconverted data
    stays available for a caller that wants to re-ask with a different
    emissivity without re-capturing."""

    conversion: RadiometricConversion
    device_key: str
    width: int
    height: int
    companion_agc: Optional[AgcFrame] = None
    """The co-registered picture half, where the device delivers one (the
    InfiRay split frame). Same instant, same optics, same pixel grid - which
    is what makes it worth carrying: the operator's view and the measurement
    cannot disagree about which frame they came from."""

    metadata_rows: object = None
    """The device's calibration coefficient rows, RAW AND UNDECODED. Present
    so whoever writes the HT-301/T2S+ decoder has the bytes; nothing in this
    module interprets them."""

    notes: Tuple[str, ...] = ()

    def __post_init__(self):
        if self.conversion is None:
            raise ThermalRefusal(
                REFUSE_NO_CONVERSION,
                "a RadiometricFrame cannot exist without a conversion. If you "
                "have counts and no conversion, what you have is an image.")

    @property
    def can_measure_temperature(self) -> bool:
        return True

    @property
    def counts_in_range(self) -> bool:
        """False when the frame leaves the device's documented count range.

        A DIAGNOSTIC, NOT A CORRECTION. All-zero and all-0xFFFF frames are
        what a mis-bound or disconnected sensor produces, and both convert to
        a uniform, plausible temperature field. Nothing here repairs it; a
        caller should treat False as "this is not a picture of the world".
        """
        c = self.conversion
        if c.count_min is None and c.count_max is None:
            return True
        if not _is_array(self.counts):
            return True                            # pragma: no cover
        lo = c.count_min if c.count_min is not None else -(2 ** 62)
        hi = c.count_max if c.count_max is not None else (2 ** 62)
        return bool((self.counts >= lo).all() and (self.counts <= hi).all())

    def apparent_celsius_at(self, x: int, y: int) -> float:
        """Apparent (blackbody-equivalent) temperature at one pixel.

        Named `apparent` because that is what it is. A surface with emissivity
        below 1 differs from this and `temperature_at` is the one that
        accounts for it. NOTE this does NOT apply the saturation rule: it is
        the sensor's own domain, and a caller asking for the apparent value is
        asking what the detector said."""
        return float(self.conversion.apparent_celsius(
            float(self._pixel(x, y))))

    def temperature_at(self, x: int, y: int,
                       compensation: EmissivityCompensation
                       ) -> TemperatureReading:
        """Surface temperature at one pixel. `compensation` IS REQUIRED, and
        the reading is None when the sensor saturated there."""
        return counts_to_celsius(float(self._pixel(x, y)), self.conversion,
                                 compensation, device_key=self.device_key)

    def celsius_field(self, compensation: EmissivityCompensation
                      ) -> TemperatureReading:
        """The whole frame as celsius, saturated pixels NaN.

        ONE EMISSIVITY FOR THE WHOLE FRAME IS ALWAYS WRONG SOMEWHERE - a scene
        contains foliage and metal at once. Offered because a single
        conservative value plus this caveat is more honest than per-pixel
        values nobody supplied. The caveat is attached to the reading."""
        r = counts_to_celsius(self.counts, self.conversion, compensation,
                              device_key=self.device_key)
        return TemperatureReading(
            celsius=r.celsius, verified=r.verified, device_key=r.device_key,
            saturated=r.saturated, ceiling_c=r.ceiling_c,
            saturated_fraction=r.saturated_fraction,
            assumptions=r.assumptions,
            caveats=r.caveats + (
                "one emissivity applied to the whole frame: any surface in "
                "view with a different emissivity is reported wrong, and "
                "shiny surfaces are reported far too hot.",))

    def max_celsius(self, compensation: EmissivityCompensation
                    ) -> TemperatureReading:
        """Hottest pixel in the frame, compensated - or SATURATED with no
        number, which is the whole point of survey finding 3."""
        np = _require_numpy()
        peak = float(np.max(self.counts))
        return counts_to_celsius(peak, self.conversion, compensation,
                                 device_key=self.device_key)

    def hotspot(self, threshold_c: float,
                compensation: EmissivityCompensation) -> HotspotReport:
        """How much of the frame is above `threshold_c`, and where the peak is.

        SATURATED PIXELS COUNT AS ABOVE THRESHOLD (they are at least the
        ceiling, and the ceiling is above any usable threshold - see
        `check_threshold`), but they contribute no number to `max_celsius`.
        Counting them as below would make a fire hot enough to saturate the
        sensor read as no detection at all, which is the worst possible
        direction for this failure.
        """
        np = _require_numpy()
        field_r = counts_to_celsius(self.counts, self.conversion, compensation,
                                    device_key=self.device_key)
        cels = field_r.celsius
        app_c = self.conversion.apparent_celsius(self.counts)
        sat_mask = app_c >= self.conversion.max_measurable_c

        above = np.logical_or(sat_mask,
                              np.nan_to_num(cels, nan=-1e9) >= threshold_c)
        count = int(np.count_nonzero(above))
        total = int(cels.size)

        flat = int(np.argmax(app_c))              # peak in the sensor's domain
        yy, xx = divmod(flat, app_c.shape[1])
        peak_saturated = bool(sat_mask.reshape(-1)[flat])
        max_c = None
        if not peak_saturated:
            max_c = float(np.nanmax(cels)) if total else None

        return HotspotReport(
            pixel_count=count,
            total_pixels=total,
            fraction=(count / total if total else 0.0),
            max_celsius=max_c,
            max_xy=(int(xx), int(yy)),
            threshold_c=float(threshold_c),
            saturated=bool(field_r.saturated),
            saturated_fraction=field_r.saturated_fraction,
            ceiling_c=self.conversion.max_measurable_c,
            verified=field_r.verified,
            device_key=self.device_key,
            assumptions=field_r.assumptions,
            caveats=field_r.caveats)

    def _pixel(self, x: int, y: int):
        arr = self.counts
        rows, cols = arr.shape[0], arr.shape[1]
        if not (0 <= x < cols and 0 <= y < rows):
            raise ThermalRefusal(
                REFUSE_FRAME_SIZE,
                f"pixel ({x}, {y}) is outside the {cols}x{rows} data plane. "
                "On a split-frame device the data plane is SHORTER than the "
                "delivered frame, so a y taken from the full frame height "
                "indexes the wrong rows.")
        return arr[y, x]


def _slice_planes(device: ThermalDevice, plane, np):
    """(raw_rows, agc_rows, metadata_rows) sliced out of one delivered frame."""
    layout = device.frame_layout
    if layout is None:
        return plane, plane, None
    def cut(p):
        return None if p is None else plane[p.row_offset:p.end, :]
    raw = cut(layout.raw)
    agc = cut(layout.agc)
    meta = cut(layout.metadata)
    if raw is None:
        raw = plane
    if agc is None and layout.raw is None:
        agc = plane
    return raw, agc, meta


def _to_mono8(plane, np):
    """Whatever arrived -> 8-bit mono for human display.

    A 16-bit AGC stream (some devices emit Y16 that has ALREADY been
    gain-controlled) is scaled down here, and scaling loses nothing: the scene
    reference was gone before the bytes left the camera.
    """
    if plane.dtype == np.uint8:
        return plane
    lo, hi = float(plane.min()), float(plane.max())
    span = (hi - lo) or 1.0
    return ((plane.astype(np.float32) - lo) * (255.0 / span)).astype(np.uint8)


def normalise(device: ThermalDevice, raw,
              assertion: Optional[RadiometryAssertion] = None):
    """Raw device frame -> the canonical form for that device.

    RETURNS A `RadiometricFrame` OR AN `AgcFrame`, AND THE CALLER CANNOT
    CHOOSE. The device descriptor plus the assertion decide. A picture-only
    camera, and an unproven `possible` one, both have no path to an object
    with a temperature method on it.

    `raw` may be:
      * bytes/bytearray/memoryview - a V4L2/UVC buffer, checked against
        `device.frame_bytes` EXACTLY. A wrong size means the binding drifted
        onto another camera (constraint 1) or the format is not what the
        descriptor says. Both are refusals, never reshapes.
      * a numpy array - for FMT_RAW_COUNTS (an I2C driver's output) or a
        pre-decoded frame. Passed through `require_raw_frame_form()` first, so
        a silently RGB-converted frame is refused here too rather than being
        reshaped into plausible nonsense.

    CANONICAL FORMS
      measuring : counts, shape (data_rows, width), native byte order
                  resolved, planes split, companion AGC picture attached where
                  the device delivers one, metadata rows carried undecoded.
      otherwise : uint8 mono, shape (rows, width), plus the reason any raw
                  plane present was withheld.
    """
    np = _require_numpy()
    fmt = device.pixel_format
    bpp = BYTES_PER_PIXEL[fmt]

    if isinstance(raw, (bytes, bytearray, memoryview)):
        buf = bytes(raw)
        expect = device.frame_bytes
        if expect is None:
            raise ThermalRefusal(
                REFUSE_FRAME_SIZE,
                f"{device.key} declares {FMT_RAW_COUNTS}, which is a numeric "
                "array from a driver, not a byte buffer. Passing bytes here "
                "would reinterpret a driver's floats as counts.")
        if len(buf) != expect:
            raise ThermalRefusal(
                REFUSE_FRAME_SIZE,
                f"{device.key} produces {expect} bytes per frame "
                f"({device.width}x{device.height} {fmt}, {bpp} B/px) but "
                f"{len(buf)} arrived. THIS IS USUALLY A BINDING ERROR, NOT A "
                "SHORT READ: a 640x480 YUYV colour webcam frame is 614400 "
                "bytes, so if the thermal binding drifted onto the RGB camera "
                f"this is what it looks like. Check that {FPMS_CAM_THERMAL} "
                f"points at the thermal device and {FPMS_CAM_RGB} at the "
                "colour one, and never open either by index.")
        if fmt == FMT_GREY:
            plane = np.frombuffer(buf, dtype=np.uint8).reshape(
                device.height, device.width)
        elif fmt == FMT_YUYV:
            # 4:2:2 packing is Y0 U Y1 V; the luma bytes are every other one.
            plane = np.frombuffer(buf, dtype=np.uint8)[0::2].reshape(
                device.height, device.width)
        else:
            plane = np.frombuffer(buf, dtype=np.dtype(device.byte_order + "u2")
                                  ).reshape(device.height, device.width)
    else:
        # THE CONVERT_RGB GUARD, on every array that enters this module.
        require_raw_frame_form(device, raw)
        arr = np.asarray(raw)
        if arr.size != device.width * device.height:
            raise ThermalRefusal(
                REFUSE_FRAME_SIZE,
                f"{device.key} produces {device.width}x{device.height} = "
                f"{device.width * device.height} values but {arr.size} "
                "arrived. For an I2C sensor this normally means a partial "
                "read, which must NOT be padded: a padded frame has "
                "real-looking cold pixels.")
        plane = arr.reshape(device.height, device.width)

    raw_rows, agc_rows, meta_rows = _slice_planes(device, plane, np)

    # -- THE GATE. `possible` behaves exactly as `no` until proven. -------
    try:
        conversion = device.require_measurement(assertion)
    except ThermalRefusal as refusal:
        withheld = (device.radiometry != RADIOMETRY_NO)
        mono_src = agc_rows if agc_rows is not None else raw_rows
        return AgcFrame(
            mono8=_to_mono8(mono_src, np),
            device_key=device.key,
            width=device.width,
            height=int(mono_src.shape[0]),
            raw_plane_withheld=withheld,
            withheld_reason=(str(refusal) if withheld else ""),
            metadata_rows=meta_rows,
            notes=device.notes)

    counts = (raw_rows if fmt == FMT_RAW_COUNTS
              else raw_rows.astype(np.uint16, copy=False))
    companion = None
    if agc_rows is not None and device.frame_layout is not None \
            and device.frame_layout.agc is not None:
        companion = AgcFrame(mono8=_to_mono8(agc_rows, np),
                             device_key=device.key,
                             width=device.width,
                             height=int(agc_rows.shape[0]),
                             notes=("co-registered picture half of a split "
                                    "frame; display only",))
    return RadiometricFrame(counts=counts,
                            conversion=conversion,
                            device_key=device.key,
                            width=device.width,
                            height=int(counts.shape[0]),
                            companion_agc=companion,
                            metadata_rows=meta_rows,
                            notes=device.notes)


# ===========================================================================
# SECTION 12 -- COLOURISATION, FOR HUMANS ONLY
# ===========================================================================

@dataclass(frozen=True)
class DisplayOnlyImage:
    """An RGB image for a human. NEVER an input to detection.

    WHY THIS IS A WRAPPER AND NOT A NUMPY ARRAY
    -------------------------------------------
    A bare (H, W, 3) uint8 array is exactly what `cv2.cvtColor` and
    `fpms_rover_agent.detect_fire` accept. If this returned one, the shortest
    path from "we have a thermal camera" to "the fire detector is using it"
    would be to pass the colourised frame into `detect_fire` - closing a loop
    with no information in it: a false-colour palette maps brightness to hue
    by a lookup WE chose, so hunting for flame colours in it finds the
    palette. On an AGC device it is worse, because the brightness itself was
    rescaled to the scene.

    So the pixels live behind `.rgb`, and reaching for them is a deliberate,
    greppable act. `palette` and `span_note` are here so a UI can label the
    image with the range it was stretched over - without that label, every
    operator reads a false-colour image as an absolute temperature scale,
    which is the human version of the same mistake.
    """

    rgb: object
    palette: str
    caveat: str
    device_key: str
    span_note: str = ""

    display_only = True
    """Class attribute, not a field: it cannot be constructed as False."""


_PALETTES = ("grey", "ironbow")


def colourise_for_display(frame, palette: str = "ironbow",
                          span: Optional[Tuple[float, float]] = None
                          ) -> DisplayOnlyImage:
    """Render a frame for a human. Accepts either frame type.

    `span` fixes the ends of the stretch. Supplying it is what makes two
    frames comparable BY EYE; omitting it re-stretches each frame to its own
    min/max, which is the same trap AGC sets and which makes a cooling fire
    look constant. The choice is recorded in `span_note` either way.

    THE OUTPUT IS NOT A MEASUREMENT AND IS NOT A DETECTOR INPUT. It carries no
    temperature and cannot be converted back to one: the palette is
    many-to-one.
    """
    np = _require_numpy()
    if palette not in _PALETTES:
        raise ThermalRefusal(REFUSE_DESCRIPTOR,
                             f"palette={palette!r} is not one of {_PALETTES}")

    if isinstance(frame, RadiometricFrame):
        data = frame.counts.astype(np.float32)
        key = frame.device_key
        base = ("stretched from raw COUNTS; the colours are not a temperature "
                "scale and no conversion is applied here")
    elif isinstance(frame, AgcFrame):
        data = frame.mono8.astype(np.float32)
        key = frame.device_key
        base = ("AGC source: brightness was already rescaled to the scene by "
                "the camera, so this image is not comparable frame to frame")
    else:
        raise ThermalRefusal(
            REFUSE_DESCRIPTOR,
            "colourise_for_display takes a RadiometricFrame or an AgcFrame, "
            f"not {type(frame).__name__}. Passing a bare array would lose the "
            "device identity the caveat text depends on.")

    if span is None:
        lo, hi = float(data.min()), float(data.max())
        span_note = (f"auto-stretched over this frame only [{lo:g}, {hi:g}]; "
                     "NOT comparable with any other frame")
    else:
        lo, hi = float(span[0]), float(span[1])
        span_note = f"fixed stretch [{lo:g}, {hi:g}]; comparable frame to frame"
    if hi <= lo:
        hi = lo + 1.0
    norm = np.clip((data - lo) / (hi - lo), 0.0, 1.0)

    if palette == "grey":
        g = (norm * 255.0).astype(np.uint8)
        rgb = np.dstack([g, g, g])
    else:
        # Cheap piecewise ironbow, arithmetic rather than a lookup table so
        # this module keeps no data blob and no cv2 dependency.
        r = np.clip(norm * 3.0, 0.0, 1.0)
        g = np.clip(norm * 3.0 - 1.0, 0.0, 1.0)
        b = np.clip(norm * 3.0 - 2.0, 0.0, 1.0)
        b = np.maximum(b, np.clip(0.6 - norm * 3.0, 0.0, 1.0) * 0.8)
        rgb = np.dstack([(r * 255).astype(np.uint8),
                         (g * 255).astype(np.uint8),
                         (b * 255).astype(np.uint8)])

    return DisplayOnlyImage(
        rgb=rgb, palette=palette, device_key=key, span_note=span_note,
        caveat=("DISPLAY ONLY. " + base + ". Never feed this to detect_fire "
                "or any colour heuristic: the hues come from a palette we "
                "chose, so a colour detector run on it would be detecting the "
                "palette and reporting it as corroboration."))


# ===========================================================================
# SECTION 13 -- BINDING
#
# Constraint 1, as a function a caller can be made to pass through. Pure
# string checks - this module still opens nothing and stats nothing.
# ===========================================================================

_INDEXY_PREFIXES = ("/dev/video",)


def require_distinct_bindings(rgb_path, thermal_path) -> None:
    """Refuse any camera binding that can silently swap. Returns None on OK.

    THE THREE REFUSALS, AND THE INCIDENT BEHIND EACH
    ------------------------------------------------
    1. AN INTEGER (or a digit string). `fpms_rover_agent.py:474` calls
       `cv2.VideoCapture(idx)` with `FPMS_CAMERA_INDEX=0`. With two cameras
       the index depends on enumeration order, which is stable across neither
       boots nor a mid-run USB re-enumeration - this rover has already had
       /dev/video0 become /dev/video1 after an error -71.
    2. A BARE /dev/videoN. The same thing wearing a path. The kernel assigns N
       in enumeration order, and several nodes on one device are METADATA
       ONLY: they open fine, report isOpened(), and never produce a frame,
       which is why `find_camera()` already insists on a successful read.
    3. THE SAME PATH TWICE. Both streams then come from one camera. The fire
       detector sees the thermal image (finds no flame hue, ever) or the
       thermal reader sees colour bytes. Neither raises anywhere.

    The sanctioned paths are the udev symlinks in 98-fpms-thermal.rules, or a
    /dev/v4l/by-path/ entry - the camera equivalent of the
    /dev/serial/by-path/ discipline `fpms_duty_driver.validate_port()` already
    enforces, and for the same reason: both CP2102 adapters report ID_SERIAL
    "0001", so identity-based naming was never available on this rover.
    """
    for label, p in (("rgb", rgb_path), ("thermal", thermal_path)):
        if isinstance(p, int) or (isinstance(p, str) and p.strip().isdigit()):
            raise ThermalRefusal(
                REFUSE_BINDING,
                f"{label} camera is bound by INDEX ({p!r}). Indices shift when "
                "a second camera is plugged in, when USB re-enumerates, and "
                "across boots - and nothing errors when they do; the fire "
                "detector simply looks for flame hue in a thermal image, or "
                f"the thermal reader converts colour pixels. Use "
                f"{FPMS_CAM_RGB} / {FPMS_CAM_THERMAL}.")
        if not isinstance(p, str) or not p:
            raise ThermalRefusal(
                REFUSE_BINDING,
                f"{label} camera binding {p!r} is not a device path")
        for pre in _INDEXY_PREFIXES:
            if p.startswith(pre) and p[len(pre):].isdigit():
                raise ThermalRefusal(
                    REFUSE_BINDING,
                    f"{label} camera is bound to {p}, which is an index with "
                    "a path in front of it. The kernel assigns videoN in "
                    "enumeration order and some nodes on the same device are "
                    "metadata-only (they open, report isOpened(), and never "
                    f"yield a frame). Use {FPMS_CAM_RGB} / {FPMS_CAM_THERMAL}, "
                    "or a /dev/v4l/by-path/ entry.")

    if rgb_path == thermal_path:
        raise ThermalRefusal(
            REFUSE_BINDING,
            f"both cameras are bound to {rgb_path}. One device cannot be both "
            "the colour screen and its independent corroboration - that is "
            "the failure this whole module exists to make impossible, and it "
            "produces no error anywhere downstream.")
