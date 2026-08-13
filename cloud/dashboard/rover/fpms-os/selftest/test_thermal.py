#!/usr/bin/env python3
"""fpms_thermal against synthetic frames with known content.

    python3 selftest/test_thermal.py

Exit 0 all pass, 1 one or more fail.

WHY THIS FILE EXISTS
====================
No thermal camera has ever been connected to this rover, and the one that
eventually is will not be the one this was written against. So the thing worth
testing is not "does the arithmetic work" - it is "can this module be made to
report a temperature it has no right to report". Every check below names, in
its comment, the real failure it guards.

The three claims this file is here to prove, in order of importance:

  1. A NON-RADIOMETRIC DEVICE CANNOT BE MADE TO REPORT A TEMPERATURE BY ANY
     CODE PATH. Not "returns None", not "logs a warning" - there is no method,
     no attribute, no constructor argument and no JSON edit that gets one out.
     Section B walks the public API exhaustively looking for a way in.
  2. THE JSON LOADER REFUSES A DISHONEST ENTRY. Six shapes of dishonesty,
     each of which would silently manufacture confidence in a fire alert.
  3. UNKNOWN AND ABSENT DEVICES REFUSE CLEANLY rather than defaulting to
     anything - including defaulting to the better of two variants that share
     a USB identity, which is the specific trap docs/THERMAL_HARDWARE.md
     found for the FLIR Lepton 3.5 versus 3.0.

WHAT THIS IS NOT
================
It is not a camera. Every frame here is synthesised from a formula, so a check
that passes proves the module handles what the DESCRIPTOR says the device
produces - not what the device actually produces. Every device entry in
thermal_devices.json is UNVERIFIED for exactly that reason, and section D
asserts that they all still say so.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.path.normpath(os.path.join(HERE, "..", "overlay", "usr", "local",
                                    "lib", "fpms"))
JSON_PATH = os.path.normpath(os.path.join(HERE, "..", "overlay", "etc",
                                          "fpms", "thermal_devices.json"))
sys.path.insert(0, LIB)

try:
    import numpy as np
except Exception as exc:                                  # pragma: no cover
    print(f"SKIP: numpy is required for these tests: {exc}")
    sys.exit(1)

try:
    import fpms_thermal as T
except Exception as exc:                                  # pragma: no cover
    # SKIP LOUDLY. A test that cannot import the thing it tests must never
    # look like a pass - that is how a module gets deleted and nothing goes red.
    print(f"SKIP: cannot import fpms_thermal from {LIB}: {exc}")
    sys.exit(1)

FAILURES = []
CHECKS = [0]


def check(name, ok, detail=""):
    CHECKS[0] += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:<58} {detail}")
    if not ok:
        FAILURES.append(name)


def refuses(fn, *a, **k):
    """(did_it_refuse, message). Only a ThermalRefusal counts as a refusal -
    a TypeError or an AttributeError would mean the module crashed rather than
    declined, and those are different outcomes with different fixes."""
    try:
        fn(*a, **k)
    except T.ThermalRefusal as exc:
        return True, str(exc)
    except Exception as exc:
        return False, f"raised {type(exc).__name__}: {exc}"
    return False, "returned without refusing"


# ---------------------------------------------------------------- fixtures --
#
# Hand-built devices, so a check does not depend on the shipped JSON staying
# as it is. Section D tests the shipped file separately.

LEPTON = T.ThermalDevice(
    key="test-lepton-10mk", display_name="test Lepton TLinear 0.01",
    transport=T.TRANSPORT_UVC, pixel_format=T.FMT_Y16,
    width=160, height=120, fps=8.7, radiometry=T.RADIOMETRY_POSSIBLE,
    usb_vendor_id="1e4e", usb_product_id="0100",
    conversion=T.RadiometricConversion(
        kind=T.CONVERSION_LINEAR, kelvin_per_count=0.01,
        kelvin_at_zero_counts=0.0, max_measurable_c=140.0,
        source="test fixture mirroring the TLinear 0.01 K/count entry"))

AGC_ONLY = T.ThermalDevice(
    key="test-agc", display_name="test picture-only dongle",
    transport=T.TRANSPORT_UVC, pixel_format=T.FMT_YUYV,
    width=256, height=192, fps=25.0, radiometry=T.RADIOMETRY_NO)

PROVEN = T.ThermalDevice(
    key="test-proven", display_name="test proven radiometric",
    transport=T.TRANSPORT_UVC, pixel_format=T.FMT_Y16,
    width=32, height=24, fps=8.0, radiometry=T.RADIOMETRY_YES,
    conversion=T.RadiometricConversion(
        kind=T.CONVERSION_LINEAR, kelvin_per_count=0.01,
        kelvin_at_zero_counts=0.0, max_measurable_c=140.0,
        source="test fixture"))

ASSERTED = T.RadiometryAssertion(
    device_key="test-lepton-10mk", confirmed=True,
    method="test fixture", evidence="synthetic", asserted_by="selftest")

BLACKBODY = T.EmissivityCompensation(emissivity=1.0, ambient_c=25.0,
                                     source="test: blackbody assumed")
PAINTED = T.EmissivityCompensation(emissivity=0.95, ambient_c=25.0,
                                   source="test: painted/organic surface")
SHINY = T.EmissivityCompensation(emissivity=0.10, ambient_c=25.0,
                                 source="test: bare aluminium")


def y16_bytes(device, counts_value, hot=None):
    """A Y16 buffer of `counts_value`, optionally with one hotter pixel."""
    a = np.full((device.height, device.width), counts_value, dtype="<u2")
    if hot is not None:
        a[device.height // 2, device.width // 2] = hot
    return a.tobytes()


def counts_for_c(conv, celsius):
    """Counts that produce `celsius` apparent through `conv`."""
    return (celsius + T.KELVIN_AT_ZERO_C - conv.kelvin_at_zero_counts) \
        / conv.kelvin_per_count


print(__doc__.strip().splitlines()[0])
print()

# ===========================================================================
print("A. DESCRIPTOR HONESTY -- the four combinations that must not exist")
print("-" * 78)
# ===========================================================================

# A device that claims it measures but carries nothing to measure with would
# build a RadiometricFrame whose temperature_at has no conversion. Whatever
# came back would be counts wearing a degree sign.
ok, msg = refuses(T.ThermalDevice, key="k", display_name="d",
                  transport=T.TRANSPORT_UVC, pixel_format=T.FMT_Y16,
                  width=8, height=8, fps=1.0, radiometry=T.RADIOMETRY_YES)
check("radiometry yes with no conversion is refused", ok, msg[:44])

# THE DISHONEST DIRECTION, and the one that makes constraint 2 structural
# rather than procedural: attaching a plausible slope to an AGC camera would
# bypass every other layer with a one-word JSON edit.
ok, msg = refuses(T.ThermalDevice, key="k", display_name="d",
                  transport=T.TRANSPORT_UVC, pixel_format=T.FMT_Y16,
                  width=8, height=8, fps=1.0, radiometry=T.RADIOMETRY_NO,
                  conversion=T.RadiometricConversion(
                      kind=T.CONVERSION_LINEAR, kelvin_per_count=0.01,
                      kelvin_at_zero_counts=0.0, max_measurable_c=140.0,
                      source="smuggled"))
check("radiometry no WITH a conversion is refused", ok, msg[:44])

# A YUYV luma plane and an 8-bit GREY frame have both already been through the
# camera's AGC. Somebody adds a camera, writes radiometry "yes" because the box
# said thermal, and the format they also wrote down proves otherwise.
for fmt in T.AGC_ONLY_FORMATS:
    ok, msg = refuses(T.ThermalDevice, key="k", display_name="d",
                      transport=T.TRANSPORT_UVC, pixel_format=fmt,
                      width=8, height=8, fps=1.0, radiometry=T.RADIOMETRY_YES,
                      conversion=T.RadiometricConversion(
                          kind=T.CONVERSION_LINEAR, kelvin_per_count=0.01,
                          kelvin_at_zero_counts=0.0, max_measurable_c=140.0,
                          source="x"))
    check(f"radiometry yes on AGC-only format {fmt} is refused", ok, msg[:40])

# A sensor with no stated ceiling cannot be given a corroboration threshold
# honestly, and its saturated frames are indistinguishable from merely-warm
# ones. THE CEILING IS REQUIRED - survey finding 3.
ok, msg = refuses(T.RadiometricConversion, kind=T.CONVERSION_LINEAR,
                  kelvin_per_count=0.01, kelvin_at_zero_counts=0.0,
                  max_measurable_c=None, source="x")
check("a conversion with no max_measurable_c is refused", ok, msg[:44])

# A linear slope beside "the real conversion lives in the frame metadata"
# means the approximation is what gets used, silently, forever.
ok, msg = refuses(T.RadiometricConversion, kind=T.CONVERSION_DEVICE_METADATA,
                  kelvin_per_count=0.01, kelvin_at_zero_counts=0.0,
                  max_measurable_c=450.0, source="x")
check("device_metadata conversion carrying a slope is refused", ok, msg[:40])

# A zero slope maps the whole scene onto one plausible temperature; a negative
# one reports fires as cold spots. Both look exactly like data.
for bad in (0.0, -0.01):
    ok, _ = refuses(T.RadiometricConversion, kind=T.CONVERSION_LINEAR,
                    kelvin_per_count=bad, kelvin_at_zero_counts=0.0,
                    max_measurable_c=140.0, source="x")
    check(f"kelvin_per_count={bad} is refused", ok)

# An unattributed slope cannot be checked, corrected or blamed. On this rover
# that is how CMD_SCALE = 6.1 survived as a "calibration".
ok, _ = refuses(T.RadiometricConversion, kind=T.CONVERSION_LINEAR,
                kelvin_per_count=0.01, kelvin_at_zero_counts=0.0,
                max_measurable_c=140.0, source="   ")
check("a conversion with no source provenance is refused", ok)

# A split frame with no raw slice would read the human-facing half, or the
# calibration coefficient rows, as measurements.
ok, msg = refuses(T.ThermalDevice, key="k", display_name="d",
                  transport=T.TRANSPORT_UVC, pixel_format=T.FMT_YUYV_RAW16,
                  width=256, height=384, fps=25.0,
                  radiometry=T.RADIOMETRY_YES,
                  conversion=T.RadiometricConversion(
                      kind=T.CONVERSION_LINEAR, kelvin_per_count=0.015625,
                      kelvin_at_zero_counts=0.0, max_measurable_c=550.0,
                      source="x"))
check("split-frame format with no frame_layout.raw is refused", ok, msg[:40])

# Overlapping planes mean rows are read twice under two meanings. The
# coefficient rows read as counts is the case that invents a fire.
ok, _ = refuses(T.FrameLayout(raw=T.PlaneSlice(0, 192),
                              metadata=T.PlaneSlice(190, 4)).validate, 200)
check("overlapping frame planes are refused", ok)

# A udev rule written from an I2C entry with a USB id could never fire, and
# would look like a broken rule rather than an impossible one.
ok, _ = refuses(T.ThermalDevice, key="k", display_name="d",
                transport=T.TRANSPORT_I2C, pixel_format=T.FMT_RAW_COUNTS,
                width=8, height=8, fps=1.0, radiometry=T.RADIOMETRY_NO,
                usb_vendor_id="1234", usb_product_id="5678")
check("an I2C device carrying a USB id is refused", ok)


# ===========================================================================
print()
print("B. THE HEADLINE CLAIM -- an AGC device cannot report a temperature")
print("-" * 78)
# ===========================================================================

agc_buf = (np.arange(AGC_ONLY.width * AGC_ONLY.height * 2, dtype=np.uint8)
           % 251).tobytes()
agc_frame = T.normalise(AGC_ONLY, agc_buf)

# L1: TWO FRAME TYPES, NOT ONE FLAG. The device descriptor decides which type
# comes back and the caller cannot choose.
check("normalise on an AGC device returns AgcFrame",
      isinstance(agc_frame, T.AgcFrame), type(agc_frame).__name__)

# THE PROOF ITSELF. Not "the method returns None" - THERE IS NO METHOD. A
# method that raised could be caught by an over-broad `except Exception` three
# layers up and turned into a default; an absent method cannot.
for meth in ("temperature_at", "max_celsius", "hotspot", "celsius_field",
             "apparent_celsius_at", "counts", "conversion"):
    check(f"AgcFrame has no {meth!r}", not hasattr(agc_frame, meth))

# Nothing on the class or the instance even NAMES a temperature, so there is
# no attribute a caller could stumble onto by tab-completion or getattr().
names = set(dir(agc_frame))
tempish = sorted(n for n in names
                 if any(w in n.lower()
                        for w in ("celsius", "kelvin", "temp", "degre"))
                 and n not in ("can_measure_temperature", "why_no_temperature"))
check("no temperature-named attribute on an AgcFrame", not tempish,
      str(tempish))

# ...and the two names that DO mention temperature are the refusals.
check("AgcFrame.can_measure_temperature is False",
      agc_frame.can_measure_temperature is False)
check("AgcFrame.why_no_temperature() explains the AGC problem",
      "rescaled" in agc_frame.why_no_temperature())

# Frozen, so a temperature cannot be grafted onto an instance at runtime
# either. (AttributeError covers dataclasses.FrozenInstanceError.)
try:
    agc_frame.temperature_at = lambda x, y, c: 400.0
    grafted = True
except (AttributeError, TypeError):
    grafted = False
check("a temperature method cannot be attached to an AgcFrame", not grafted)

# THE GATE ITSELF. require_measurement is the only sanctioned way to obtain a
# conversion, and it refuses on the merits with text an operator can read.
ok, msg = refuses(AGC_ONLY.require_measurement)
check("require_measurement refuses on an AGC device", ok, msg[:40])
check("...and the refusal explains WHY corroboration is worse than nothing",
      "manufactur" in msg or "zero information" in msg)
check("AGC_ONLY.can_measure() is False", AGC_ONLY.can_measure() is False)

caps = T.capabilities(AGC_ONLY)
check("capabilities.can_measure_temperature is False",
      caps.can_measure_temperature is False)
# THE BRANCH THE WHOLE MODULE EXISTS FOR. Corroborating an HSV colour screen
# with a second brightness image adds no independent information: both respond
# to "this differs from its background" and both fire on a sunlit cone.
check("capabilities.may_corroborate_colour_detection is False",
      caps.may_corroborate_colour_detection is False)
check("capabilities still says it can render an image for a human",
      caps.can_render_image is True)

# L2: no conversion, no RadiometricFrame. There is no default conversion
# anywhere in the module to fall back on.
ok, _ = refuses(T.RadiometricFrame, counts=np.zeros((4, 4), dtype=np.uint16),
                conversion=None, device_key="k", width=4, height=4)
check("RadiometricFrame cannot be built without a conversion", ok)

# And the free functions refuse a None conversion rather than treating it as
# identity - returning raw counts labelled "kelvin" is the exact confusion.
ok, _ = refuses(T.counts_to_apparent_kelvin, 30000, None)
check("counts_to_apparent_kelvin(None conversion) refuses", ok)
ok, _ = refuses(T.counts_to_celsius, 30000, None, PAINTED)
check("counts_to_celsius(None conversion) refuses", ok)

# THE EXHAUSTIVE SWEEP. Walk every public callable and try to get a number out
# of it using only an AGC device and its frame. This is the mechanised form of
# "by any code path": anything that returned a float here would be a hole.
leaked = []
for name in T.__all__:
    obj = getattr(T, name, None)
    if not callable(obj):
        continue
    for args in ((agc_frame,), (AGC_ONLY,), (AGC_ONLY, agc_frame)):
        try:
            got = obj(*args)
        except Exception:
            continue
        if isinstance(got, (int, float)) and not isinstance(got, bool):
            leaked.append(f"{name}{args!r} -> {got}")
        if getattr(got, "celsius", None) is not None:
            leaked.append(f"{name} -> reading with celsius {got.celsius}")
check("no public callable yields a number from an AGC device",
      not leaked, str(leaked[:2]))

# There is no DATA path either: the loader refuses the entry that would pair a
# picture-only device with a conversion, so the JSON cannot create one.
bad_db = T.load_device_database({"schema": 1, "devices": {"x": {
    "display_name": "smuggler", "transport": "uvc", "pixel_format": "Y16",
    "width": 8, "height": 8, "radiometry": "no",
    "conversion": {"kind": "linear", "kelvin_per_count": 0.01,
                   "kelvin_at_zero_counts": 0.0, "max_measurable_c": 140.0,
                   "source": "smuggled"}}}})
check("JSON cannot pair radiometry no with a conversion",
      "x" in bad_db.refused and "x" not in bad_db.devices)


# ===========================================================================
print()
print("C. THE 'possible' GATE -- unproven behaves exactly as picture-only")
print("-" * 78)
# ===========================================================================

buf = y16_bytes(LEPTON, int(counts_for_c(LEPTON.conversion, 20.0)),
                hot=int(counts_for_c(LEPTON.conversion, 100.0)))

# THE FINDING THAT BREAKS THE OBVIOUS DESIGN: Lepton 3.5 (radiometric) and 3.0
# (picture-only) present the SAME USB identity. An entry that asserted
# radiometry from VID:PID would be making a claim the OS cannot verify.
unproven = T.normalise(LEPTON, buf)
check("unproven 'possible' device yields an AgcFrame",
      isinstance(unproven, T.AgcFrame), type(unproven).__name__)
check("...the SAME type a picture-only camera yields",
      type(unproven) is type(agc_frame))
check("...and it has no temperature_at either",
      not hasattr(unproven, "temperature_at"))
# The data IS there and was deliberately not interpreted. Saying so is the
# difference between a refusal and a bug.
check("...and it says the raw plane was withheld, not absent",
      unproven.raw_plane_withheld is True)
check("...and why_no_temperature() names the unproven variant",
      "proven" in unproven.why_no_temperature())

ok, msg = refuses(LEPTON.require_measurement)
check("require_measurement refuses an unasserted 'possible' device", ok)
check("...naming the identical-USB-identity variant problem",
      "same USB identity" in msg.lower() or "picture-only variant" in msg)

caps = T.capabilities(LEPTON)
check("capabilities marks it awaiting_assertion", caps.awaiting_assertion)
check("...and still refuses corroboration",
      caps.may_corroborate_colour_detection is False)

# An assertion for a DIFFERENT device must not authorise this one. A bare
# boolean in config.env could not express that distinction at all.
wrong = T.RadiometryAssertion(device_key="some-other-camera", confirmed=True,
                              method="m", evidence="e", asserted_by="a")
ok, _ = refuses(LEPTON.require_measurement, wrong)
check("an assertion naming another device does not authorise this one", ok)

# A NEGATIVE assertion is a result, not an error: an operator who correctly
# identifies a 3.0 has done the right thing.
neg = T.RadiometryAssertion.from_operator("test-lepton-10mk", "Lepton 3.0",
                                          "part number on the module",
                                          "operator")
check("asserting a Lepton 3.0 yields confirmed=False", neg.confirmed is False)
ok, _ = refuses(LEPTON.require_measurement, neg)
check("...and a negative assertion still refuses measurement", ok)

pos = T.RadiometryAssertion.from_operator("test-lepton-10mk", "Lepton 3.5",
                                          "part number on the module",
                                          "operator")
check("asserting a Lepton 3.5 yields confirmed=True", pos.confirmed is True)

# Asserting a variant you cannot name from the hardware is exactly the guess
# the tri-state exists to prevent.
ok, _ = refuses(T.RadiometryAssertion.from_operator, "test-lepton-10mk",
                "some thermal camera", "the box said thermal", "operator")
check("asserting an unrecognised variant is refused", ok)

# An unattributed assertion is a guess with a timestamp.
ok, _ = refuses(T.RadiometryAssertion, device_key="k", confirmed=True,
                method="", evidence="e", asserted_by="a")
check("an assertion with no method is refused", ok)

# THE PROBE. A radiometric part in TLinear reads ~29315 counts at 20 C; a
# picture-only 3.0 reports 14-bit scene flux with no fixed relation to
# temperature. One known reference separates them by a huge margin.
good = T.evaluate_radiometry_probe(LEPTON, 29315.0, 20.0, 5.0, "selftest")
check("probe against a 20 C reference confirms a TLinear part", good.confirmed,
      good.evidence[:36])
bad = T.evaluate_radiometry_probe(LEPTON, 8000.0, 20.0, 5.0, "selftest")
check("probe on 14-bit flux counts does NOT confirm",
      bad.confirmed is False, bad.evidence[:36])
check("...and a negative probe does not raise, it reports",
      isinstance(bad, T.RadiometryAssertion))

# Once proven, and ONLY once proven, the frame type changes.
proven = T.normalise(LEPTON, buf, ASSERTED)
check("an asserted 'possible' device yields a RadiometricFrame",
      isinstance(proven, T.RadiometricFrame), type(proven).__name__)
check("...which does have temperature_at", hasattr(proven, "temperature_at"))


# ===========================================================================
print()
print("D. THE SHIPPED DATABASE AND THE LOADER'S REFUSALS")
print("-" * 78)
# ===========================================================================

with open(JSON_PATH, "r", encoding="utf-8") as fh:
    doc = json.load(fh)
db = T.load_device_database(doc)

# The shipped file must be self-consistent, or the rover boots with no camera
# database and every temperature request refuses for the wrong reason.
check("shipped thermal_devices.json loads with zero refusals",
      not db.refused, str(sorted(db.refused))[:44])
check("shipped database is non-empty", len(db.devices) >= 8,
      f"{len(db.devices)} devices")

# NOTHING HERE HAS BEEN MEASURED. If somebody flips a status or a verified
# flag without connecting hardware, this is what goes red.
unverified = [k for k, d in db.devices.items() if d.status != "UNVERIFIED"]
check("every shipped device is still marked UNVERIFIED", not unverified,
      str(unverified))
claimed = [k for k, d in db.devices.items()
           if d.conversion is not None and d.conversion.verified]
check("no shipped conversion claims verified:true", not claimed, str(claimed))

# The ceiling is required for anything that might measure, because the
# threshold and the device must be chosen together.
noceil = [k for k, d in db.devices.items()
          if d.radiometry != T.RADIOMETRY_NO
          and (d.conversion is None or d.conversion.max_measurable_c is None)]
check("every measuring entry carries a ceiling", not noceil, str(noceil))

# The Lepton entries are the reason resolve() must refuse: three database
# entries legitimately share one USB identity.
lepton_keys = db.match_usb("1e4e", "0100")
check("three shipped entries share the PureThermal USB identity",
      len(lepton_keys) == 3, str(lepton_keys))
# ...and at least one of them is picture-only, which is what makes guessing
# from the identity a fabrication rather than merely a scaling error.
check("...and at least one of them is picture-only",
      any(db.devices[k].radiometry == T.RADIOMETRY_NO for k in lepton_keys))

# THE FIVE DISHONEST ENTRIES. Each is refused on its own; the honest entry
# beside them still loads, because a typo in an aspirational MLX90640 entry
# must not take down the camera that is actually plugged in.
honest = {"display_name": "honest", "transport": "uvc",
          "pixel_format": "Y16", "width": 8, "height": 8, "radiometry": "no"}
dishonest = {
    "claims-yes-no-conversion": {
        "display_name": "d", "transport": "uvc", "pixel_format": "Y16",
        "width": 8, "height": 8, "radiometry": "yes"},
    "no-with-conversion": {
        "display_name": "d", "transport": "uvc", "pixel_format": "Y16",
        "width": 8, "height": 8, "radiometry": "no",
        "conversion": {"kind": "linear", "kelvin_per_count": 0.01,
                       "kelvin_at_zero_counts": 0.0,
                       "max_measurable_c": 140.0, "source": "s"}},
    "yes-on-yuyv": {
        "display_name": "d", "transport": "uvc", "pixel_format": "YUYV",
        "width": 8, "height": 8, "radiometry": "yes",
        "conversion": {"kind": "linear", "kelvin_per_count": 0.01,
                       "kelvin_at_zero_counts": 0.0,
                       "max_measurable_c": 140.0, "source": "s"}},
    "no-ceiling": {
        "display_name": "d", "transport": "uvc", "pixel_format": "Y16",
        "width": 8, "height": 8, "radiometry": "yes",
        "conversion": {"kind": "linear", "kelvin_per_count": 0.01,
                       "kelvin_at_zero_counts": 0.0, "source": "s"}},
    "old-boolean-schema": {
        "display_name": "d", "transport": "uvc", "pixel_format": "Y16",
        "width": 8, "height": 8, "radiometric": True,
        "conversion": {"kind": "linear", "kelvin_per_count": 0.01,
                       "kelvin_at_zero_counts": 0.0,
                       "max_measurable_c": 140.0, "source": "s"}},
    "radiometry-not-a-string": {
        "display_name": "d", "transport": "uvc", "pixel_format": "Y16",
        "width": 8, "height": 8, "radiometry": True},
}
mixed = dict(dishonest)
mixed["honest"] = honest
mdb = T.load_device_database({"schema": 1, "devices": mixed})
for key in dishonest:
    check(f"loader refuses {key}", key in mdb.refused and key not in mdb.devices)
check("...while the honest entry beside them still loads",
      "honest" in mdb.devices)

# A REFUSED ENTRY IS REMEMBERED, NOT DROPPED. If it simply vanished, an
# operator would go hunting for a typo in the key instead of reading why.
ok, msg = refuses(mdb.get, "no-with-conversion")
check("get() on a refused key reports the refusal, not 'unknown'",
      ok and "REFUSED at load" in msg, msg[:40])

# An unknown key must refuse, not return None and not fall back to anything.
ok, msg = refuses(mdb.get, "camera-nobody-described")
check("get() on an unknown key refuses cleanly", ok, msg[:40])
check("...and the refusal says how to add one",
      "thermal_devices.json" in msg)

# Absent database: the shipped-with-no-file state must be a refusal, never a
# guess. Compare the silence trap in config.env's own header.
empty = T.load_device_database(None)
check("a missing database loads to zero devices and a note",
      not empty.devices and empty.notes)
ok, _ = refuses(T.resolve, empty, None, ("1e4e", "0100"))
check("resolve against an empty database refuses", ok)

# A future schema that renamed a field would parse as "absent" under a
# best-effort loader, and any default for `radiometry` is wrong in one
# direction or the other.
future = T.load_device_database({"schema": 2, "devices": {"a": honest}})
check("an unknown schema version is refused entirely", not future.devices)


# ===========================================================================
print()
print("E. RESOLUTION AND BINDING -- never guess which camera is which")
print("-" * 78)
# ===========================================================================

# THE AMBIGUITY REFUSAL. Choosing between the two TLinear scales silently is a
# factor of ten on every temperature; choosing the AGC entry fabricates them.
ok, msg = refuses(T.resolve, db, None, ("1e4e", "0100"))
check("an ambiguous USB match refuses instead of picking the first", ok)
check("...and the refusal names the candidates and how to disambiguate",
      "FPMS_THERMAL_DEVICE" in msg and "0.01" in msg)

# An unambiguous one resolves.
one = T.resolve(db, None, ("0bda", "5830"))
check("an unambiguous USB match resolves", one.key == "infiray-p2pro", one.key)

# An unknown camera is not assumed to be like a known one.
ok, msg = refuses(T.resolve, db, None, ("dead", "beef"))
check("an unknown USB id refuses cleanly", ok, msg[:40])
check("...telling the operator to declare radiometry no unless proven",
      'radiometry \"no\"' in msg or "radiometry" in msg)

# No device at all is a stated limitation, not a silent one.
ok, msg = refuses(T.resolve, db, None, None)
check("resolve with nothing to go on refuses", ok)
check("...and says the RGB colour screen runs alone", "colour screen" in msg)

# An explicit key beats everything, which is the only way to name a device
# whose real mode is invisible on the wire.
check("an explicit key resolves past the ambiguity",
      T.resolve(db, "flir-lepton-purethermal-tlinear-10mk").key
      == "flir-lepton-purethermal-tlinear-10mk")

# CONSTRAINT 1. fpms_rover_agent.py:474 opens the camera by integer index, and
# that has already failed in the field after a USB re-enumeration.
for bad in (0, "0", 1):
    ok, _ = refuses(T.require_distinct_bindings, bad, T.FPMS_CAM_THERMAL)
    check(f"binding the RGB camera by index {bad!r} is refused", ok)
ok, msg = refuses(T.require_distinct_bindings, "/dev/video0",
                  T.FPMS_CAM_THERMAL)
check("a bare /dev/videoN is refused (an index with a path in front)", ok)
check("...and the refusal names the metadata-only node trap",
      "metadata-only" in msg)
ok, _ = refuses(T.require_distinct_bindings, T.FPMS_CAM_RGB, T.FPMS_CAM_RGB)
check("binding both cameras to one device is refused", ok)
check("the two symlinks are distinct", T.FPMS_CAM_RGB != T.FPMS_CAM_THERMAL)
T.require_distinct_bindings(T.FPMS_CAM_RGB, T.FPMS_CAM_THERMAL)
check("the two udev symlinks are accepted", True)
# by-path is the discipline validate_port() already enforces for the serial
# devices, for the same reason: identity naming was never available here.
T.require_distinct_bindings("/dev/v4l/by-path/platform-x-usb-0:1.1:1.0-video-index0",
                            T.FPMS_CAM_THERMAL)
check("a /dev/v4l/by-path entry is accepted", True)


# ===========================================================================
print()
print("F. RAW FRAME FORM -- the one flag that silently destroys all data")
print("-" * 78)
# ===========================================================================

good_frame = np.full((LEPTON.height, LEPTON.width), 29315, dtype=np.uint16)
T.require_raw_frame_form(LEPTON, good_frame)
check("a correct uint16 frame passes the raw-form assertion", True)

# THE FAILURE THIS EXISTS FOR. Without cv2.CAP_PROP_CONVERT_RGB = 0, read()
# returns 3-channel uint8: a perfectly pretty image with the upper bits thrown
# away and NO ERROR RAISED ANYWHERE. Every downstream stage keeps working and
# every temperature is wrong.
converted = np.zeros((LEPTON.height, LEPTON.width, 3), dtype=np.uint8)
ok, msg = refuses(T.require_raw_frame_form, LEPTON, converted)
check("a 3-channel uint8 frame is refused", ok)
check("...naming CAP_PROP_CONVERT_RGB explicitly", "CONVERT_RGB" in msg)
check("...and saying the upper bits are gone", "UPPER BITS" in msg.upper())

# Some paths return single-channel 8-bit instead, which the ndim check misses.
eight_bit = np.zeros((LEPTON.height, LEPTON.width), dtype=np.uint8)
ok, msg = refuses(T.require_raw_frame_form, LEPTON, eight_bit)
check("a 2-D uint8 frame from a 16-bit device is refused", ok)
check("...and points at the raw-unlock control as the other cause",
      "CONVERT_RGB" in msg or "raw mode" in msg)

# A silent resolution fallback is caught too - "log the negotiated fourcc and
# geometry, every time" is only useful if something also checks it.
ok, _ = refuses(T.require_raw_frame_form, LEPTON,
                np.zeros((240, 320), dtype=np.uint16))
check("a frame of the wrong geometry is refused", ok)

# A metadata-only V4L2 node opens, reports isOpened(), and yields nothing.
ok, msg = refuses(T.require_raw_frame_form, LEPTON, None)
check("a None frame is refused, naming metadata-only nodes", ok)
check("...explicitly", "METADATA ONLY" in msg.upper())

# The guard is wired into normalise(), not just available beside it.
ok, _ = refuses(T.normalise, LEPTON, converted, ASSERTED)
check("normalise() applies the raw-form guard to arrays too", ok)

# CONSTRAINT 1's cheapest guard: a 640x480 YUYV colour frame is 614400 bytes
# and a 160x120 Y16 frame is 38400, so a drifted binding is off by 16x.
ok, msg = refuses(T.normalise, LEPTON, b"\x00" * (640 * 480 * 2), ASSERTED)
check("an RGB-sized buffer on the thermal device is refused", ok)
check("...and is called out as a BINDING error, not a short read",
      "BINDING ERROR" in msg)

# The capture contract is how this descriptor-only module tells the node
# author what to set, so nobody has to remember the flag.
cc = T.capture_contract(LEPTON)
check("capture_contract demands CONVERT_RGB be disabled for a 16-bit device",
      cc.convert_rgb_must_be_disabled is True)
check("capture_contract states the expected dtype",
      cc.expected_dtype == "uint16", cc.expected_dtype)
check("capture_contract carries the exact frame byte count",
      cc.expected_bytes == 160 * 120 * 2, str(cc.expected_bytes))
t2s = db.get("infiray-t2splus-ht301")
check("capture_contract carries the vendor magic controls",
      any(c.value == 0x8004 for c in T.capture_contract(t2s).controls))
check("...including the shutter/FFC trigger",
      any(c.value == 0x8000 for c in T.capture_contract(t2s).controls))


# ===========================================================================
print()
print("G. CONVERSION, EMISSIVITY AND THE CEILING")
print("-" * 78)
# ===========================================================================

conv = LEPTON.conversion

# The arithmetic, against a hand-computed value: 29315 counts * 0.01 K = 293.15
# K = 20.00 C. A test that cannot be checked by hand is not much of a test.
apparent = conv.apparent_celsius(29315)
check("29315 counts at 0.01 K/count is 20.00 C", abs(apparent - 20.0) < 1e-6,
      f"{apparent:.4f} C")

# THE TLINEAR SCALE TRAP: the same counts through the other scale differ by
# exactly ten times in kelvin. This is why resolve() refuses the ambiguity.
lo = T.RadiometricConversion(kind=T.CONVERSION_LINEAR, kelvin_per_count=0.1,
                             kelvin_at_zero_counts=0.0,
                             max_measurable_c=140.0, source="the other scale")
k_hi = conv.apparent_kelvin(29315)
k_lo = lo.apparent_kelvin(29315)
check("the two TLinear scales differ by exactly 10x in kelvin",
      abs(k_lo / k_hi - 10.0) < 1e-9, f"{k_hi:.1f} K vs {k_lo:.1f} K")

# Emissivity 1.0 is the identity, exactly.
r = T.counts_to_celsius(29315, conv, BLACKBODY, device_key="t")
check("emissivity 1.0 leaves the apparent temperature unchanged",
      abs(r.celsius - 20.0) < 1e-9, f"{r.celsius:.6f} C")

# THE MAGNITUDE CLAIM, ASSERTED RATHER THAN DESCRIBED. Apparent 80 C over a
# 25 C room: assuming 0.95 for a surface that is really 0.10 bare aluminium
# does not shift the answer by a few degrees.
c80 = counts_for_c(conv, 80.0)
painted = T.counts_to_celsius(c80, conv, PAINTED, device_key="t").celsius
shiny = T.counts_to_celsius(c80, conv, SHINY, device_key="t").celsius
check("a wrong emissivity is worth far more than a few degrees",
      (shiny - painted) > 20.0,
      f"e=0.95 -> {painted:.1f} C, e=0.10 -> {shiny:.1f} C "
      f"(delta {shiny - painted:.1f})")
# ...and it errs in the direction that INVENTS a fire, which is why it cannot
# be given a friendly default.
check("...and the error direction invents heat, not cold", shiny > painted)

# There is no path that omits the compensation. `counts_to_celsius` takes it
# positionally and required, so "just give me the temperature" does not exist.
try:
    T.counts_to_celsius(29315, conv)     # type: ignore[call-arg]
    omitted = True
except TypeError:
    omitted = False
check("counts_to_celsius cannot be called without a compensation", not omitted)
ok, _ = refuses(T.counts_to_celsius, 29315, conv, None)
check("...and an explicit None compensation is refused too", ok)

# 0.95 assumed for a surface nobody characterised is the failure; an
# emissivity outside (0, 1] is a unit or a typo.
for bad in (0.0, 1.5, -0.2):
    ok, _ = refuses(T.EmissivityCompensation, emissivity=bad, ambient_c=25.0)
    check(f"emissivity {bad} is refused", ok)
ok, _ = refuses(T.EmissivityCompensation, emissivity=0.9, ambient_c=-300.0)
check("an ambient below absolute zero is refused (kelvin passed as celsius)",
      ok)

# An apparent radiance below what the reflected background alone contributes
# is not a cold object, it is an impossible one. A NaN or a clamped floor here
# would hand a plausible number to a fire alert.
hot_bg = T.EmissivityCompensation(emissivity=0.1, ambient_c=200.0,
                                  source="test")
ok, msg = refuses(T.counts_to_celsius, counts_for_c(conv, 20.0), conv, hot_bg,
                  device_key="t")
check("an unphysical emissivity/ambient combination is refused", ok)
check("...telling the operator ambient means the REFLECTED background",
      "REFLECTED" in msg)

# THE DOUBLE-CORRECTION GUARD. The Melexis driver already applies emissivity,
# so applying it again is a silent, physically-shaped error indistinguishable
# from a miscalibrated sensor.
mlx = db.get("mlx90640")
check("the MLX90640 entry declares emissivity applied upstream",
      mlx.conversion.emissivity_applied_upstream is True)
ok, msg = refuses(T.counts_to_celsius, 25.0, mlx.conversion, PAINTED,
                  device_key="mlx90640")
check("a second emissivity correction on the MLX90640 is refused", ok)
check("...telling the caller to pass emissivity to the driver instead",
      "DRIVER" in msg)
r = T.counts_to_celsius(25.0, mlx.conversion, BLACKBODY, device_key="mlx90640")
check("...while emissivity 1.0 says 'already corrected' and works",
      abs(r.celsius - 25.0) < 1e-9, f"{r.celsius:.4f} C")

# SATURATION: SURVEY FINDING 3. At or above the ceiling there is no
# temperature, only "at least this hot" - and a clamped ceiling value would
# flow through a threshold comparison and a JSON payload without a murmur.
sat = T.counts_to_celsius(counts_for_c(conv, 150.0), conv, BLACKBODY,
                          device_key="t")
check("a reading above the ceiling reports SATURATED", sat.saturated is True)
check("...with celsius None, NOT a clamped ceiling value",
      sat.celsius is None, repr(sat.celsius))
ok, msg = refuses(sat.celsius_or_refuse)
check("...and celsius_or_refuse() refuses rather than returning a number", ok)
check("...and it is not treated as evidence", sat.is_evidence is False)
# Just below the ceiling is a real reading, so the boundary is where it says.
near = T.counts_to_celsius(counts_for_c(conv, 139.0), conv, BLACKBODY,
                           device_key="t")
check("just below the ceiling is still a real reading",
      near.saturated is False and abs(near.celsius - 139.0) < 1e-6)

# UNVERIFIED IS CARRIED OUTWARD, so a publisher cannot lose it.
check("an unverified conversion is never 'evidence'",
      near.is_evidence is False)
check("...and says so in the caveats",
      any("UNVERIFIED" in c for c in near.caveats))

# THRESHOLD vs CEILING, chosen together (survey §10.4).
amg = db.get("amg8833")
check("the AMG8833 ceiling is 80 C", amg.conversion.max_measurable_c == 80.0)
check("...so it cannot survive a flame",
      amg.conversion.survives_flame is False)
ok, msg = refuses(T.check_threshold, amg, 150.0)
check("a 150 C threshold on an 80 C sensor is refused", ok)
check("...because it could NEVER be crossed", "NEVER" in msg)
T.check_threshold(amg, 70.0)
check("a 70 C threshold on the AMG8833 is accepted", True)
# A threshold a sunlit cone can reach confirms exactly the false positive the
# colour screen already makes.
ok, msg = refuses(T.check_threshold, amg, 50.0)
check("a threshold a sunlit cone reaches is refused", ok)
p2 = db.get("infiray-p2pro")
check("the InfiRay ceiling survives a flame",
      p2.conversion.survives_flame is True,
      f"{p2.conversion.max_measurable_c:g} C")
T.check_threshold(p2, 150.0)
check("a 150 C threshold on the InfiRay is accepted", True)
# The gate runs before the ceiling test, so an unproven device cannot have a
# threshold validated against a capability it has not demonstrated.
ok, _ = refuses(T.check_threshold, LEPTON, 100.0)
check("check_threshold refuses on an unproven device", ok)

# A device_metadata conversion must refuse rather than approximate.
ok, msg = refuses(t2s.require_measurement,
                  T.RadiometryAssertion(device_key="infiray-t2splus-ht301",
                                        confirmed=True, method="m",
                                        evidence="e", asserted_by="a"))
check("a proven device_metadata device still refuses to convert here", ok)
check("...pointing at the metadata rows and the missing decoder",
      "metadata" in msg and "decoder" in msg)


# ===========================================================================
print()
print("H. FRAME NORMALISATION AND THE PLANE SPLITTER")
print("-" * 78)
# ===========================================================================

# Canonical form for a proven radiometric device: 16-bit counts.
rframe = T.normalise(LEPTON, buf, ASSERTED)
check("radiometric canonical form is 16-bit mono",
      rframe.counts.dtype == np.uint16, str(rframe.counts.dtype))
check("...at the device's geometry", rframe.counts.shape == (120, 160),
      str(rframe.counts.shape))
# Canonical form for everything else: 8-bit.
check("AGC canonical form is 8-bit mono",
      agc_frame.mono8.dtype == np.uint8, str(agc_frame.mono8.dtype))

# The known hot pixel comes back where it was put, at the value it was given.
hot = rframe.temperature_at(80, 60, BLACKBODY)
check("the planted 100 C pixel reads back as 100 C",
      abs(hot.celsius - 100.0) < 0.02, f"{hot.celsius:.3f} C")
bg = rframe.temperature_at(0, 0, BLACKBODY)
check("the 20 C background reads back as 20 C",
      abs(bg.celsius - 20.0) < 0.02, f"{bg.celsius:.3f} C")
check("max_celsius finds the hot pixel",
      abs(rframe.max_celsius(BLACKBODY).celsius - 100.0) < 0.02)
hs = rframe.hotspot(50.0, BLACKBODY)
check("hotspot counts exactly the one pixel above 50 C",
      hs.pixel_count == 1, f"{hs.pixel_count} of {hs.total_pixels}")
check("...and reports where it is", hs.max_xy == (80, 60), str(hs.max_xy))

# Indexing past the DATA plane must refuse: on a split-frame device the data
# plane is shorter than the delivered frame, so a y taken from the full height
# indexes the wrong rows.
ok, _ = refuses(rframe.temperature_at, 0, 200, BLACKBODY)
check("a pixel outside the data plane is refused", ok)

# An all-zero frame is what a disconnected or mis-bound sensor produces, and
# it converts to a perfectly plausible uniform field.
zero = T.normalise(db.get("flir-lepton-purethermal-tlinear-10mk"),
                   b"\x00" * 38400,
                   T.RadiometryAssertion(
                       device_key="flir-lepton-purethermal-tlinear-10mk",
                       confirmed=True, method="m", evidence="e",
                       asserted_by="a"))
check("an all-zero frame is flagged out of the documented count range",
      zero.counts_in_range is False)

# THE SPLITTER. The InfiRay P2 Pro delivers ONE 256x384 buffer: top half AGC
# picture, bottom half raw 16-bit. If the split were wrong, the picture half
# (filled with 0xFFFF here = 1024 K = 750 C) would saturate the 550 C ceiling.
top = np.full((192, 256), 0xFFFF, dtype="<u2")
bottom = np.full((192, 256), int(counts_for_c(p2.conversion, 100.0)),
                 dtype="<u2")
split_buf = np.vstack([top, bottom]).tobytes()
check("the split buffer is the declared frame size",
      len(split_buf) == p2.frame_bytes, f"{len(split_buf)} B")
sframe = T.normalise(p2, split_buf)
check("a split-frame device yields a RadiometricFrame",
      isinstance(sframe, T.RadiometricFrame))
check("...whose counts are the BOTTOM half only",
      sframe.counts.shape == (192, 256), str(sframe.counts.shape))
mx = sframe.max_celsius(BLACKBODY)
check("...so the picture half does not leak into the measurement",
      mx.saturated is False and abs(mx.celsius - 100.0) < 0.1,
      f"{mx.celsius if mx.celsius is not None else 'SATURATED'}")
check("...and the co-registered picture is carried alongside",
      isinstance(sframe.companion_agc, T.AgcFrame))
check("...as an AgcFrame with no temperature method",
      not hasattr(sframe.companion_agc, "temperature_at"))

# METADATA ROWS ARE COEFFICIENTS, NOT PIXELS. Reading them as counts yields
# wild values that look exactly like a fire.
meta_buf = b"\x00" * t2s.frame_bytes
mframe = T.normalise(t2s, meta_buf)
check("a metadata-carrying device exposes its metadata rows undecoded",
      mframe.metadata_rows is not None
      and mframe.metadata_rows.shape == (4, 256),
      str(None if mframe.metadata_rows is None else mframe.metadata_rows.shape))
check("...while still refusing to report any temperature",
      isinstance(mframe, T.AgcFrame))

# I2C sensors hand over an already-shaped numeric array, not a byte buffer.
grid = np.full((8, 8), 160.0)        # 160 * 0.25 K + 273.15 = 313.15 K = 40 C
gframe = T.normalise(amg, grid)
check("an I2C RAW_COUNTS array normalises",
      isinstance(gframe, T.RadiometricFrame))
check("...to 40 C as the datasheet LSB implies",
      abs(gframe.temperature_at(0, 0, BLACKBODY).celsius - 40.0) < 1e-6)
# A partial I2C read must never be padded: a padded frame has real-looking
# cold pixels.
ok, msg = refuses(T.normalise, amg, np.zeros(40))
check("a short I2C read is refused, not padded", ok)
check("...naming the padded-cold-pixels failure", "padded" in msg)

# A whole-frame saturated field: every pixel at the ceiling. The point of
# finding 3 is that this looks identical to a merely-warm frame otherwise.
hotgrid = np.full((8, 8), 400.0)     # 400 * 0.25 + 273.15 = 373.15 K = 100 C
hframe = T.normalise(amg, hotgrid)
sat_field = hframe.celsius_field(BLACKBODY)
check("a fully saturated field reports saturated_fraction 1.0",
      abs(sat_field.saturated_fraction - 1.0) < 1e-9)
check("...with the saturated pixels NaN rather than clamped",
      bool(np.isnan(np.asarray(sat_field.celsius)).all()))
hs2 = hframe.hotspot(70.0, BLACKBODY)
check("...and hotspot still counts them as above threshold",
      hs2.pixel_count == 64, f"{hs2.pixel_count}")
check("...while giving no max temperature for them",
      hs2.max_celsius is None)


# ===========================================================================
print()
print("I. DISPLAY-ONLY COLOURISATION")
print("-" * 78)
# ===========================================================================

img = T.colourise_for_display(rframe)
check("colourise returns a DisplayOnlyImage wrapper",
      isinstance(img, T.DisplayOnlyImage))
# THE WRAPPER IS THE POINT. A bare (H, W, 3) uint8 array is exactly what
# detect_fire() accepts, so returning one would make the shortest path from
# "we have a thermal camera" to "the colour detector is hunting our own
# palette and reporting it as corroboration".
check("...not a bare numpy array", not isinstance(img, np.ndarray))
check("...with the pixels behind an explicitly named attribute",
      img.rgb.shape == (120, 160, 3), str(img.rgb.shape))
check("...and no temperature on the display object",
      not hasattr(img, "temperature_at") and not hasattr(img, "celsius"))
check("display_only is a class attribute and cannot be constructed False",
      T.DisplayOnlyImage.display_only is True)
check("the caveat forbids feeding it to detect_fire",
      "detect_fire" in img.caveat)
# An unlabelled false-colour image is read as an absolute temperature scale by
# everyone who looks at it - the human version of the AGC mistake.
check("an auto-stretched image says it is not comparable frame to frame",
      "NOT comparable" in img.span_note)
fixed = T.colourise_for_display(rframe, span=(0.0, 65535.0))
check("a fixed span says it IS comparable", "comparable frame to frame"
      in fixed.span_note)
check("an AGC frame colourises too, with its own caveat",
      "AGC source" in T.colourise_for_display(agc_frame).caveat)
ok, _ = refuses(T.colourise_for_display, np.zeros((4, 4)))
check("colourising a bare array is refused (it loses the device identity)", ok)


# ===========================================================================
print()
print("=" * 78)
if FAILURES:
    print(f"  {len(FAILURES)} of {CHECKS[0]} checks FAILED")
    for f in FAILURES:
        print(f"    - {f}")
    print("=" * 78)
    sys.exit(1)
print(f"  all {CHECKS[0]} checks passed")
print("  NOTHING HERE TOUCHED A THERMAL CAMERA. Every frame was synthesised")
print("  from a formula, every device entry is UNVERIFIED, and every")
print("  conversion carries verified:false. These checks prove the module")
print("  refuses what it should refuse - not that any camera works.")
print("=" * 78)
sys.exit(0)
