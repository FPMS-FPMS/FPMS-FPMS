#!/usr/bin/env python3
"""Offline correctness checks for the YOLO26 decode. No Pi, no NPU, no model.

    python3 npu/tests/test_decode.py

WHY THIS FILE EXISTS
--------------------
Every decode bug this project has had produced *plausible-looking boxes*, not
an exception. A crash is cheap: the agent logs "inference failed", the
dashboard shows no boxes, somebody investigates. A decode bug is expensive:
the rover draws confident rectangles in the wrong place, or at the wrong
threshold, and every layer above it -- the dashboard, the telemetry, the
operator -- reports success. NPU_SPEC.md calls that class of failure "the
single worst failure mode in the project, because it is indistinguishable
from success at every level an operator normally checks."

So this file is not here for coverage. It is here to pin down the handful of
facts that, if quietly violated, produce a rover that looks like it is working.
Each check names the real failure it guards against.

RELATIONSHIP TO rover/test_yolo26_decode.py
-------------------------------------------
That file exists and holds 22 checks: round-trip through the rescale, both
tensor orientations, threshold, NMS duplicates, per-class NMS, clipping,
dtypes. Those are still the right checks and they are NOT repeated here.

This file covers the ground that one does not:

  * the double-sigmoid trap, tested as a *discriminating* case rather than
    an equality that a double sigmoid could coincidentally survive
  * stretch-vs-letterbox rescaling proved with a NON-SQUARE frame and an
    OFF-CENTRE box -- the only geometry where the two disagree
  * the [1,300,6] end2end=True tensor being refused rather than misparsed
  * numerical edges: all-zero, NaN, Inf, saturation, class 0 and class 79
  * the anchor count, which the decoder does not currently check at all

Run both. Neither supersedes the other.

WHAT THIS FILE CANNOT CHECK -- see README.md for the full list. In short:
it proves the decode is self-consistent against a synthesised tensor. It
cannot prove the real .rknn emits that tensor. Nothing here has run on an
RK3588S; the board has not arrived.
"""

import math
import os
import sys
import time

try:
    import numpy as np
except ImportError:
    sys.stderr.write("numpy is required (numpy<2 per NPU_SPEC.md). Aborting.\n")
    sys.exit(2)

# tests -> npu -> fpms-os -> rover/, where fpms_yolo26_npu.py lives.
HERE = os.path.dirname(os.path.abspath(__file__))
ROVER = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

failures, warnings, checks_run = [], [], [0]


def fail(name, msg):
    failures.append((name, msg))


def warn(name, msg):
    warnings.append((name, msg))


def check(name, cond, msg=""):
    """One assertion. Counted whether it passes or not."""
    checks_run[0] += 1
    if not cond:
        fail(name, msg or "assertion failed")
    return bool(cond)


# --------------------------------------------------------------------------
# Import the module under test, and be explicit if we cannot.
# --------------------------------------------------------------------------
def _load():
    """Import fpms_yolo26_npu without needing rknnlite, cv2, or the Pi.

    The module is deliberately numpy-only -- it holds no import of rknnlite
    and no filesystem access at import time (MODEL is a string constant, not
    an open()). If that ever stops being true this import breaks, and it must
    break with a message that says which dependency crept in, not a bare
    traceback from three frames deep.
    """
    sys.path.insert(0, ROVER)
    try:
        import fpms_yolo26_npu as mod
        return mod, None
    except ImportError as exc:
        return None, (
            "cannot import fpms_yolo26_npu from %s: %r.\n"
            "  The decode module is supposed to be numpy-only so it can be "
            "tested and reviewed off the Pi. If a hardware import (rknnlite, "
            "cv2) has been added at module scope, that is the bug -- move it "
            "inside the function that needs it." % (ROVER, exc)
        )
    except Exception as exc:  # noqa: BLE001
        return None, (
            "fpms_yolo26_npu raised at import time: %r.\n"
            "  Import must have no side effects -- no file reads, no device "
            "opens. Anything else means the decode cannot be exercised "
            "without the hardware it is meant to be independent of." % exc
        )


Y, IMPORT_ERROR = _load()


# --------------------------------------------------------------------------
# Tensor synthesis helpers. Everything below builds the ONNX layout the real
# graph emits, (84, 8400), and lets decode() handle the batch axis.
# --------------------------------------------------------------------------
def blank(channels=None, anchors=None):
    """An all-zero prediction in the [84, 8400] ONNX layout.

    All-zero matters on its own: channels 4..83 arrive ALREADY SIGMOIDED, so
    zero means probability zero, and an all-zero tensor must yield zero
    detections. If anyone ever applies sigmoid inside decode(), zero becomes
    0.5, which is above CONF_THRES=0.35, and this same tensor yields 8400
    detections. See test_no_double_sigmoid.
    """
    c = Y.NUM_CH if channels is None else channels
    a = Y.NUM_ANCHORS if anchors is None else anchors
    return np.zeros((c, a), dtype=np.float32)


def put(pred, anchor, cx, cy, w, h, cls, conf):
    """Write one detection into an anchor column, in 640x640 input pixels."""
    pred[0, anchor], pred[1, anchor] = cx, cy
    pred[2, anchor], pred[3, anchor] = w, h
    pred[4 + cls, anchor] = conf
    return pred


def dec(pred, ih, iw, **kw):
    """Call decode the way the agent does: a list holding a batched tensor.

    fpms_rover_agent.py line ~604:  decode(outputs, ih, iw) where outputs is
    the list returned by rknn.inference(). Test the real calling convention,
    not a convenient one.
    """
    return Y.decode([pred[None, ...]], ih, iw, **kw)


def sigmoid(x):
    return 1.0 / (1.0 + math.exp(-x))


# ==========================================================================
# 1. The shape contract.  output0 is [1, 84, 8400] and nothing else.
# ==========================================================================
def test_output_shape_contract():
    """A wrong-shaped tensor must raise, never decode into plausible boxes.

    GUARDS AGAINST: a re-export that changes the head layout. NPU_SPEC.md
    fixes output0 at [1,84,8400] = 4 box + 80 class over 80*80+40*40+20*20
    anchors. A decoder that shrugs at an unexpected shape is how a
    wrong-version model "loads and produces garbage boxes" (SPEC section 0).
    """
    check("shape.num_ch", Y.NUM_CH == 84, "NUM_CH=%r, spec says 84" % Y.NUM_CH)
    check("shape.num_classes", Y.NUM_CLASSES == 80, "NUM_CLASSES=%r" % Y.NUM_CLASSES)
    check("shape.num_anchors", Y.NUM_ANCHORS == 8400,
          "NUM_ANCHORS=%r, spec says 8400" % Y.NUM_ANCHORS)
    check("shape.input_size", Y.INPUT_SIZE == 640, "INPUT_SIZE=%r" % Y.INPUT_SIZE)
    check("shape.anchor_sum", 80 * 80 + 40 * 40 + 20 * 20 == Y.NUM_ANCHORS,
          "the P3/P4/P5 anchor counts no longer sum to NUM_ANCHORS")

    # The canonical tensor must decode without complaint.
    p = put(blank(), 0, 320.0, 320.0, 40.0, 40.0, cls=0, conf=0.9)
    try:
        b, s, c = dec(p, 480, 640)
        check("shape.canonical_accepted", b.shape == (1, 4), "got %r" % (b.shape,))
    except Exception as exc:  # noqa: BLE001
        check("shape.canonical_accepted", False,
              "the documented [1,84,8400] layout raised %r" % exc)

    # Shapes that must be refused.
    bad = [
        ("v8_85ch", (1, 85, 8400), "an 80-class v8 export with objectness"),
        ("v5_25200", (1, 25200, 85), "a v5-style output"),
        ("wrong_ch_64", (1, 64, 8400), "a 60-class model"),
        ("3d_branches", (1, 3, 80, 80, 85), "a split-head export"),
        ("1d", (8400,), "a flattened tensor"),
    ]
    for name, shape, why in bad:
        try:
            Y.decode([np.zeros(shape, np.float32)], 480, 640)
            check("shape.reject_" + name, False,
                  "%s (%s) decoded WITHOUT raising -- it would emit boxes from "
                  "a tensor this decoder does not understand" % (str(shape), why))
        except ValueError:
            check("shape.reject_" + name, True)
        except Exception as exc:  # noqa: BLE001
            # Still refused, but with the wrong exception type; the agent
            # catches bare Exception so this is survivable, not silent.
            check("shape.reject_" + name, True)
            warn("shape.reject_" + name,
                 "refused with %s, not ValueError -- the docstring promises "
                 "ValueError" % type(exc).__name__)


def test_end2end_tensor_is_refused():
    """[1,300,6] from end2end=True must be REFUSED, not misparsed.

    GUARDS AGAINST: the most dangerous plausible-looking output in the whole
    stack. NPU_SPEC.md: "end2end=false is mandatory. Ultralytics' RKNN docs
    report end2end=True emits [1,300,6] whose top-k op segfaults on the NPU."

    [1,300,6] is x1,y1,x2,y2,score,class for 300 detections. It is already
    decoded, already NMS'd, and in a DIFFERENT box format (corners, not
    centres). If this decoder ever accepted it, every one of the 300 rows
    would be re-read as xywh centre form and re-scaled -- producing 300
    boxes that are the right count, the right dtype, and geometrically
    nonsense. Nothing downstream would notice.

    Neither axis of (300, 6) is 84, so _as_anchor_major raises. This check
    pins that behaviour so a future "be permissive about layouts" change
    cannot quietly open the door.
    """
    try:
        Y.decode([np.zeros((1, 300, 6), np.float32)], 480, 640)
        check("end2end.refused", False,
              "[1,300,6] decoded without raising. An end2end=True export "
              "would produce 300 garbage boxes instead of a hard failure.")
    except ValueError:
        check("end2end.refused", True)
    except Exception as exc:  # noqa: BLE001
        check("end2end.refused", True)
        warn("end2end.refused", "refused with %s, not ValueError" % type(exc).__name__)

    # A populated one, in case an all-zero tensor is refused for the wrong
    # reason (e.g. an emptiness short-circuit rather than a shape check).
    e2e = np.zeros((1, 300, 6), np.float32)
    e2e[0, :, :4] = [10.0, 20.0, 110.0, 120.0]
    e2e[0, :, 4] = 0.9
    e2e[0, :, 5] = 0.0
    try:
        Y.decode([e2e], 480, 640)
        check("end2end.refused_populated", False,
              "a populated [1,300,6] decoded without raising")
    except Exception:  # noqa: BLE001
        check("end2end.refused_populated", True)


def test_anchor_count_is_not_validated():
    """The decoder checks the 84 axis but NEVER the 8400 axis. Documented gap.

    GUARDS AGAINST: nothing yet -- this check exists to record a real hole.

    NUM_ANCHORS is defined in fpms_yolo26_npu.py and then never used by
    decode(). So a model exported at 320x320 emits [1,84,2100], which
    _as_anchor_major accepts happily (one axis is 84), and decode() then
    scales the box coordinates by iw/640 when they are in 320-pixel units.
    Every box comes back at half size, centred half-way toward the top-left.
    Boxes appear. Scores look normal. Classes are right. It is exactly the
    "plausible-looking box" failure this file exists to catch, and the
    decoder currently cannot see it.

    This check asserts the CURRENT behaviour (accepted) and warns. If someone
    adds the anchor-count guard, this check flips to the strict branch and
    the warning disappears -- deliberately, so tightening the decoder does
    not look like a regression.
    """
    p320 = put(blank(anchors=2100), 0, 160.0, 160.0, 40.0, 40.0, cls=0, conf=0.9)
    try:
        b, _, _ = Y.decode([p320[None, ...]], 480, 640)
        check("anchors.wrong_count_behaviour", True)
        warn("anchors.wrong_count",
             "a [1,84,2100] tensor (a 320x320 export) decoded into %d box(es) "
             "instead of raising. NUM_ANCHORS is defined but unused in "
             "decode(). Consider rejecting any anchor axis != NUM_ANCHORS: "
             "the resulting boxes are half-size and mispositioned, which "
             "survives a visual check." % b.shape[0])
    except ValueError:
        check("anchors.wrong_count_behaviour", True)  # guard has been added


# ==========================================================================
# 2. THE DOUBLE SIGMOID.  Classes arrive already sigmoided.
# ==========================================================================
def test_no_double_sigmoid():
    """Class channels are ALREADY probabilities. Sigmoiding again is silent.

    GUARDS AGAINST: the single most insidious change anyone could make to
    this file. NPU_SPEC.md and the module docstring both state the two
    Softmax ops of model.23 fold DFL into the graph and channels 4..83 have
    already been through Sigmoid.

    A second sigmoid does not error. It maps [0,1] onto [0.5, 0.731] --
    it compresses every score toward 0.5 and, crucially, LIFTS every zero to
    0.5. Against CONF_THRES=0.35 that means:

        - a real 0.90 detection is reported as 0.71 (operator sees a weaker
          but still-present box: no alarm)
        - a genuine 0.00 non-detection becomes 0.50, ABOVE THRESHOLD

    So the visible symptom is not "detections vanish". It is "everything is
    a detection, at middling confidence" -- and after NMS collapses them,
    a handful of confident-looking phantom boxes. On a fire-detection rover
    that is a false alarm generator that never crashes.

    Three independent discriminators below. Any one alone could in principle
    be satisfied by a coincidence; together they cannot.
    """
    IH, IW = 480, 640

    # (a) The all-zero tensor. Under the correct reading: no detections.
    #     Under a double sigmoid: sigmoid(0)=0.5 >= 0.35 for all 8400 anchors.
    b, s, c = dec(blank(), IH, IW)
    check("sigmoid.zero_tensor_empty", b.shape[0] == 0,
          "an all-zero output produced %d detection(s) at scores %r. "
          "sigmoid(0)=0.5 which clears CONF_THRES=%.2f -- this is the exact "
          "signature of a double sigmoid."
          % (b.shape[0], s[:5].tolist() if s.size else [], Y.CONF_THRES))

    # (b) Score identity. A stored 0.90 must come back as 0.90, not
    #     sigmoid(0.90)=0.7109. Tolerance far tighter than the gap.
    p = put(blank(), 100, 320.0, 320.0, 64.0, 64.0, cls=0, conf=0.90)
    b, s, c = dec(p, IH, IW)
    if check("sigmoid.identity_returns_one", b.shape[0] == 1, "got %d" % b.shape[0]):
        got = float(s[0])
        check("sigmoid.score_passthrough", abs(got - 0.90) < 1e-6,
              "stored probability 0.90 came back as %.6f. sigmoid(0.90)=%.6f "
              "-- if that is what you see, decode() is sigmoiding an "
              "already-sigmoided tensor." % (got, sigmoid(0.90)))

    # (c) The logit-vs-probability discriminator. Feed a value that is a
    #     plausible LOGIT for a strong detection (3.0 -> p=0.953) and a value
    #     that is a plausible PROBABILITY (0.40). Under the correct reading
    #     both clear the 0.35 threshold and 3.0 wins. Under a double sigmoid
    #     they become 0.953 and 0.599 -- still both above threshold, still in
    #     the same order. THE ORDER IS NOT DIAGNOSTIC. Only the value is.
    p = blank()
    put(p, 200, 100.0, 100.0, 40.0, 40.0, cls=0, conf=3.0)
    b, s, c = dec(p, IH, IW)
    if check("sigmoid.logit_case_returns_one", b.shape[0] == 1, "got %d" % b.shape[0]):
        got = float(s[0])
        check("sigmoid.raw_value_not_squashed", abs(got - 3.0) < 1e-5,
              "a channel value of 3.0 came back as %.6f. The decode must pass "
              "the tensor through untouched; %.6f is sigmoid(3.0)."
              % (got, sigmoid(3.0)))
        # A score above 1.0 is impossible from a correctly-exported graph.
        # decode() has no business clamping it, but the AGENT rounds it into
        # telemetry, so record that it is reported honestly rather than
        # silently clipped into a believable range.
        check("sigmoid.no_silent_clamp", got > 1.0,
              "a >1.0 score was clamped to %.4f. Clamping makes an impossible "
              "value look legitimate; a model emitting logits would then be "
              "undetectable from the outside." % got)

    # (d) Threshold placement is only meaningful if scores are untransformed.
    #     A value one epsilon under the threshold must be dropped, and one
    #     epsilon over must be kept. Under a double sigmoid BOTH are kept
    #     (sigmoid(0.34)=0.584 and sigmoid(0.36)=0.589, both > 0.35).
    p = put(blank(), 300, 320.0, 320.0, 40.0, 40.0, cls=0, conf=Y.CONF_THRES - 0.01)
    b, _, _ = dec(p, IH, IW)
    check("sigmoid.just_under_threshold_dropped", b.shape[0] == 0,
          "conf=%.3f (below CONF_THRES=%.2f) survived. Under a double "
          "sigmoid every value in [0,1] lands in [0.5,0.731] and NOTHING is "
          "ever below threshold." % (Y.CONF_THRES - 0.01, Y.CONF_THRES))


def test_threshold_boundary_is_inclusive():
    """conf == CONF_THRES is KEPT. The comparison is `>=`, not `>`.

    GUARDS AGAINST: a silent one-epsilon threshold shift. Not dangerous on
    its own, but it pins the operator-facing meaning of the number: "0.35"
    in config means detections at 0.35 are reported. If someone tunes the
    threshold by editing the comparison rather than the constant, this
    catches it.
    """
    IH, IW = 480, 640
    for delta, want, label in ((-1e-6, 0, "just_below"),
                               (0.0, 1, "exactly_at"),
                               (+1e-6, 1, "just_above")):
        p = put(blank(), 5, 320.0, 320.0, 40.0, 40.0,
                cls=0, conf=Y.CONF_THRES + delta)
        b, _, _ = dec(p, IH, IW)
        check("threshold." + label, b.shape[0] == want,
              "conf=CONF_THRES%+g gave %d detections, expected %d"
              % (delta, b.shape[0], want))

    # The caller-supplied override must actually be honoured -- the agent
    # does not use it today, but fpms-npud is specified to.
    p = put(blank(), 5, 320.0, 320.0, 40.0, 40.0, cls=0, conf=0.20)
    b, _, _ = dec(p, IH, IW, conf_thres=0.10)
    check("threshold.override_honoured", b.shape[0] == 1,
          "conf_thres=0.10 did not admit a 0.20 detection (got %d) -- the "
          "keyword argument is being ignored in favour of the module "
          "constant" % b.shape[0])


# ==========================================================================
# 3. THE STRETCH RESCALE.  The highest-value test in this file.
# ==========================================================================
def test_stretch_not_letterbox():
    """Rescaling is an INDEPENDENT per-axis ratio. A letterbox is wrong here.

    GUARDS AGAINST: the failure that survives a visual check.

    fpms_rover_agent.py line 601 is literally:

        img = cv2.resize(frame, (640, 640))

    A plain resize. No aspect preservation, no padding. So the inverse is
    x_frame = x_640 * (iw/640) and y_frame = y_640 * (ih/640), computed
    INDEPENDENTLY. NPU_SPEC.md: "not a single uniform scale plus padding
    offset."

    Almost every YOLO decoder on the internet does the letterbox inverse
    instead, because almost every YOLO preprocessor letterboxes. Pasting one
    in here would produce boxes that are offset vertically and squashed --
    close enough to the object that a glance at the video says "working",
    wrong enough that the box centre is metres off in the world frame once
    the fire detector uses it.

    GEOMETRY NOTE, and the reason this test is built the way it is: at the
    CENTRE of the image the stretch and letterbox inverses AGREE exactly.
    A centred box proves nothing. The existing rover/test_yolo26_decode.py
    checks a centred box on a 640x480 frame; this check deliberately uses a
    16:9 frame and an OFF-CENTRE box, the only geometry that discriminates.
    """
    IH, IW = 720, 1280          # 16:9, the aspect the camera actually delivers

    # Off-centre box, in 640x640 input pixels. cy is chosen so that the
    # LETTERBOX answer also lands inside the frame: if the wrong answer were
    # negative it would be clipped to 0, and clipping would mask the very
    # discrepancy this test exists to detect. Both interpretations must be
    # legal coordinates for the comparison below to mean anything.
    #   stretch   -> y in [256.5, 328.5]
    #   letterbox -> y in [176.0, 304.0]     (both inside 0..719)
    cx, cy, w, h = 160.0, 260.0, 64.0, 64.0
    p = put(blank(), 42, cx, cy, w, h, cls=0, conf=0.90)
    b, s, c = dec(p, IH, IW)

    if not check("stretch.one_box", b.shape[0] == 1, "got %d" % b.shape[0]):
        return
    x1, y1, x2, y2 = [float(v) for v in b[0]]

    # What the stretch inverse must give.
    rx, ry = IW / 640.0, IH / 640.0          # 2.0 and 1.125
    want = ((cx - w / 2) * rx, (cy - h / 2) * ry,
            (cx + w / 2) * rx, (cy + h / 2) * ry)   # (256, 144, 384, 216)
    check("stretch.x1", abs(x1 - want[0]) < 1e-3, "got %.4f want %.4f" % (x1, want[0]))
    check("stretch.y1", abs(y1 - want[1]) < 1e-3, "got %.4f want %.4f" % (y1, want[1]))
    check("stretch.x2", abs(x2 - want[2]) < 1e-3, "got %.4f want %.4f" % (x2, want[2]))
    check("stretch.y2", abs(y2 - want[3]) < 1e-3, "got %.4f want %.4f" % (y2, want[3]))

    # And what a letterbox inverse WOULD have given, asserted to be absent.
    # letterbox: gain = 640/1280 = 0.5, the 720px height maps to 360px, so
    # pad = (640 - 360)/2 = 140 rows top and bottom.
    gain = 640.0 / IW
    pad = (640.0 - IH * gain) / 2.0
    # Clipped the same way decode() clips, so a letterbox answer that happens
    # to fall off the frame still compares against what would be RETURNED.
    lb_y1 = min(max((cy - h / 2 - pad) / gain, 0.0), IH - 1.0)
    lb_y2 = min(max((cy + h / 2 - pad) / gain, 0.0), IH - 1.0)
    check("stretch.not_letterbox_y1", abs(y1 - lb_y1) > 1.0,
          "y1=%.3f matches the LETTERBOX inverse (%.3f), not the stretch "
          "inverse (%.3f). Preprocessing is cv2.resize, not letterbox -- "
          "padding compensation must not be applied here."
          % (y1, lb_y1, want[1]))
    check("stretch.not_letterbox_y2", abs(y2 - lb_y2) > 1.0,
          "y2=%.3f matches the letterbox inverse (%.3f)" % (y2, lb_y2))

    # A uniform-scale (no padding) inverse is the other common wrong answer.
    check("stretch.not_uniform_scale", abs((y2 - y1) - h * ry) < 1e-3,
          "box height %.3f is not h*(ih/640)=%.3f. A uniform scale would "
          "give %.3f." % (y2 - y1, h * ry, h * rx))

    # Aspect ratio of the box must CHANGE, because the image was stretched.
    # A square box in 640-space is 128x72 in a 1280x720 frame.
    box_ar = (x2 - x1) / (y2 - y1)
    check("stretch.aspect_distorted", abs(box_ar - (IW / float(IH))) < 1e-3,
          "a square input box came back with aspect %.4f; a stretched 16:9 "
          "frame must give %.4f. An aspect of 1.0 means the rescale is "
          "preserving aspect, i.e. treating the input as a letterbox."
          % (box_ar, IW / float(IH)))


def test_rescale_axes_are_not_swapped():
    """x uses iw and y uses ih. The classic argument-order bug.

    GUARDS AGAINST: decode(outputs, ih, iw) taking height first is easy to
    get backwards, and on the rover's old 640x480 frame a swap is a mild
    distortion that still lands boxes roughly on objects. On a portrait or
    a widescreen frame it is catastrophic. Test with a frame whose two
    dimensions are far apart, in BOTH orientations, so a swap cannot hide.
    """
    for IH, IW in ((720, 1280), (1280, 720)):
        # Full-input box: covers the entire 640x640, so it must come back
        # covering the entire frame after clipping.
        p = put(blank(), 0, 320.0, 320.0, 640.0, 640.0, cls=0, conf=0.9)
        b, _, _ = dec(p, IH, IW)
        if not check("axes.full_box_%dx%d" % (IW, IH), b.shape[0] == 1):
            continue
        x1, y1, x2, y2 = [float(v) for v in b[0]]
        check("axes.width_%dx%d" % (IW, IH), abs((x2 - x1) - (IW - 1)) < 1e-3,
              "full-frame box spans %.2f px in x, frame width is %d. If this "
              "equals the HEIGHT, ih and iw are swapped." % (x2 - x1, IW))
        check("axes.height_%dx%d" % (IW, IH), abs((y2 - y1) - (IH - 1)) < 1e-3,
              "full-frame box spans %.2f px in y, frame height is %d"
              % (y2 - y1, IH))

    # Square frame: ratios are equal, so this can only catch gross errors --
    # included precisely to make the point that a square test is not enough.
    p = put(blank(), 0, 100.0, 500.0, 20.0, 20.0, cls=0, conf=0.9)
    b, _, _ = dec(p, 640, 640)
    check("axes.square_identity",
          b.shape[0] == 1 and abs(float(b[0][0]) - 90.0) < 1e-3
          and abs(float(b[0][1]) - 490.0) < 1e-3,
          "a 640x640 frame must be the identity rescale, got %r"
          % (b[0].tolist() if b.shape[0] else None))


def test_boxes_are_ordered_corners_inside_the_frame():
    """x1<=x2, y1<=y2, all within [0, dim-1]. The agent int()s these directly.

    GUARDS AGAINST: an inverted or out-of-range box reaching cv2.rectangle
    and the telemetry payload. fpms_rover_agent.py does
    `x1, y1, x2, y2 = [int(v) for v in b]` with no validation and puts the
    result straight into the MQTT detection record, so anything wrong here
    ships to the dashboard as fact.
    """
    IH, IW = 720, 1280
    p = blank()
    put(p, 1, 5.0, 5.0, 400.0, 400.0, cls=0, conf=0.9)      # off the top-left
    put(p, 2, 635.0, 635.0, 400.0, 400.0, cls=1, conf=0.9)  # off the bottom-right
    put(p, 3, 320.0, 320.0, 0.0, 0.0, cls=2, conf=0.9)      # zero-area
    b, s, c = dec(p, IH, IW)
    check("corners.count", b.shape[0] == 3, "got %d" % b.shape[0])
    check("corners.x_ordered", bool((b[:, 2] >= b[:, 0]).all()),
          "x2 < x1 in %r" % b.tolist())
    check("corners.y_ordered", bool((b[:, 3] >= b[:, 1]).all()),
          "y2 < y1 in %r" % b.tolist())
    check("corners.in_frame_x",
          bool((b[:, 0::2] >= 0).all() and (b[:, 0::2] <= IW - 1).all()),
          "x outside [0,%d] in %r" % (IW - 1, b.tolist()))
    check("corners.in_frame_y",
          bool((b[:, 1::2] >= 0).all() and (b[:, 1::2] <= IH - 1).all()),
          "y outside [0,%d] in %r" % (IH - 1, b.tolist()))
    check("corners.int_castable",
          all(isinstance(int(v), int) for v in b.reshape(-1)),
          "int() failed on a coordinate -- fpms_rover_agent.py does exactly "
          "this and would raise inside its inference try/except, leaving "
          "STALE boxes on screen with no visible error")


# ==========================================================================
# 4. NMS.  "NMS-free" is a training property; the CPU still does this work.
# ==========================================================================
def test_nms_is_actually_running():
    """The graph has no NonMaxSuppression op, so decode() must do it.

    GUARDS AGAINST: someone reading "YOLO26 is NMS-free" in the release notes
    and deleting the NMS call. NPU_SPEC.md is explicit: NMS-free describes
    TRAINING. The end2end=false export "still emits all 8400 anchors, so
    thresholding and a light NMS still run on the CPU."

    Symptom if removed: a person standing still produces four or five
    stacked boxes, each counted as a separate detection in telemetry. The
    video still looks broadly right. The detection COUNT does not.
    """
    IH, IW = 720, 1280
    p = blank()
    for i, a in enumerate((10, 11, 12, 13, 14, 15)):
        put(p, a, 320.0 + i * 0.5, 320.0 + i * 0.5, 120.0, 120.0,
            cls=0, conf=0.90 - i * 0.02)
    b, s, c = dec(p, IH, IW)
    check("nms.stack_collapses", b.shape[0] == 1,
          "six near-identical boxes returned %d detections. If this is 6, "
          "NMS is not running at all." % b.shape[0])
    if b.shape[0]:
        check("nms.keeps_best_score", abs(float(s[0]) - 0.90) < 1e-6,
              "NMS kept score %.4f, not the highest (0.90) -- suppression is "
              "picking the wrong survivor" % float(s[0]))


def test_nms_iou_threshold_edges():
    """Boxes at IoU just under / just over IOU_THRES behave correctly.

    GUARDS AGAINST: an off-by-one in the comparison direction. `_nms` keeps
    `iou <= iou_thres`, so a pair exactly AT the threshold is KEPT (both
    survive). Getting this backwards halves or doubles the detection count
    in crowded scenes without ever erroring.

    Construction: two axis-aligned squares of side L offset by d in x only.
    IoU = (L-d)/(L+d), so d = L*(1-iou)/(1+iou) gives an exact target IoU.
    Working in a 640x640 frame keeps the rescale the identity so the IoU we
    construct is the IoU that _nms sees.
    """
    IH = IW = 640
    L = 200.0
    t = Y.IOU_THRES

    def pair(target_iou):
        d = L * (1.0 - target_iou) / (1.0 + target_iou)
        p = blank()
        put(p, 100, 300.0, 320.0, L, L, cls=0, conf=0.95)
        put(p, 101, 300.0 + d, 320.0, L, L, cls=0, conf=0.85)
        return dec(p, IH, IW)

    b, _, _ = pair(t + 0.05)
    check("nms.above_iou_suppressed", b.shape[0] == 1,
          "IoU=%.3f (> IOU_THRES=%.2f) left %d boxes; the overlapping pair "
          "should collapse to 1" % (t + 0.05, t, b.shape[0]))

    b, _, _ = pair(t - 0.05)
    check("nms.below_iou_kept", b.shape[0] == 2,
          "IoU=%.3f (< IOU_THRES=%.2f) left %d boxes; both should survive"
          % (t - 0.05, t, b.shape[0]))

    b, _, _ = pair(t)
    check("nms.at_iou_kept", b.shape[0] == 2,
          "IoU exactly == IOU_THRES=%.2f left %d boxes. _nms uses "
          "`iou <= iou_thres` to keep, so equality must KEEP both. A change "
          "to `<` silently suppresses one more box per crowded pair."
          % (t, b.shape[0]))

    # Zero overlap must never be suppressed, whatever the threshold.
    p = blank()
    put(p, 200, 100.0, 100.0, 50.0, 50.0, cls=0, conf=0.9)
    put(p, 201, 500.0, 500.0, 50.0, 50.0, cls=0, conf=0.9)
    b, _, _ = dec(p, IH, IW)
    check("nms.disjoint_survive", b.shape[0] == 2,
          "two non-overlapping same-class boxes collapsed to %d -- two people "
          "at opposite ends of a fire line would be reported as one"
          % b.shape[0])

    # Fully contained box: IoU of a small box inside a large one is
    # area_small/area_large. Make it small enough to stay under threshold and
    # confirm containment alone does not suppress.
    p = blank()
    put(p, 300, 320.0, 320.0, 300.0, 300.0, cls=0, conf=0.9)
    put(p, 301, 320.0, 320.0, 60.0, 60.0, cls=0, conf=0.8)   # IoU = 0.04
    b, _, _ = dec(p, IH, IW)
    check("nms.contained_low_iou_kept", b.shape[0] == 2,
          "a small box inside a large one (IoU=0.04) was suppressed; greedy "
          "IoU NMS must not suppress on containment alone (got %d)"
          % b.shape[0])


def test_nms_class_offset_cannot_collide():
    """The per-class offset trick must isolate ALL 80 classes, including 79.

    GUARDS AGAINST: an arithmetic collision at the top of the class range.
    decode() shifts each box by class_id * (max(ih,iw)+1) so one NMS call
    behaves per-class. That is only sound if the shift exceeds the largest
    possible box extent -- boxes are clipped to [0, dim-1], so the spacing
    of max(ih,iw)+1 leaves a gap. If someone "optimises" the spread constant,
    class 79 boxes start overlapping class 78 boxes in the shifted space and
    two different object types suppress each other. On this rover that means
    a person standing by a truck disappears from telemetry.
    """
    IH, IW = 720, 1280
    # Same location, adjacent class ids at the top of the range.
    p = blank()
    put(p, 400, 320.0, 320.0, 200.0, 200.0, cls=78, conf=0.90)
    put(p, 401, 320.0, 320.0, 200.0, 200.0, cls=79, conf=0.85)
    b, s, c = dec(p, IH, IW)
    check("nms.classes_78_79_both_kept", b.shape[0] == 2,
          "identical boxes with class 78 and 79 collapsed to %d. The "
          "per-class offset is not separating the top of the class range."
          % b.shape[0])
    check("nms.classes_78_79_ids", set(int(v) for v in c) == {78, 79},
          "got class ids %r" % (c.tolist() if c.size else []))

    # class 0 and class 79 together -- the widest possible shift.
    p = blank()
    put(p, 500, 320.0, 320.0, 200.0, 200.0, cls=0, conf=0.90)
    put(p, 501, 320.0, 320.0, 200.0, 200.0, cls=79, conf=0.85)
    b, s, c = dec(p, IH, IW)
    check("nms.classes_0_79_both_kept", b.shape[0] == 2, "got %d" % b.shape[0])

    # And the offset must not leak into the RETURNED coordinates. The code
    # does `_nms(boxes + offset[:, None], ...)` which creates a new array; if
    # it ever became `boxes += offset` the class-79 box would be returned
    # ~101k pixels to the right, and the agent would int() and draw it
    # off-screen -- a detection that exists in telemetry but is invisible.
    if b.shape[0] == 2:
        check("nms.offset_not_leaked_into_output",
              bool((b[:, 0::2] <= IW - 1).all() and (b[:, 1::2] <= IH - 1).all()),
              "a returned box lies outside the frame (%r) -- the NMS class "
              "offset has leaked into the output coordinates" % b.tolist())


def test_class_ids_at_both_ends():
    """Class 0 and class 79 must both decode. The channel arithmetic is 4+cls.

    GUARDS AGAINST: an off-by-one in the 4:84 slice. Class 0 is `person`,
    which is the most operationally important detection this rover makes,
    and it lives in channel 4 -- the first channel after the box. A slice of
    pred[:, 5:] (the v5/v8 objectness layout) would shift every class id
    down by one: person becomes bicycle, and the fire-detector's WILDLIFE_IDS
    and COCO_TREE_IDS lookups in fpms_rover_agent.py all point at the wrong
    animals. Nothing errors. The labels are just wrong.
    """
    IH, IW = 720, 1280
    for cls in (0, 1, 39, 78, 79):
        p = put(blank(), 7, 320.0, 320.0, 80.0, 80.0, cls=cls, conf=0.90)
        b, s, c = dec(p, IH, IW)
        ok = b.shape[0] == 1 and int(c[0]) == cls
        check("classid.roundtrip_%d" % cls, ok,
              "wrote class %d into channel %d, decoded class %r"
              % (cls, 4 + cls, c.tolist() if c.size else None))
    check("classid.dtype_int32", dec(put(blank(), 0, 320.0, 320.0, 40.0, 40.0, 0, 0.9),
                                     IH, IW)[2].dtype == np.int32,
          "classes must be int32; the agent does COCO_NAMES[cid] with it")


# ==========================================================================
# 5. Numerical edges.  Where a decode stops being arithmetic and starts
#    being a crash, a hang, or a lie.
# ==========================================================================
def test_empty_result_shapes():
    """No detections must return correctly-SHAPED empties, not None or [].

    GUARDS AGAINST: a shape mismatch downstream. fpms_rover_agent.py does
    `for b, s, c in zip(boxes, scores, classes)` -- a None would raise inside
    the try/except and leave the previous frame's boxes on screen forever,
    which reads as "the rover still sees the fire" long after it does not.
    """
    b, s, c = dec(blank(), 720, 1280)
    check("empty.boxes_shape", b.shape == (0, 4), "got %r" % (b.shape,))
    check("empty.scores_shape", s.shape == (0,), "got %r" % (s.shape,))
    check("empty.classes_shape", c.shape == (0,), "got %r" % (c.shape,))
    check("empty.boxes_dtype", b.dtype == np.float32, "got %r" % b.dtype)
    check("empty.classes_dtype", c.dtype == np.int32, "got %r" % c.dtype)
    check("empty.zip_is_safe", len(list(zip(b, s, c))) == 0)


def test_single_detection_path():
    """The N==1 branch skips NMS entirely. It must still be correct.

    GUARDS AGAINST: the special case in decode():

        if boxes.shape[0] > 1:  keep = _nms(...)
        else:                   keep = np.zeros((1,), dtype=np.int32)

    That else-branch hardcodes index 0 rather than calling NMS. It is only
    correct because exactly one box exists. If the guard ever became `>= 1`
    or the branch were reordered, a zero-box case would index into an empty
    array. The empty case is short-circuited earlier, so this is currently
    safe -- pin it.
    """
    IH, IW = 720, 1280
    p = put(blank(), 4242, 400.0, 200.0, 100.0, 50.0, cls=17, conf=0.77)
    b, s, c = dec(p, IH, IW)
    check("single.count", b.shape[0] == 1, "got %d" % b.shape[0])
    if b.shape[0] == 1:
        check("single.score", abs(float(s[0]) - 0.77) < 1e-6, "got %r" % float(s[0]))
        check("single.class", int(c[0]) == 17, "got %r" % int(c[0]))
        check("single.geometry",
              abs(float(b[0][0]) - 700.0) < 1e-3 and abs(float(b[0][1]) - 196.875) < 1e-3,
              "got %r, want x1=700.0 y1=196.875" % b[0].tolist())


def test_all_anchors_above_threshold():
    """8400 detections at once must not hang, blow memory, or over-report.

    GUARDS AGAINST: a latency cliff nobody would find until the field. The
    NPU budget is ~3 inferences/second (NPU_SPEC.md), and MAX_DETECTIONS=50
    is applied AFTER the full greedy NMS, so it bounds the OUTPUT but not
    the WORK. A saturated frame -- a wall of flame, a whited-out sensor, or
    a model that has gone unstable -- pushes every anchor over threshold and
    the O(kept * N) loop runs to completion inside the camera thread.

    Two cases, because they cost very differently:
      (a) all anchors on the same object: NMS collapses in one pass, cheap.
      (b) all anchors spread out: NMS iterates once per survivor, expensive.

    We assert the OUTPUT is capped and record the WALL TIME as a warning
    rather than a failure, because timing on a Windows dev box says nothing
    about an RK3588S. MEASURE ME on real hardware.
    """
    IH, IW = 720, 1280

    # (a) degenerate: every anchor claims the same box.
    p = blank()
    p[0, :] = 320.0
    p[1, :] = 320.0
    p[2, :] = 100.0
    p[3, :] = 100.0
    p[4, :] = 0.9
    t0 = time.time()
    b, s, c = dec(p, IH, IW)
    dt_same = time.time() - t0
    check("saturate.same_object_collapses", b.shape[0] == 1,
          "8400 identical boxes gave %d detections" % b.shape[0])

    # (b) worst case: a grid of non-overlapping small boxes, all above
    # threshold, spread across the input so NMS suppresses almost nothing.
    p = blank()
    side = int(math.ceil(math.sqrt(Y.NUM_ANCHORS)))       # 92
    step = 640.0 / side
    idx = np.arange(Y.NUM_ANCHORS)
    p[0, :] = (idx % side) * step + step / 2.0
    p[1, :] = (idx // side) * step + step / 2.0
    p[2, :] = step * 0.4
    p[3, :] = step * 0.4
    p[4, :] = 0.5 + (idx % 100) * 0.001                   # distinct scores
    t0 = time.time()
    b, s, c = dec(p, IH, IW)
    dt_spread = time.time() - t0

    check("saturate.capped_at_max",
          b.shape[0] <= Y.MAX_DETECTIONS,
          "%d detections returned, MAX_DETECTIONS=%d -- the cap is not being "
          "applied and the MQTT payload would carry thousands of boxes"
          % (b.shape[0], Y.MAX_DETECTIONS))
    check("saturate.returns_something", b.shape[0] > 0,
          "a fully-saturated frame returned zero detections")
    check("saturate.shapes_consistent",
          b.shape[0] == s.shape[0] == c.shape[0],
          "boxes/scores/classes disagree: %r %r %r" % (b.shape, s.shape, c.shape))
    check("saturate.sorted_desc",
          bool(np.all(np.diff(s) <= 1e-6)),
          "scores are not in descending order: %r" % s[:8].tolist())
    check("saturate.finite", bool(np.isfinite(b).all()),
          "non-finite coordinate in the saturated result")

    warn("saturate.timing",
         "worst-case NMS on a saturated frame took %.0f ms here (degenerate "
         "case %.0f ms). MAX_DETECTIONS=%d is applied AFTER the NMS loop, so "
         "it bounds the output and not the work: the greedy loop runs once "
         "per survivor over the whole candidate set. decode() is called "
         "inline in the camera thread (fpms_rover_agent.py ~line 604), so a "
         "saturated frame stalls the frame pump for the whole of that time, "
         "which is exactly what NPU_SPEC.md design rule 2 forbids. A one-line "
         "`if len(keep) >= MAX_DETECTIONS: break` inside _nms would bound it. "
         "This timing is from an x86 dev box -- MEASURE ME on the RK3588S."
         % (dt_spread * 1000.0, dt_same * 1000.0, Y.MAX_DETECTIONS))


def test_nan_and_inf_do_not_produce_confident_boxes():
    """NaN/Inf must not become a plausible detection. Records current behaviour.

    GUARDS AGAINST: garbage from a mismatched runtime looking like a fire.
    A driver/runtime version mismatch is the documented failure mode for
    this stack (NPU_SPEC.md section 1), and a partially-initialised NPU
    buffer is a realistic source of NaN and Inf.

    Established numpy behaviour this relies on:
      * `np.argmax` over a row containing NaN returns the NaN's index
      * `NaN >= threshold` is False, so that anchor is dropped
    So a single NaN in a class row does not create a detection -- but it
    DOES silently destroy the real detection in that same anchor. That is
    worth knowing, not worth failing on.

    Inf is the dangerous one: `inf >= threshold` is True, and `np.clip` maps
    an infinite coordinate onto the frame edge, so an Inf score yields a
    full-frame box at score inf. Assert it is at least finite and in-frame
    if it is emitted, because the agent int()s it -- int(nan) raises, and
    the resulting exception is swallowed by the agent's try/except, leaving
    STALE detections on the dashboard with no error visible to the operator.
    """
    IH, IW = 720, 1280

    # inf - inf inside the centre->corner conversion legitimately raises a
    # numpy RuntimeWarning. Silence it here only: it is expected in THIS test
    # and nowhere else, and letting it print makes a clean run look dirty.
    err = np.seterr(invalid="ignore", over="ignore", divide="ignore")
    try:
        _nan_inf_body(IH, IW)
    finally:
        np.seterr(**err)


def _nan_inf_body(IH, IW):
    # NaN in the class channels.
    p = put(blank(), 60, 320.0, 320.0, 80.0, 80.0, cls=0, conf=0.9)
    p[10, 60] = np.nan
    try:
        b, s, c = dec(p, IH, IW)
        check("nan.class_no_crash", True)
        check("nan.class_not_reported", b.shape[0] == 0 or bool(np.isfinite(s).all()),
              "a NaN class score was reported as a detection: %r" % s.tolist())
        if b.shape[0] == 0:
            warn("nan.class_eats_detection",
                 "a single NaN in an anchor's class row SILENTLY DROPS the "
                 "real detection in that anchor (argmax picks the NaN, then "
                 "NaN >= thresh is False). Corrupt output therefore reduces "
                 "the detection rate with no error anywhere. Consider a "
                 "np.isfinite() guard on the tensor and a loud fault.")
    except Exception as exc:  # noqa: BLE001
        check("nan.class_no_crash", False, "raised %r" % exc)

    # NaN in the box coordinates of an otherwise-valid detection.
    p = put(blank(), 61, 320.0, 320.0, 80.0, 80.0, cls=0, conf=0.9)
    p[0, 61] = np.nan
    try:
        b, s, c = dec(p, IH, IW)
        check("nan.box_no_crash", True)
        if b.shape[0] and not bool(np.isfinite(b).all()):
            warn("nan.box_reaches_agent",
                 "a NaN box coordinate is returned to the caller. "
                 "fpms_rover_agent.py does `int(v)` on it, and int(nan) "
                 "raises ValueError inside the inference try/except -- the "
                 "agent logs 'inference failed' and keeps drawing the "
                 "PREVIOUS frame's boxes indefinitely. Consider dropping "
                 "non-finite rows in decode().")
        check("nan.box_shapes_consistent",
              b.shape[0] == s.shape[0] == c.shape[0],
              "%r %r %r" % (b.shape, s.shape, c.shape))
    except Exception as exc:  # noqa: BLE001
        check("nan.box_no_crash", False, "raised %r" % exc)

    # +/-Inf everywhere.
    p = blank()
    p[:, 70] = np.inf
    p[:, 71] = -np.inf
    try:
        b, s, c = dec(p, IH, IW)
        check("inf.no_crash", True)
        if b.shape[0]:
            check("inf.boxes_clipped_in_frame",
                  bool((b[:, 0::2] >= 0).all() and (b[:, 0::2] <= IW - 1).all()
                       and (b[:, 1::2] >= 0).all() and (b[:, 1::2] <= IH - 1).all()
                       or not np.isfinite(b).all()),
                  "an Inf-derived box escaped the frame bounds: %r" % b.tolist())
            if bool(np.isinf(s).any()):
                warn("inf.score_reported",
                     "an infinite confidence was returned as a detection. "
                     "The agent rounds it into telemetry as Infinity, which "
                     "is not valid JSON and will be dropped or corrupted by "
                     "the broker. Consider refusing non-finite tensors.")
    except Exception as exc:  # noqa: BLE001
        check("inf.no_crash", False, "raised %r" % exc)

    # A wholly non-finite tensor: whatever happens, it must not be silent
    # AND wrong. Either raise, or return finite in-frame boxes.
    p = np.full((Y.NUM_CH, Y.NUM_ANCHORS), np.nan, dtype=np.float32)
    try:
        b, s, c = dec(p, IH, IW)
        check("nan.all_nan_safe",
              b.shape[0] == 0 or bool(np.isfinite(b).all()),
              "an all-NaN tensor produced %d non-finite box(es)" % b.shape[0])
        if b.shape[0] == 0:
            warn("nan.all_nan_silent",
                 "an all-NaN output tensor decodes to zero detections and "
                 "NO error. That is indistinguishable from a quiet scene. "
                 "A corrupt NPU buffer should raise a named fault, not look "
                 "like an empty forest.")
    except Exception:  # noqa: BLE001
        check("nan.all_nan_safe", True)


def test_input_is_not_mutated():
    """decode() must not write into the caller's tensor.

    GUARDS AGAINST: an in-place optimisation corrupting a buffer the RKNN
    runtime may reuse. decode() currently does `boxes[:, 0::2] *= ...` on a
    fancy-indexed COPY (pred[mask, :4]), which is safe, and adds the NMS
    class offset out-of-place. Both are easy to "optimise" into in-place
    writes on `pred`. If rknn.inference() ever returns a view of a persistent
    output buffer, that corrupts the NEXT frame -- an intermittent,
    frame-dependent wrongness that is close to undebuggable.
    """
    p = blank()
    put(p, 80, 320.0, 320.0, 100.0, 100.0, cls=3, conf=0.9)
    put(p, 81, 100.0, 400.0, 60.0, 60.0, cls=9, conf=0.8)
    before = p.copy()
    dec(p, 720, 1280)
    check("purity.input_unmodified", bool(np.array_equal(p, before)),
          "decode() mutated its input tensor")


def test_layout_and_batch_variants_agree():
    """Every layout RKNN might hand back must decode to the SAME answer.

    GUARDS AGAINST: a runtime upgrade flipping the output layout. The
    module's own docstring says "RKNN is not consistent about whether it
    hands back the ONNX layout (1, 84, 8400) or a channel-last (1, 8400,
    84)". A silent transpose would put box coordinates where class scores
    should be: the decode would still return boxes, at nonsense positions,
    with nonsense classes. This is checked in rover/test_yolo26_decode.py for
    one detection; extended here to a multi-detection, multi-class case
    because a transpose can survive a single sparse anchor by luck.
    """
    IH, IW = 720, 1280
    p = blank()
    put(p, 11, 100.0, 120.0, 40.0, 30.0, cls=0, conf=0.91)
    put(p, 222, 500.0, 480.0, 90.0, 90.0, cls=7, conf=0.62)
    put(p, 3333, 300.0, 60.0, 20.0, 20.0, cls=79, conf=0.44)

    ref = Y.decode([p[None, ...]], IH, IW)                     # (1,84,8400)
    variants = {
        "channel_last": [np.ascontiguousarray(p.T)[None, ...]],  # (1,8400,84)
        "no_batch": [p],                                         # (84,8400)
        "no_batch_channel_last": [np.ascontiguousarray(p.T)],    # (8400,84)
        "bare_array": p[None, ...],                              # not in a list
        "extra_unit_axes": [p[None, ..., None]],                 # (1,84,8400,1)
    }
    for name, arg in variants.items():
        try:
            b, s, c = Y.decode(arg, IH, IW)
        except Exception as exc:  # noqa: BLE001
            check("layout." + name, False, "raised %r" % exc)
            continue
        same = (b.shape == ref[0].shape and np.allclose(b, ref[0], atol=1e-4)
                and np.allclose(s, ref[1], atol=1e-6)
                and bool((c == ref[2]).all()))
        check("layout." + name, same,
              "layout %s decoded differently from the ONNX layout:\n"
              "    boxes   %r vs %r\n    classes %r vs %r"
              % (name, b.tolist(), ref[0].tolist(),
                 c.tolist(), ref[2].tolist()))

    check("layout.reference_found_all_three", ref[0].shape[0] == 3,
          "the reference case itself returned %d detections, not 3"
          % ref[0].shape[0])


def test_decoder_matches_agent_preprocessing_assumption():
    """Pin the coupling between decode() and fpms_rover_agent.py's resize.

    GUARDS AGAINST: the two files drifting apart. NPU_SPEC.md: "If
    preprocessing ever becomes a letterbox, the decoder needs padding
    compensation added." Nothing enforces that today except a comment, so
    this check reads the agent source and fails if the resize call it depends
    on has changed shape.

    This is a source-text check, not a behavioural one, and it is honest
    about that: it can only see a literal `cv2.resize(frame, (640, 640))`.
    A refactor into a helper function would make it warn, not fail.
    """
    agent = os.path.join(ROVER, "fpms_rover_agent.py")
    if not os.path.exists(agent):
        warn("coupling.agent_missing",
             "fpms_rover_agent.py not found at %s; the preprocessing "
             "assumption could not be verified" % agent)
        return
    with open(agent, encoding="utf-8", errors="ignore") as fh:
        src = fh.read()

    stretch = "cv2.resize(frame, (640, 640))" in src
    check("coupling.stretch_resize_present", stretch,
          "fpms_rover_agent.py no longer contains the literal "
          "`cv2.resize(frame, (640, 640))` this decoder's per-axis rescale "
          "depends on. If preprocessing now letterboxes, EVERY box is "
          "offset and squashed and nothing will error. Re-verify the "
          "rescale in decode() before dismissing this.")
    if not stretch:
        return
    for token in ("letterbox", "copyMakeBorder", "BORDER_CONSTANT"):
        if token in src:
            warn("coupling.letterbox_token",
                 "'%s' appears in fpms_rover_agent.py. If it is on the "
                 "inference path, decode() needs padding compensation that "
                 "it does not have." % token)


# ==========================================================================
def main():
    print("=" * 72)
    print(" FPMS-OS YOLO26 decode -- offline correctness checks")
    print("=" * 72)

    if Y is None:
        print("  [FAIL] import: %s" % IMPORT_ERROR)
        print("-" * 72)
        print("  0 checks run -- the module under test could not be loaded.")
        print("  This is NOT a pass. Nothing was verified.")
        print("=" * 72)
        return 2

    print("  module: %s" % os.path.join(ROVER, "fpms_yolo26_npu.py"))
    print("  numpy:  %s%s" % (np.__version__,
                              "" if np.__version__.startswith("1.")
                              else "   [note] NPU_SPEC.md pins numpy<2 on the Pi"))
    print("  config: CONF_THRES=%.2f  IOU_THRES=%.2f  MAX_DETECTIONS=%d"
          % (Y.CONF_THRES, Y.IOU_THRES, Y.MAX_DETECTIONS))
    print("-" * 72)

    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_")]
    for name, fn in tests:
        try:
            fn()
        except Exception as exc:  # noqa: BLE001
            fail(name, "the test itself raised: %r" % exc)

    for name, msg in warnings:
        print("  [WARN] %s: %s" % (name, msg))
    for name, msg in failures:
        print("  [FAIL] %s: %s" % (name, msg))

    print("-" * 72)
    print("  %d assertions across %d test groups" % (checks_run[0], len(tests)))
    if failures:
        print("  %d FAILED, %d warnings" % (len(failures), len(warnings)))
        print()
        print("  A decode failure is not cosmetic. Read npu/tests/README.md")
        print("  before changing a test to make it pass.")
    else:
        print("  all passed, %d warnings" % len(warnings))
        print()
        print("  NOT VERIFIED HERE: that the real .rknn emits [1,84,8400], that")
        print("  the runtime loads, that the NPU produces these numbers at all.")
        print("  This proves the decode is self-consistent, nothing more.")
    print("=" * 72)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
