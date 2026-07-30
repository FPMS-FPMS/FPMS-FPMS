import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fpms_yolo26_npu as y

FAIL = []
def check(name, cond, extra=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"   {extra}" if extra else ""))
    if not cond:
        FAIL.append(name)

def blank():
    """(84, 8400) all-zero prediction in the ONNX layout."""
    return np.zeros((y.NUM_CH, y.NUM_ANCHORS), dtype=np.float32)

def put(pred, anchor, cx, cy, w, h, cls, conf):
    pred[0, anchor], pred[1, anchor] = cx, cy
    pred[2, anchor], pred[3, anchor] = w, h
    pred[4 + cls, anchor] = conf

IH, IW = 480, 640     # the rover's real frame size

print("1. box round-trips through the 640-stretch rescale")
# centre of a 640x640 input == centre of the source frame, whatever its aspect
p = blank()
put(p, 100, 320.0, 320.0, 64.0, 64.0, cls=0, conf=0.9)
b, s, c = y.decode([p[None, ...]], IH, IW)
check("one detection returned", b.shape == (1, 4), f"got {b.shape}")
cx = (b[0][0] + b[0][2]) / 2
cy = (b[0][1] + b[0][3]) / 2
check("centre maps to frame centre", abs(cx - IW/2) < 1e-3 and abs(cy - IH/2) < 1e-3,
      f"got ({cx:.2f}, {cy:.2f}) want ({IW/2}, {IH/2})")
# width 64/640 of input -> 64/640 * 640 = 64 px ; height 64/640 * 480 = 48 px
w = b[0][2] - b[0][0]
h = b[0][3] - b[0][1]
check("w scales by iw/640", abs(w - 64.0) < 1e-3, f"got {w:.3f} want 64.0")
check("h scales by ih/640", abs(h - 48.0) < 1e-3, f"got {h:.3f} want 48.0")
check("class/score preserved", c[0] == 0 and abs(s[0] - 0.9) < 1e-6, f"cls={c[0]} conf={s[0]}")

print("\n2. both tensor orientations decode identically")
p = blank()
put(p, 7, 100.0, 200.0, 40.0, 40.0, cls=3, conf=0.8)
b1, s1, c1 = y.decode([p[None, ...]], IH, IW)            # (1, 84, 8400)
b2, s2, c2 = y.decode([p.T[None, ...].copy()], IH, IW)   # (1, 8400, 84)
check("(1,84,8400) == (1,8400,84)", np.allclose(b1, b2) and np.allclose(s1, s2) and (c1 == c2).all())
b3, s3, c3 = y.decode([p], IH, IW)                       # no batch axis
check("batchless (84,8400) also works", np.allclose(b1, b3))

print("\n3. confidence threshold")
p = blank()
put(p, 10, 320.0, 320.0, 50.0, 50.0, cls=0, conf=y.CONF_THRES - 0.01)
b, s, c = y.decode([p[None, ...]], IH, IW)
check("below-threshold dropped", b.shape[0] == 0, f"got {b.shape[0]}")
p = blank()
put(p, 10, 320.0, 320.0, 50.0, 50.0, cls=0, conf=y.CONF_THRES + 0.01)
b, s, c = y.decode([p[None, ...]], IH, IW)
check("above-threshold kept", b.shape[0] == 1, f"got {b.shape[0]}")

print("\n4. NMS suppresses duplicates of the same object")
p = blank()
for i, a in enumerate((10, 11, 12, 13)):
    put(p, a, 320.0 + i, 320.0 + i, 80.0, 80.0, cls=0, conf=0.9 - i * 0.05)
b, s, c = y.decode([p[None, ...]], IH, IW)
check("4 near-identical boxes -> 1", b.shape[0] == 1, f"got {b.shape[0]}")
check("highest score survived", abs(s[0] - 0.9) < 1e-6, f"got {s[0]}")

print("\n5. NMS is per-class (overlapping person + truck both survive)")
p = blank()
put(p, 20, 320.0, 320.0, 80.0, 80.0, cls=0, conf=0.9)   # person
put(p, 21, 322.0, 322.0, 80.0, 80.0, cls=7, conf=0.85)  # truck, same place
b, s, c = y.decode([p[None, ...]], IH, IW)
check("both classes kept", b.shape[0] == 2, f"got {b.shape[0]}")
check("class ids correct", set(c.tolist()) == {0, 7}, f"got {c.tolist()}")

print("\n6. distant same-class objects are not merged")
p = blank()
put(p, 30, 100.0, 100.0, 40.0, 40.0, cls=0, conf=0.9)
put(p, 31, 500.0, 500.0, 40.0, 40.0, cls=0, conf=0.9)
b, s, c = y.decode([p[None, ...]], IH, IW)
check("2 separate boxes kept", b.shape[0] == 2, f"got {b.shape[0]}")

print("\n7. empty / degenerate input")
b, s, c = y.decode([blank()[None, ...]], IH, IW)
check("all-zero -> no detections", b.shape == (0, 4) and s.shape == (0,) and c.shape == (0,))

print("\n8. boxes are clipped into the frame")
p = blank()
put(p, 40, 5.0, 5.0, 200.0, 200.0, cls=0, conf=0.9)     # hangs off top-left
b, s, c = y.decode([p[None, ...]], IH, IW)
check("no negative coords", (b >= 0).all(), f"got {b.tolist()}")
check("within frame bounds", (b[:, 0::2] <= IW - 1).all() and (b[:, 1::2] <= IH - 1).all())

print("\n9. a wrong-shaped output raises, not silently misdecodes")
try:
    y.decode([np.zeros((1, 25200, 85), np.float32)], IH, IW)
    check("bad shape rejected", False, "no exception raised")
except ValueError as e:
    check("bad shape rejected", True, f"ValueError: {str(e)[:50]}")

print("\n10. dtype / ordering contract")
p = blank()
put(p, 50, 320.0, 320.0, 60.0, 60.0, cls=5, conf=0.7)
put(p, 51, 100.0, 100.0, 60.0, 60.0, cls=5, conf=0.95)
b, s, c = y.decode([p[None, ...]], IH, IW)
check("boxes float32", b.dtype == np.float32, str(b.dtype))
check("classes int32", c.dtype == np.int32, str(c.dtype))
check("sorted by score desc", s[0] >= s[-1], f"{s.tolist()}")
check("int() on coords works (agent does this)", all(isinstance(int(v), int) for v in b[0]))

print("\n" + ("ALL PASSED" if not FAIL else f"{len(FAIL)} FAILED: {FAIL}"))
sys.exit(1 if FAIL else 0)
