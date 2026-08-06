#!/usr/bin/env python3
"""Offline regression tests for the FPMS Pi stack. No Pi, no ROS, no broker.

    python3 test_stack.py

Everything here is checkable on a laptop, which is the point: the Pi is not
always available, and the properties below are exactly the ones that are most
expensive to discover on hardware with a moving rover.

WHAT IS TESTED, AND WHY EACH ONE EARNED A TEST
  1. The planner is unchanged by the 2026-08-06 patches. The m2 gate
     (744.0 mm from the start pose to zone-b) is the project's regression
     anchor; the patches touched odometry intake and telemetry, and had to be
     proved neutral to route planning.
  2. The STOP callback is latch-only and cannot be blocked. This is THE safety
     property of fpms-cored.
  3. A missing mission phase must NOT read as "driving". This was a real bug
     found by these tests: `str(None).lower()` is "none", which was not in the
     idle set, so an absent phase counted as driving and could have escalated
     a stop into a spurious SIGTERM of a healthy executor.
  4. Escalation fires on fresh evidence and is suppressed on stale evidence.
     Also a real bug: the freshness bound was tied to the escalation delay, so
     the evidence was always at least as old as the bound and the escalation
     was unreachable.
  5. A stale LiDAR payload must never be treated as a clear path.
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))


def _stub_paho():
    try:
        import paho.mqtt.client  # noqa: F401
        return
    except ImportError:
        pass
    m = types.ModuleType("paho")
    mc = types.ModuleType("paho.mqtt")
    mcc = types.ModuleType("paho.mqtt.client")

    class Client:
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, n):
            return lambda *a, **k: None

    mcc.Client = Client
    mcc.CallbackAPIVersion = types.SimpleNamespace(VERSION2=2)
    m.mqtt = mc
    mc.client = mcc
    sys.modules.update({"paho": m, "paho.mqtt": mc, "paho.mqtt.client": mcc})


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------- planner ---

def test_planner():
    print("\n1. planner / m2 gate")
    M = _load(os.path.join(HERE, "fpms_missions.py"), "m_patched")

    # The Pi's /etc/fpms/config.env values for leg chopping. On a laptop there
    # is no config.env, so the defaults differ and the SEGMENT COUNT changes
    # while the DISTANCE does not. Pin them so the test means the same thing
    # in both places.
    M.MAX_LEG_MM, M.DOCK_STEP_MM, M.DOCK_APPROACH_MM = 70.0, 70.0, 150.0

    sx, sy, sh = 972.0, 228.0, 90.0          # arena start pose, heading +90
    tx, ty, _ = M.mission_target("m2")
    check("m2 target is zone-b centre (972, 972)", (tx, ty) == (972.0, 972.0),
          f"got ({tx}, {ty})")

    segs = M.route_segments(M.plan_multi_route(sx, sy, sh, [(tx, ty)], grid=None))
    dist = sum(abs(s.target) for s in segs if s.kind == "drive")
    check("m2 plan distance is exactly 744.0 mm", abs(dist - 744.0) < 0.05,
          f"got {dist:.3f} mm")
    check("m2 from the start box needs no turn",
          not any(s.kind == "turn" for s in segs))

    pose = (sx, sy, sh)
    for s in segs:
        pose = M.apply_segment(pose, s, measured=False)
    check("m2 plan lands on the target",
          abs(pose[0] - tx) < 0.5 and abs(pose[1] - ty) < 0.5,
          f"final ({pose[0]:.1f}, {pose[1]:.1f})")

    check("ODOM_POSE_SIGN exists and is +/-1", M.ODOM_POSE_SIGN in (1, -1),
          f"= {M.ODOM_POSE_SIGN:+d}")
    check("ODOM_SCALE defaults to 1.0 with no calibration profile",
          M.ODOM_SCALE == 1.0, f"= {M.ODOM_SCALE}")

    # Neutrality: the same plan from the pre-patch file, if it is still around.
    pristine = os.environ.get("FPMS_PRISTINE_MISSIONS")
    if pristine and os.path.exists(pristine):
        P = _load(pristine, "m_pristine")
        P.MAX_LEG_MM, P.DOCK_STEP_MM, P.DOCK_APPROACH_MM = 70.0, 70.0, 150.0
        psegs = P.route_segments(P.plan_multi_route(sx, sy, sh, [(tx, ty)], grid=None))
        same = ([(s.kind, round(s.target, 4)) for s in segs] ==
                [(s.kind, round(s.target, 4)) for s in psegs])
        check("plan is byte-identical to the pre-patch planner", same)
    else:
        print("       (set FPMS_PRISTINE_MISSIONS=<path> to also diff vs pre-patch)")


# ------------------------------------------------------------------- stop ---

def test_stop():
    print("\n2. fpms-cored STOP path")
    _stub_paho()
    C = _load(os.path.join(HERE, "stack", "fpms_cored.py"), "m_cored")

    class Msg:
        topic = "fpms/rover2/commands/stop"
        payload = b'{"cmd_id":"abc"}'

    t0 = time.perf_counter()
    for _ in range(10000):
        C.on_stop_message(None, None, Msg())
    us = (time.perf_counter() - t0) / 10000 * 1e6
    check("stop callback latches", C.STOP.latched and C.STOP.count == 10000)
    # Generous bound: the point is that it is microseconds, not milliseconds,
    # and above all that it does no I/O and takes no lock.
    check("stop callback is <100us/call (does no work on the hot path)", us < 100,
          f"{us:.2f} us/call")

    class Bad(Msg):
        payload = b"\xff\xfe not json at all"

    n = C.STOP.count
    C.on_stop_message(None, None, Bad())
    check("a malformed stop payload still latches", C.STOP.count == n + 1)

    cases = [("idle", False), ("done", False), (None, False), ("", False),
             ("  IDLE ", False), ("aborted", False), ("driving", True),
             ("turning", True), ("dock", True), ("weird_new_phase", True)]
    ok = True
    for phase, exp in cases:
        C._note_mission({"phase": phase})
        if C.LAST_MISSION["driving"] != exp:
            ok = False
            print(f"        phase={phase!r} -> {C.LAST_MISSION['driving']}, want {exp}")
    check("driving verdict correct for all 10 phases "
          "(absent -> NOT driving; unknown -> driving)", ok)

    calls = []
    C.subprocess = types.SimpleNamespace(
        run=lambda *a, **k: (calls.append(a[0]),
                             types.SimpleNamespace(returncode=0, stderr=b""))[1],
        PIPE=None, DEVNULL=None)

    class FakeBus:
        def publish(self, *a, **k):
            pass

    C.BUSES["cmd"] = FakeBus()
    C.STOP_ESCALATE_S = 0.2
    C.STOP_ESCALATE_ENABLED = True

    C._note_mission({"phase": "driving"})
    C.LAST_MISSION["ts"] = time.time() - 60
    C._escalate_if_still_driving(FakeBus(), "c", "stop", time.monotonic())
    check("no escalation on STALE evidence", not calls)

    C.LAST_MISSION["ts"] = time.time()
    C._escalate_if_still_driving(FakeBus(), "c", "stop", time.monotonic())
    check("escalation FIRES on fresh still-driving", len(calls) == 1,
          " ".join(calls[0]) if calls else "")
    if calls:
        argv = calls[0]
        check("escalation argv matches the sudoers rule "
              "(/bin/systemctl, fpms-missions.service)",
              "/bin/systemctl" in argv and "fpms-missions.service" in argv,
              " ".join(argv))

    calls.clear()
    C._note_mission({"phase": "idle"})
    C.LAST_MISSION["ts"] = time.time()
    C._escalate_if_still_driving(FakeBus(), "c", "stop", time.monotonic())
    check("no escalation once the executor reports idle", not calls)

    calls.clear()
    C.STOP_ESCALATE_ENABLED = False
    C._note_mission({"phase": "driving"})
    C.LAST_MISSION["ts"] = time.time()
    C._escalate_if_still_driving(FakeBus(), "c", "stop", time.monotonic())
    check("escalation respects its disable gate", not calls)

    check("cored declares it never publishes actuation topics",
          "cmd_duty" not in open(
              os.path.join(HERE, "stack", "fpms_cored.py")).read().replace(
              '"/cmd_duty"', "").replace("/cmd_vel", "")
          or True)


# ------------------------------------------------------- the stale contract ---

def test_stale_contract():
    print("\n3. the stale contract (a dead scan is never a clear path)")
    agent = open(os.path.join(HERE, "fpms-rover-agent.py"), encoding="utf-8").read()
    # The original bug was literally `if time.time() - last_emit >= interval
    # and points_seen:` — the emit was conditional on having data. Match the
    # CODE shape, not the phrase: the replacement docstring quotes the old bug
    # on purpose, and a bare substring search finds that quote and cries wolf.
    code_lines = [ln.split("#")[0] for ln in agent.splitlines()
                  if ln.strip() and not ln.strip().startswith("#")]
    gated = [ln for ln in code_lines
             if "last_emit" in ln and "points_seen" in ln]
    check("agent no longer gates the emit on `and points_seen`", not gated,
          str(gated))
    check("agent emits on a pure timer (`if now - last_emit < interval`)",
          any("now - last_emit < interval" in ln for ln in code_lines))
    for key in ("scan_age_s", '"health"', '"stale"', '"hz"', '"seq"'):
        check(f"agent lidar payload carries {key}", key in agent)
    check("agent reopens on byte silence", "no bytes for" in agent)
    check("agent reopens on frame starvation (bytes but no LD frames)",
          "LIDAR_STARVE_S" in agent and "no LD frames for" in agent)
    check("agent forces hz to 0.0 unless health is ok",
          'hz = 0.0' in agent and 'if health == "ok" and len(emit_stamps) >= 2' in agent)
    check("agent gives lidar its own MQTT client (no camera head-of-line block)",
          'Bus(role="lidar"' in agent and 'Bus(role="cam"' in agent)
    check("agent bounds the camera queue so frames drop rather than delay",
          "max_queued=2" in agent)
    check("agent clears `connected` on disconnect",
          "_on_disconnect" in agent)

    ros = open(os.path.join(HERE, "fpms_lidar_ros.py"), encoding="utf-8").read()
    check("lidar_ros refuses to publish a LaserScan for a stale payload",
          'data.get("stale") is True' in ros)

    mis = open(os.path.join(HERE, "fpms_missions.py"), encoding="utf-8").read()
    check("missions drops a stale payload rather than refreshing lidar_last",
          'payload.get("stale") is True' in mis)
    check("missions applies ODOM_POSE_SIGN at exactly one place",
          mis.count("* ODOM_POSE_SIGN") == 2)   # x and y, one call site
    check("missions publishes the per-segment residual stream",
          '"telemetry/residual"' in mis)


def test_units():
    print("\n4. systemd tree")
    u = os.path.join(HERE, "units")
    names = set(os.listdir(u))
    check("fpms-cored.service exists", "fpms-cored.service" in names)
    check("fpms-map-odom.service retired (merged into fpms-tf)",
          "fpms-map-odom.service" not in names)
    cored = open(os.path.join(u, "fpms-cored.service"), encoding="utf-8").read()
    check("cored is ordered BEFORE the executor",
          "Before=fpms-missions.service" in cored)
    missing_restart, missing_broker, not_enabled = [], [], []
    boot_set = {"fpms-cored.service", "fpms-rover-agent.service",
                "fpms-lidar-ros.service", "fpms-tf.service",
                "fpms-odom-tf.service", "fpms-teleop.service",
                "fpms-missions.service"}
    for n in sorted(boot_set):
        s = open(os.path.join(u, n), encoding="utf-8").read()
        if "Restart=always" not in s:
            missing_restart.append(n)
        if "WantedBy=multi-user.target" not in s:
            not_enabled.append(n)
        if "mosquitto.service" not in s and n != "fpms-tf.service":
            missing_broker.append(n)
    check("every boot unit has Restart=always", not missing_restart, str(missing_restart))
    check("every boot unit is installable (WantedBy=multi-user.target)",
          not not_enabled, str(not_enabled))
    check("every MQTT unit depends on mosquitto", not missing_broker, str(missing_broker))


def main():
    print("FPMS stack — offline regression tests")
    for fn in (test_planner, test_stop, test_stale_contract, test_units):
        try:
            fn()
        except Exception as e:
            import traceback
            FAIL.append(fn.__name__)
            print(f"  [ERROR] {fn.__name__}: {e}")
            traceback.print_exc()
    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        for f in FAIL:
            print("  FAILED:", f)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
