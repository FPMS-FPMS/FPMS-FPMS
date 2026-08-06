#!/usr/bin/env python3
"""Verify the ROS-native dashboard bridge.

Two independent halves, because they fail for completely different reasons and
being told which one broke is most of the diagnosis:

  OFFLINE (default) — feeds synthetic rosbridge frames straight into
  `RosBridge._on_raw` and checks the envelopes that come out. Needs no Pi, no
  ROS and no network, so it runs in CI and on a laptop on a train. This is the
  half that catches "the mapping is wrong".

  LIVE (--live) — opens a real WebSocket to the Pi's `fpms-rosbridge.service`
  and subscribes to a topic that publishes continuously, proving the transport
  and the Pi service are both healthy. This is the half that catches "the link
  is down". It deliberately uses /odom rather than the arena pose topics:
  /odom heartbeats whether or not a mission is running, so a silent result
  means a real problem instead of an idle rover.

    python scripts/verify_ros_bridge.py
    python scripts/verify_ros_bridge.py --live --host fpms-pi.local

Exit code is 0 only if every check that ran passed.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
from pathlib import Path

# Import the backend package without needing it installed.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

FAILURES: list[str] = []
CHECKS = 0


def check(label: str, ok: bool, detail: str = "") -> None:
    global CHECKS
    CHECKS += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f" — {detail}" if detail else ""))
    if not ok:
        FAILURES.append(label)


# ===================================================================== offline
def offline_checks() -> None:
    print("\n== offline: pose mapping ==")

    # Enable before import so module-level config picks it up.
    os.environ["FPMS_ROS_ENABLED"] = "1"
    os.environ["FPMS_ROS_THING"] = "rover2"

    from backend import ros_bridge as rb

    captured: list[tuple[str, dict]] = []

    class FakeLoop:
        def is_closed(self) -> bool:
            return False

    # _emit hands the coroutine to the uvicorn loop. Intercept at that boundary
    # so the mapping is tested without an event loop or a live hub.
    def fake_run_coroutine_threadsafe(coro, _loop):
        coro.close()  # we inspect the envelope directly, so never await it
        return None

    real = rb.asyncio.run_coroutine_threadsafe
    rb.asyncio.run_coroutine_threadsafe = fake_run_coroutine_threadsafe

    class CapturingBridge(rb.RosBridge):
        def _emit(self, subtype, data):  # type: ignore[override]
            channel = f"{subtype}:{rb.ROS_THING}"
            merged = {**self._suppressed.get(channel, {}), **data}
            with self._claims_lock:
                self._claims[channel] = time.monotonic()
            captured.append((channel, merged))

    b = CapturingBridge(FakeLoop())  # type: ignore[arg-type]

    def frame(topic: str, value: float) -> str:
        return json.dumps({"op": "publish", "topic": topic, "msg": {"data": value}})

    # --- a partial pose must publish nothing ------------------------------
    b._on_raw(frame("/fpms/mission/x_mm", 972.0))
    check("x alone emits nothing (a partial pose is not a position)",
          captured == [], f"emitted {len(captured)}")

    # --- x + y completes a pose -------------------------------------------
    b._on_raw(frame("/fpms/mission/y_mm", 228.0))
    check("x+y emits one pose envelope", len(captured) == 1, f"emitted {len(captured)}")

    if captured:
        ch, data = captured[-1]
        check("channel is pose:rover2", ch == "pose:rover2", ch)
        # 972 mm -> 0.972 m. readPose multiplies by 1000 to get arena mm back.
        check("x_mm 972 -> x_m 0.972", abs(data.get("x_m", 0) - 0.972) < 1e-9,
              repr(data.get("x_m")))
        check("y_mm 228 -> y_m 0.228", abs(data.get("y_m", 0) - 0.228) < 1e-9,
              repr(data.get("y_m")))
        check("frame is tagged arena", data.get("frame") == "arena",
              repr(data.get("frame")))
        # Heading must be absent, not 0.0 — 0 is a real bearing (facing +x).
        check("heading absent until reported (never defaulted to 0)",
              "heading_deg" not in data, repr(data.get("heading_deg")))

    # --- heading arrives ---------------------------------------------------
    captured.clear()
    b._on_raw(frame("/fpms/mission/heading_deg", 90.0))
    check("heading completes and re-emits", len(captured) == 1)
    if captured:
        check("heading_deg is carried through",
              captured[-1][1].get("heading_deg") == 90.0,
              repr(captured[-1][1].get("heading_deg")))

    # --- the displaced MQTT heartbeat is merged, not lost ------------------
    captured.clear()
    b.note_suppressed("pose:rover2", {"uptime_s": 42.0, "lidar_ok": True,
                                      "x_m": 999.0})
    b._on_raw(frame("/fpms/mission/x_mm", 500.0))
    if captured:
        data = captured[-1][1]
        check("suppressed heartbeat fields survive the takeover",
              data.get("uptime_s") == 42.0 and data.get("lidar_ok") is True,
              repr({k: data.get(k) for k in ("uptime_s", "lidar_ok")}))
        check("ROS coordinate WINS over a heartbeat field of the same name",
              abs(data.get("x_m", 0) - 0.5) < 1e-9, repr(data.get("x_m")))
    else:
        check("suppressed heartbeat fields survive the takeover", False, "no emit")

    # --- claim lifecycle ---------------------------------------------------
    check("channel is claimed while live", "pose:rover2" in b.claimed_channels())

    with b._claims_lock:
        b._claims["pose:rover2"] = time.monotonic() - (rb._CLAIM_TTL_S + 1.0)
    check("claim lapses once ROS goes quiet (MQTT takes the panel back)",
          "pose:rover2" not in b.claimed_channels())

    # --- the message path must never raise ---------------------------------
    before = b.messages_seen
    for bad in ['not json at all', '{"op":"publish"}', '{"op":"publish","topic":"/x"}',
                '{"op":"publish","topic":"/fpms/mission/x_mm","msg":{"data":"NaN-ish"}}',
                '{"op":"publish","topic":"/fpms/mission/x_mm","msg":{}}']:
        try:
            b._on_raw(bad)
        except BaseException as e:  # noqa: BLE001
            check("malformed frame must not raise", False, f"{type(e).__name__}: {e}")
            break
    else:
        check("5 malformed frames survived without raising", True)
    check("malformed frames still counted as seen", b.messages_seen > before)

    # --- non-finite must not become a coordinate ---------------------------
    captured.clear()
    b._pose_parts.clear()
    b._on_raw(json.dumps({"op": "publish", "topic": "/fpms/mission/x_mm",
                          "msg": {"data": float("inf")}}))
    b._on_raw(frame("/fpms/mission/y_mm", 1.0))
    check("an infinite x is rejected, not drawn", captured == [],
          f"emitted {len(captured)}")

    rb.asyncio.run_coroutine_threadsafe = real

    # --- claimed_channels() must be safe before start ---------------------
    check("module-level claimed_channels() is empty when bridge is off",
          rb.claimed_channels() == set() or isinstance(rb.claimed_channels(), set))


# ======================================================================== live
async def _live(host: str, port: int, seconds: float) -> tuple[int, str | None]:
    try:
        from websockets.asyncio.client import connect
    except ImportError:
        from websockets import connect  # type: ignore[attr-defined]

    got = 0
    err: str | None = None
    try:
        async with connect(f"ws://{host}:{port}", ping_interval=20,
                           max_size=8 * 1024 * 1024) as ws:
            await ws.send(json.dumps({"op": "subscribe", "topic": "/odom",
                                      "type": "nav_msgs/msg/Odometry",
                                      "throttle_rate": 100, "queue_length": 1}))
            deadline = time.time() + seconds
            while time.time() < deadline:
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=max(0.1, deadline - time.time()))
                except asyncio.TimeoutError:
                    break
                f = json.loads(raw if isinstance(raw, str) else raw.decode())
                if f.get("op") == "publish" and f.get("topic") == "/odom":
                    got += 1
    except Exception as e:  # noqa: BLE001
        err = f"{type(e).__name__}: {e}"
    return got, err


def live_checks(host: str, port: int, seconds: float) -> None:
    print(f"\n== live: rosbridge transport ws://{host}:{port} ==")
    got, err = asyncio.run(_live(host, port, seconds))
    check("connected to rosbridge and received /odom", err is None and got > 0,
          err or f"{got} messages in {seconds:.0f}s")
    if err is None and got == 0:
        print("        /odom is published continuously by fpms-odom-tf.service.")
        print("        Silence means that service or the micro-ROS link is down.")


# ======================================================================== main
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true",
                    help="also open a real WebSocket to the Pi")
    ap.add_argument("--host", default=os.getenv("FPMS_ROS_HOST", "fpms-pi.local"))
    ap.add_argument("--port", type=int, default=int(os.getenv("FPMS_ROS_PORT", "9090")))
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()

    offline_checks()
    if args.live:
        live_checks(args.host, args.port, args.seconds)

    print(f"\n{CHECKS - len(FAILURES)}/{CHECKS} checks passed")
    if FAILURES:
        print("FAILED:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
