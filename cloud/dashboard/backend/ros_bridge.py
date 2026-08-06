"""ROS 2 → WebSocket bridge. The ROS-native half of the dashboard data path.

This is the first increment of moving the dashboard off MQTT and onto ROS. It
runs ALONGSIDE `mqtt_bridge.py` and does not replace it; see "Channel ownership"
below for how the two are kept from fighting over the same panel.

===========================================================================
WHY THIS TALKS ROSBRIDGE AND NOT rclpy
===========================================================================
The obvious reading of "port the dashboard onto a direct rclpy client" is
`import rclpy` in this backend. That is not possible here, and the reason is
worth writing down so it is not rediscovered every session:

  * This backend runs on the operator's WINDOWS laptop. There is no ROS 2
    installation on it — `import rclpy` fails outright.
  * Its interpreter is Python 3.14. ROS 2 Humble (what the Pi runs) targets
    Python 3.10; there is no supported Humble build for 3.14 on Windows.

Installing ROS 2 on Windows to satisfy the letter of "rclpy" would add a large,
fragile dependency to the one machine the operator uses on race day. So the
rclpy process lives where ROS already is — ON THE PI — and this module speaks to
it over `rosbridge_server`, which is ALREADY RUNNING there as
`fpms-rosbridge.service` on :9090.

The result is what was actually wanted: every value this bridge produces arrives
as a ROS message, and MQTT is nowhere in the path. It is the same architecture
`fpms_console` already uses successfully ("nothing in the live path from Pi to
laptop is MQTT") — this module just terminates that path in Python instead of in
a browser.

Cost of this choice: ZERO new dependencies. `websockets` is already installed as
part of `uvicorn[standard]`.

===========================================================================
ROS_DOMAIN_ID
===========================================================================
20 — and it is NOT set here, deliberately. The domain is a property of the ROS
graph on the Pi, and `fpms-rosbridge.service` already joins domain 20 via its
unit file. This process never joins the ROS graph at all; it holds a WebSocket
to a node that already has. Every "the topic list is empty" incident in this
project's history came from a ROS tool defaulting to domain 0 — that failure
mode cannot occur here, because there is no local ROS tool to misconfigure.

===========================================================================
WHY THE ARENA POSE COMES FROM /fpms/mission/*, NOT FROM /odom
===========================================================================
This is the single most important decision in this file and it is easy to get
backwards.

`frontend/src/lib/arena.ts` draws the rover glyph in ARENA coordinates: origin
bottom-left, millimetres, 1200x1200, with the start box at (972, 228). It reads
`x_m`/`y_m` off the pose envelope and multiplies by 1000.

`/odom` is NOT in that frame. It is the odometry frame, whose origin is wherever
the rover happened to be when the encoders were last zeroed. Feeding `/odom`
into `readPose` would place the glyph at plausible-looking coordinates that are
wrong by however far the odom origin sits from the arena origin — a fixed offset
of up to a couple of metres that is stable, silent, and looks exactly like a
correctly working map. That exact confusion has already cost this project a
session.

`fpms_missions.py` publishes `/fpms/mission/x_mm`, `/y_mm` and `/heading_deg`
ALREADY ANCHORED IN ARENA MILLIMETRES — it applies the teleop `set_coordinate`
origin itself. Those are the only values on the graph that mean what the arena
map means, so they are the only ones this bridge will draw.

Consequence, and it is the honest one: those topics are published while a
mission is running, not on an idle heartbeat. With the rover parked this bridge
therefore publishes no pose, `readPose` falls back to the start box, and the HUD
says POSE ASSUMED. That is correct. An assumed pose that says so beats a
confident pose in the wrong frame.

===========================================================================
WHAT THIS GAINS OVER THE MQTT PATH
===========================================================================
Not just a different transport — a capability MQTT never had. `arena.ts` says it
plainly: "THIS FLEET DOES NOT PUBLISH POSE." The rover agent's `telemetry/pose`
is a heartbeat (uptime_s / camera_ok / lidar_ok / frames / scans / streaming)
with no coordinate in it at all, so over MQTT the glyph is ALWAYS simulated. The
ROS graph does carry a real anchored position. This bridge is what puts it on
the map, and it does so in the exact envelope shape `readPose` already accepts,
so no frontend change is required.

===========================================================================
CHANNEL OWNERSHIP — how this avoids fighting mqtt_bridge
===========================================================================
Both bridges fan out into the same `hub` channels. If both published
`pose:<thing>`, the panel would flip between the ROS pose (real, ~5 Hz) and the
agent heartbeat (no coordinates, every ~5 s), so the glyph would blink between
MEASURED and ASSUMED once every five seconds and look broken.

So a channel has exactly one owner. `claimed_channels()` reports what this
bridge is currently and genuinely publishing, and `mqtt_bridge` consults it
before broadcasting. The claim is LIVE-GATED: it is only held while ROS
telemetry is actually arriving, so if the Pi link drops, the claim lapses within
`_CLAIM_TTL_S` and the MQTT path takes the panel back automatically. A bridge
that is connected but silent must not be allowed to hold a panel dark.
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import threading
import time
from typing import Any, Callable

from .hub import hub

log = logging.getLogger("fpms.ros")

# --------------------------------------------------------------------- config
# Deliberately NOT in config.Settings: this is an additive, feature-flagged data
# path and it must be possible to turn it off without touching a shared module
# every panel already depends on. Default OFF — enabling it changes where the
# Drive and LiDAR pages get their numbers, which is not something that should
# happen to an operator by surprise on an app update.
ROS_ENABLED = os.getenv("FPMS_ROS_ENABLED", "0") == "1"
ROS_HOST = os.getenv("FPMS_ROS_HOST", "fpms-pi.local")
ROS_PORT = int(os.getenv("FPMS_ROS_PORT", "9090"))
# Which dashboard "thing" the Pi's ROS graph corresponds to. The ROS side has no
# concept of a thing id — there is one robot per graph — so the mapping has to
# be stated here rather than parsed out of a topic name the way MQTT does it.
ROS_THING = os.getenv("FPMS_ROS_THING", "rover2")

# How long a claimed channel survives without a message before the MQTT path is
# allowed to take it back. Comfortably longer than the fastest topic here and
# shorter than a human notices a stale panel.
_CLAIM_TTL_S = 6.0

_RECONNECT_BASE_S = 2.0
_RECONNECT_MAX_S = 30.0


def _yaw_deg_from_quaternion(q: dict[str, Any]) -> float | None:
    """Yaw in degrees from a ROS quaternion dict, or None if it is not usable.

    Only the z/w terms matter for a planar robot. Returns None rather than 0.0
    for a degenerate quaternion: 0.0 is a real bearing (facing +x) and must never
    be manufactured from missing data — that is the same mistake `readPose`
    documents about the LiDAR payload's hard-coded `heading_deg: 0`.
    """
    try:
        z = float(q.get("z", 0.0))
        w = float(q.get("w", 0.0))
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(z) and math.isfinite(w)):
        return None
    if z == 0.0 and w == 0.0:
        return None
    return math.degrees(math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z))


class RosBridge:
    """A rosbridge WebSocket client that fans ROS messages into the hub.

    Public surface intentionally mirrors `mqtt_bridge.Bridge` (`connected`,
    `status()`, `messages_seen`, `last_message_at`, `seconds_since_message()`)
    so `/api/health` and anything else that reports on a data source can treat
    the two the same way.
    """

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop
        self.connected = False
        self.messages_seen = 0
        self.last_message_at: float | None = None
        self.last_error: str | None = None
        self.last_error_at: float | None = None
        self.connect_attempts = 0
        self.subscribed: list[str] = []
        self.dispatch_errors = 0
        self.last_dispatch_error: str | None = None
        self._last_dispatch_log_at = 0.0

        # channel -> monotonic time of last successful broadcast. Drives
        # claimed_channels(); see "Channel ownership" in the module docstring.
        self._claims: dict[str, float] = {}
        self._claims_lock = threading.Lock()

        # Latest arena-frame pose components. They arrive on three separate
        # Float32 topics, so a pose envelope can only be emitted once all three
        # have been seen — a partial pose would draw the glyph at a coordinate
        # the rover is not at.
        self._pose_parts: dict[str, float] = {}
        self._pose_parts_at: float = 0.0

        # Last MQTT payload per channel that this bridge caused to be
        # suppressed. Claiming `pose:` would otherwise THROW AWAY the agent
        # heartbeat, which is the only source of uptime_s / camera_ok /
        # lidar_ok / lidar_health / lidar_hz — none of which exist on the ROS
        # graph. Taking a panel over must not silently cost it fields it had
        # before, so the heartbeat is kept here and merged back underneath the
        # ROS values on every emit.
        self._suppressed: dict[str, dict[str, Any]] = {}

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._ws_loop: asyncio.AbstractEventLoop | None = None

        # ---- topic map ----------------------------------------------------
        # (ros_topic, ros_type, handler). Handlers return None to publish
        # nothing; anything they do publish goes out through _emit().
        #
        # Only the pose group is enabled in this increment. The remaining
        # channels are listed with the reason they are NOT yet wired, so the
        # next increment is a decision about each one rather than a rediscovery:
        #
        #   lidar:<thing>   /scan_lidar is sensor_msgs/LaserScan, but the panel
        #                   consumes the agent's MQTT shape (ranges_m + health +
        #                   hz + scan_age_s + seq). Mapping LaserScan onto that
        #                   means re-deriving the health/staleness fields that
        #                   the agent computes from scanner timing the ROS
        #                   message does not carry. Doing it badly would replace
        #                   a feed that is currently honest about going stale
        #                   with one that is not, so it needs its own increment.
        #   drive:<thing>   /fpms_health is an Int32MultiArray whose meaning is
        #                   a 12-field table in fpms_main.cpp. Worth doing, but
        #                   the panel also reads teleop-only fields (measured
        #                   topic rates, micro-ROS link state) that are not on
        #                   the ROS graph at all.
        #   events          /fpms/events is a String of JSON and maps almost
        #                   directly; held back only so this increment lands
        #                   with one behaviour change, not two.
        #   camera:/thermal: NOT AVAILABLE ON ROS AT ALL. The agent publishes
        #                   frames straight to MQTT and there is no ROS
        #                   publisher for either. These two tabs cannot be
        #                   ported without new rover-side publishers, and that
        #                   is a rover change, not a dashboard one.
        self._topics: list[tuple[str, str, Callable[[dict[str, Any]], None]]] = [
            ("/fpms/mission/x_mm", "std_msgs/msg/Float32",
             lambda m: self._on_pose_part("x_mm", m)),
            ("/fpms/mission/y_mm", "std_msgs/msg/Float32",
             lambda m: self._on_pose_part("y_mm", m)),
            ("/fpms/mission/heading_deg", "std_msgs/msg/Float32",
             lambda m: self._on_pose_part("heading_deg", m)),
        ]

    # ---- lifecycle -------------------------------------------------------
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="fpms-ros-bridge")
        self._thread.start()
        log.info("ROS bridge starting against ws://%s:%s (thing=%s)",
                 ROS_HOST, ROS_PORT, ROS_THING)

    def stop(self) -> None:
        self._stop.set()
        loop = self._ws_loop
        if loop is not None and not loop.is_closed():
            # Wake the client out of its recv() so shutdown is prompt.
            loop.call_soon_threadsafe(lambda: None)

    def _run(self) -> None:
        """Own asyncio loop on our own thread, so the websockets client never
        shares a loop with uvicorn's. Mirrors how paho keeps its own thread."""
        loop = asyncio.new_event_loop()
        self._ws_loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._client_forever())
        except Exception:  # noqa: BLE001
            log.exception("ROS bridge thread exited unexpectedly")
        finally:
            try:
                loop.close()
            except Exception:  # noqa: BLE001
                pass

    async def _client_forever(self) -> None:
        backoff = _RECONNECT_BASE_S
        while not self._stop.is_set():
            try:
                await self._session()
                backoff = _RECONNECT_BASE_S
            except Exception as e:  # noqa: BLE001
                self.connected = False
                self.last_error = f"{type(e).__name__}: {e}"
                self.last_error_at = time.time()
                log.warning("ROS bridge link to ws://%s:%s failed (%s); "
                            "retrying in %.0fs", ROS_HOST, ROS_PORT, e, backoff)
            if self._stop.is_set():
                return
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, _RECONNECT_MAX_S)

    async def _session(self) -> None:
        # websockets >= 14 moved the asyncio client here; keep the legacy import
        # as a fallback so this does not become the reason the app will not boot
        # on a machine with an older wheel.
        try:
            from websockets.asyncio.client import connect
        except ImportError:  # pragma: no cover - depends on installed version
            from websockets import connect  # type: ignore[attr-defined]

        url = f"ws://{ROS_HOST}:{ROS_PORT}"
        self.connect_attempts += 1
        async with connect(url, ping_interval=20, ping_timeout=20,
                           max_size=8 * 1024 * 1024) as ws:
            self.connected = True
            self.last_error = None
            self.subscribed = []
            for topic, ros_type, _ in self._topics:
                await ws.send(json.dumps({
                    "op": "subscribe", "topic": topic, "type": ros_type,
                    # Keep the socket honest under load: we would rather drop
                    # intermediate frames than build a backlog that makes the
                    # glyph lag the rover.
                    "throttle_rate": 0, "queue_length": 1,
                }))
                self.subscribed.append(topic)
            log.info("ROS bridge connected to %s; subscribed to %d topic(s): %s",
                     url, len(self.subscribed), ", ".join(self.subscribed))

            while not self._stop.is_set():
                raw = await ws.recv()
                self._on_raw(raw)

        self.connected = False

    # ---- message path ----------------------------------------------------
    def _on_raw(self, raw: Any) -> None:
        """Route one rosbridge frame. MUST NOT raise.

        Same contract, and same hard-won reason, as `mqtt_bridge._on_message`:
        an exception escaping the read loop kills the link, and a dead link that
        never reports itself leaves every panel frozen on its last value while
        the app still says it is connected. No single bad frame may cost the
        feed.
        """
        try:
            self.messages_seen += 1
            self.last_message_at = time.time()
            try:
                frame = json.loads(raw if isinstance(raw, str) else raw.decode())
            except Exception:  # noqa: BLE001
                log.exception("bad rosbridge frame")
                return
            if frame.get("op") != "publish":
                return
            topic = frame.get("topic")
            msg = frame.get("msg")
            if not isinstance(msg, dict):
                return
            for t, _type, handler in self._topics:
                if t == topic:
                    handler(msg)
                    return
        except BaseException as e:  # noqa: BLE001 - catching everything is the point
            self.dispatch_errors += 1
            self.last_dispatch_error = f"{type(e).__name__}: {e}"
            now = time.time()
            if self.dispatch_errors == 1 or now - self._last_dispatch_log_at > 30:
                self._last_dispatch_log_at = now
                log.exception("ROS dispatch failed (%d so far); frame dropped, "
                              "link kept alive", self.dispatch_errors)

    def _on_pose_part(self, key: str, msg: dict[str, Any]) -> None:
        """Collect one third of the arena pose and emit once all three exist."""
        try:
            value = float(msg.get("data"))
        except (TypeError, ValueError):
            return
        if not math.isfinite(value):
            return

        now = time.time()
        # The three topics are published together by fpms_missions. If they ever
        # drift apart far enough to matter, the older parts are stale and mixing
        # them would place the glyph at a coordinate that never existed — so the
        # set is dropped rather than blended.
        if now - self._pose_parts_at > _CLAIM_TTL_S:
            self._pose_parts.clear()
        self._pose_parts[key] = value
        self._pose_parts_at = now

        if not {"x_mm", "y_mm"} <= self._pose_parts.keys():
            return

        data: dict[str, Any] = {
            # readPose reads metres and multiplies by 1000. These are ARENA
            # metres — see the module docstring on why that distinction is the
            # whole point of this file.
            "x_m": self._pose_parts["x_mm"] / 1000.0,
            "y_m": self._pose_parts["y_mm"] / 1000.0,
            # Provenance, so a panel (or a person reading a capture) can tell a
            # ROS-anchored pose from the agent heartbeat that carries none.
            "source": "ros:/fpms/mission",
            "frame": "arena",
        }
        heading = self._pose_parts.get("heading_deg")
        if heading is not None:
            data["heading_deg"] = heading
        self._emit("pose", data)

    def note_suppressed(self, channel: str, payload: dict[str, Any]) -> None:
        """Record an MQTT payload this bridge displaced, so it can be merged.

        Called by `mqtt_bridge` immediately before it skips a claimed channel.
        Never raises: it sits in the MQTT dispatch path, which must not be able
        to die because of anything in this module.
        """
        try:
            if isinstance(payload, dict):
                self._suppressed[channel] = payload
        except Exception:  # noqa: BLE001
            pass

    def _emit(self, subtype: str, data: dict[str, Any]) -> None:
        """Broadcast one envelope, in exactly mqtt_bridge's shape."""
        channel = f"{subtype}:{ROS_THING}"
        # ROS values win; the displaced MQTT heartbeat fills in everything it
        # was carrying that ROS has no equivalent for. Ordering matters — a
        # heartbeat must never overwrite a real coordinate.
        merged = {**self._suppressed.get(channel, {}), **data}
        envelope = {"thing": ROS_THING, "subtype": subtype,
                    "ts": self.last_message_at or time.time(), "data": merged}
        with self._claims_lock:
            self._claims[channel] = time.monotonic()
        if self.loop.is_closed():
            return
        asyncio.run_coroutine_threadsafe(hub.broadcast(channel, envelope), self.loop)

    # ---- ownership + status ---------------------------------------------
    def claimed_channels(self) -> set[str]:
        """Channels this bridge is CURRENTLY publishing, live-gated.

        A claim lapses `_CLAIM_TTL_S` after the last message, so a ROS link that
        goes quiet hands the panel back to MQTT instead of holding it dark.
        """
        now = time.monotonic()
        with self._claims_lock:
            return {ch for ch, at in self._claims.items() if now - at <= _CLAIM_TTL_S}

    def seconds_since_message(self) -> float | None:
        if self.last_message_at is None:
            return None
        return max(0.0, time.time() - self.last_message_at)

    def status(self) -> dict[str, Any]:
        age = self.seconds_since_message()
        st: dict[str, Any] = {
            "enabled": True,
            "connected": self.connected,
            "url": f"ws://{ROS_HOST}:{ROS_PORT}",
            "transport": "rosbridge",
            "thing": ROS_THING,
            "messages_seen": self.messages_seen,
            "last_message_at": self.last_message_at,
            "seconds_since_message": age,
            "subscribed": list(self.subscribed),
            "claimed_channels": sorted(self.claimed_channels()),
            "connect_attempts": self.connect_attempts,
            "dispatch_errors": self.dispatch_errors,
            "last_dispatch_error": self.last_dispatch_error,
            "last_error": self.last_error,
            "last_error_at": self.last_error_at,
        }
        if not self.connected:
            st["problem"] = (
                f"Not connected to rosbridge at ws://{ROS_HOST}:{ROS_PORT}"
                + (f" — {self.last_error}" if self.last_error else
                   " — is fpms-rosbridge.service running on the Pi?")
            )
        elif self.messages_seen == 0:
            st["problem"] = (
                "Connected to rosbridge but no message has arrived yet. The "
                "arena pose topics (/fpms/mission/x_mm etc.) are published by "
                "fpms_missions.py while a mission runs, not on an idle "
                "heartbeat — so silence with a parked rover is expected."
            )
        else:
            st["problem"] = None
        return st


_bridge: RosBridge | None = None


def start_bridge(loop: asyncio.AbstractEventLoop) -> RosBridge | None:
    """Start the ROS bridge if it is enabled. Returns None when it is not."""
    global _bridge
    if not ROS_ENABLED:
        log.info("ROS bridge disabled (set FPMS_ROS_ENABLED=1 to turn it on)")
        return None
    if _bridge is None:
        _bridge = RosBridge(loop)
        _bridge.start()
    return _bridge


def get_bridge() -> RosBridge | None:
    return _bridge


def claimed_channels() -> set[str]:
    """Module-level helper so mqtt_bridge can ask without importing the class.

    Returns an empty set when the ROS bridge is off or has never published, which
    is what makes the MQTT path the default rather than a fallback.
    """
    b = _bridge
    if b is None:
        return set()
    try:
        return b.claimed_channels()
    except Exception:  # noqa: BLE001
        return set()
