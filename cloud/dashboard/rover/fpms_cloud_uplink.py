#!/usr/bin/env python3
"""FPMS rover cloud uplink — a second, independent telemetry path to the Worker.

WHY THIS EXISTS
The rover publishes to the LOCAL MQTT broker, and the laptop backend
(backend/cloud_forwarder.py) relays a throttled copy up to the Cloudflare
Worker. That makes the laptop a mandatory relay: close the lid and the cloud
dashboard goes stale even though the rover is still driving, still seeing, and
still perfectly able to reach the internet. This module removes the laptop from
that path — the rover talks to the Worker directly, so the cloud view survives
the relay being off. It runs *alongside* MQTT rather than replacing it; the
local dashboard stays the low-latency view and this is the one that keeps
working when nobody is home.

WHY NEWEST-WINS FOR TELEMETRY BUT FIFO FOR EVENTS
Telemetry is a *sample of the present*. If the uplink stalls for three seconds
on a bad cellular handover, an operator who opens the dashboard wants the frame
from now — not a three-second backlog of stale frames that must drain before
the live one appears. So telemetry uses a single slot per subtype (a dict): a
newer frame overwrites the pending one and the stale bytes are never sent. That
bounds memory by the number of subtypes rather than by outage length, which is
the property that matters on a box with no swap.

Events are the opposite: they are an *audit trail*. "fire", "fire_cleared",
"obstacle" only make sense in order, and silently overwriting one loses the
alert entirely. So events go through a strict bounded FIFO and, if the network
is down, spill to a disk spool so an alert raised in a dead zone still arrives
once the link returns. Camera frames are never spooled — too big, and a
ten-minute-old frame is worthless anyway.

WHY THE DEFAULT MUST BE OFF
This path costs money and radio time that the operator has not agreed to spend.
It sends over a metered cellular link, it counts against the Workers free
tier's 100k requests/day, and it pushes imagery off the local network to a
third party. None of that is a decision this file gets to make on an operator's
behalf, and a rover that silently starts uploading after a software update is a
bill and a privacy incident waiting to happen. So UPLINK_ENABLED is false
unless FPMS_CLOUD_UPLINK is explicitly turned on in /etc/fpms/config.env, and
from_config() returns None until then. Opting in is a deliberate act.

Standard library only, with one OPTIONAL extra: websocket-client. If that
import fails the uplink still works over HTTPS, so nothing has to be installed
on a Pi on a slow link.

Config lives in /etc/fpms/config.env, same as the agent.
"""
from __future__ import annotations

import argparse
import base64
import http.client
import json
import os
import random
import socket
import sys
import threading
import time
from collections import deque
from urllib.parse import urlsplit, quote as _quote

# websocket-client is optional on purpose — see the module docstring. Imported
# at module level so the failure is discovered once, at startup, rather than on
# every reconnect attempt.
try:
    import websocket  # type: ignore
    _WS_AVAILABLE = True
    _WS_IMPORT_ERROR = None
except Exception as e:  # noqa: BLE001
    websocket = None  # type: ignore
    _WS_AVAILABLE = False
    _WS_IMPORT_ERROR = str(e)


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# ---------------------------------------------------------------- config ---

def load_config(path="/etc/fpms/config.env"):
    cfg = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    cfg[k] = v
    except (FileNotFoundError, OSError):
        pass
    return cfg


CFG = load_config()
THING = CFG.get("FPMS_THING_NAME", "rover2")

# Off unless an operator opts in. See the docstring — this is the safety gate,
# not a convenience default.
UPLINK_ENABLED = CFG.get("FPMS_CLOUD_UPLINK", "0").strip().lower() in ("1", "true", "yes", "on")

INGEST_URL = CFG.get("FPMS_INGEST_URL", "")
INGEST_WS_URL = CFG.get("FPMS_INGEST_WS_URL", "")
INGEST_TOKEN = CFG.get("FPMS_INGEST_TOKEN", "")
UPLINK_MODE = CFG.get("FPMS_CLOUD_UPLINK_MODE", "auto").strip().lower()
UPLINK_FPS = float(CFG.get("FPMS_CLOUD_UPLINK_FPS", "5") or 5)
SPOOL_PATH = CFG.get("FPMS_CLOUD_SPOOL", "/var/lib/fpms/uplink-spool.jsonl")

# --- server-imposed limits. These are the Worker's, not ours: exceeding any of
# them gets the WHOLE batch rejected, so they are enforced here rather than
# discovered as a 413.
MAX_BODY_BYTES = 262144         # worker: MAX_INGEST_BYTES = 256 * 1024
MAX_ITEMS = 100                 # worker: "batch too large (max 100)"
# A camera item is ~28 KB of base64, so 8 is the most that fits under 256 KB
# with headroom for the pose/lidar/event items riding along.
MAX_CAMERA_ITEMS = 8
# Leave a margin under the hard cap: the Worker re-checks the decoded length
# after reading, and being a few hundred bytes over loses everything.
BODY_TARGET_BYTES = 240 * 1024

# The Worker validates subtype against /^[a-z0-9_-]+$/i and rejects the entire
# batch on the first bad one. One malformed subtype would therefore poison every
# batch it rides in, so offer() sanitises rather than trusting its caller.
_SUBTYPE_OK = frozenset(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")

TICK_S = 0.1                    # one array per tick
EVENT_QUEUE_MAX = 200           # bounded FIFO; an outage must not grow memory
SPOOL_MAX_BYTES = 2 * 1024 * 1024
BACKOFF_MIN_S = 1.0
BACKOFF_MAX_S = 30.0
CONNECT_TIMEOUT_S = 10.0
SEND_TIMEOUT_S = 15.0

# When the server says nobody is watching (camera_fps 0), still send a frame
# occasionally: the dashboard should show something within seconds of someone
# opening it, not wait for the next rate command to arrive.
TRICKLE_INTERVAL_S = 10.0

# Cloudflare answers the default Python User-Agent with a 403 — the laptop
# forwarder learned this the hard way. Keep an explicit UA on every request.
USER_AGENT = "fpms-rover-uplink/1.0"

# Subtypes treated as "camera": big, rate-capped, never spooled.
_CAMERA_PREFIXES = ("camera",)


def _clean_subtype(subtype, kind):
    """Return (kind, subtype) both safe for the Worker's validators.

    The agent's call sites use combined suffixes like "telemetry/camera"
    (that is what Bus.publish takes), so accept that shape and split it rather
    than making the integrator remember which half goes where.
    """
    s = str(subtype or "").strip().strip("/")
    if "/" in s:
        head, tail = s.split("/", 1)
        if head in ("telemetry", "events"):
            kind = head
            s = tail
        s = s.replace("/", "-")
    s = "".join(c if c in _SUBTYPE_OK else "-" for c in s)
    if not s:
        s = "unknown"
    return ("events" if kind == "events" else "telemetry"), s[:64]


def _is_camera(subtype):
    return subtype.lower().startswith(_CAMERA_PREFIXES)


def _derive_ws_url(url):
    """https://host/ingest -> wss://host/ingest/ws.

    Same origin by construction, so an operator only has to configure one URL
    and the two paths cannot drift apart in config.
    """
    try:
        p = urlsplit(url)
        if not p.scheme or not p.netloc:
            return ""
        scheme = "wss" if p.scheme == "https" else "ws"
        path = (p.path or "/ingest").rstrip("/")
        if not path.endswith("/ws"):
            path = path + "/ws"
        return f"{scheme}://{p.netloc}{path}"
    except Exception:  # noqa: BLE001
        return ""


# ------------------------------------------------------------ transports ---

class _HttpTransport:
    """HTTPS POST over a REUSED connection.

    A fresh TLS handshake per tick would dominate both latency and cellular
    data — the handshake is larger than a small batch. http.client keeps the
    socket open between requests as long as the response body is fully read,
    which is why _send() always drains it.
    """

    name = "http"

    def __init__(self, url, token):
        p = urlsplit(url)
        if not p.netloc:
            raise ValueError(f"bad ingest URL: {url!r}")
        self._secure = (p.scheme != "http")
        self._host = p.hostname or ""
        self._port = p.port or (443 if self._secure else 80)
        self._path = p.path or "/ingest"
        if p.query:
            self._path += "?" + p.query
        self._token = token
        self._conn = None

    def connect(self):
        self.close()
        if self._secure:
            self._conn = http.client.HTTPSConnection(
                self._host, self._port, timeout=CONNECT_TIMEOUT_S)
        else:
            self._conn = http.client.HTTPConnection(
                self._host, self._port, timeout=CONNECT_TIMEOUT_S)
        # Connect eagerly so a dead endpoint is discovered here, where the
        # backoff lives, instead of on the first real batch.
        self._conn.connect()
        self._conn.sock.settimeout(SEND_TIMEOUT_S)

    def send(self, body):
        """POST one JSON array. Returns (ok, detail). Raises on transport loss."""
        if self._conn is None:
            raise OSError("not connected")
        self._conn.request("POST", self._path, body=body, headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self._token}",
            "User-Agent": USER_AGENT,
            "Content-Length": str(len(body)),
        })
        resp = self._conn.getresponse()
        # Must read the body in full or the connection cannot be reused.
        detail = resp.read(512).decode("utf-8", "replace")
        if 200 <= resp.status < 300:
            return True, None
        # 401/403 are configuration faults, not transport faults. Report them
        # so the caller can back off hard rather than hammering with a bad token.
        return False, f"HTTP {resp.status}: {detail[:200]}".strip()

    def poll(self):
        return []       # HTTPS gives us no channel for rate commands

    def close(self):
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001
                pass
            self._conn = None


class _WsTransport:
    """WebSocket to /ingest/ws. Preferred: one connection, no per-tick handshake.

    The Bearer token goes in a connection HEADER, never a query string — query
    strings land in access logs and proxy logs, and this token is the only thing
    standing between the internet and the ingest endpoint.
    """

    name = "ws"

    def __init__(self, ws_url, token, thing=None):
        if not _WS_AVAILABLE:
            raise RuntimeError(f"websocket-client unavailable: {_WS_IMPORT_ERROR}")
        if not ws_url:
            raise ValueError("no websocket URL")
        # The server requires ?thing= on the upgrade itself, and pins the socket
        # to that rover: a publisher may only speak for the thing it connected
        # as. Without this the handshake is rejected with 400 before a single
        # frame is sent, so it has to be on the URL, not in the payload.
        if thing and "thing=" not in ws_url:
            sep = "&" if "?" in ws_url else "?"
            ws_url = f"{ws_url}{sep}thing={_quote(str(thing))}"
        self._url = ws_url
        self._token = token
        self._ws = None

    def connect(self):
        self.close()
        self._ws = websocket.create_connection(
            self._url,
            timeout=CONNECT_TIMEOUT_S,
            header=[f"Authorization: Bearer {self._token}",
                    f"User-Agent: {USER_AGENT}"],
            enable_multithread=True,
        )
        # Non-blocking from here on: the send loop polls for rate commands and
        # must never park a tick waiting for a message that may never come.
        self._ws.settimeout(SEND_TIMEOUT_S)

    def send(self, body):
        if self._ws is None:
            raise OSError("not connected")
        # The server sends no per-message ack by design, so a successful write
        # is all the confirmation there is.
        self._ws.send(body)
        return True, None

    def poll(self):
        """Drain any pending server messages without blocking."""
        out = []
        if self._ws is None:
            return out
        for _ in range(8):
            try:
                self._ws.settimeout(0.0)
                msg = self._ws.recv()
            except Exception:  # noqa: BLE001
                # Timeout with nothing pending is the normal case and is not
                # distinguishable cheaply across websocket-client versions, so
                # any read miss just ends the drain.
                break
            finally:
                try:
                    self._ws.settimeout(SEND_TIMEOUT_S)
                except Exception:  # noqa: BLE001
                    pass
            if not msg:
                break
            if isinstance(msg, bytes):
                msg = msg.decode("utf-8", "replace")
            out.append(msg)
        return out

    def pong(self):
        try:
            if self._ws is not None:
                self._ws.send("pong")
        except Exception:  # noqa: BLE001
            pass

    def close(self):
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:  # noqa: BLE001
                pass
            self._ws = None


# ---------------------------------------------------------------- uplink ---

class CloudUplink:
    """Rover -> Worker uplink. Every public method swallows its own errors.

    offer() is called from the camera hot loop, so it is the one method whose
    contract really matters: non-blocking, and never raising. An exception
    escaping into camera_loop() would take the camera down, and a camera that
    dies to make an *optional* uplink work is a strictly worse rover.
    """

    def __init__(self, thing, url, ws_url, token, mode="auto", fps=5.0,
                 spool_path=None):
        self.thing = thing or "rover"
        self.url = (url or "").strip()
        self.ws_url = (ws_url or "").strip() or _derive_ws_url(self.url)
        self.token = (token or "").strip()
        self.mode = (mode or "auto").strip().lower()
        try:
            self.fps = max(0.0, float(fps))
        except (TypeError, ValueError):
            self.fps = 5.0

        self._lock = threading.Lock()
        # Newest-wins single slot per subtype. See the module docstring.
        self._latest = {}
        # Strict bounded FIFO. maxlen makes the drop automatic and O(1); we
        # count it so a silent loss still shows up in status().
        self._events = deque(maxlen=EVENT_QUEUE_MAX)

        self._stop = threading.Event()
        self._thread = None
        self._transport = None
        self._server_fps = None         # None = server has not told us yet
        self._last_camera_send = 0.0
        self._backoff = BACKOFF_MIN_S

        # Counters. Plain ints guarded by the same lock as the queues.
        self.offered = 0
        self.sent = 0
        self.batches = 0
        self.errors = 0
        self.dropped_events = 0
        self.dropped_telemetry = 0
        self.rate_commands = []
        self.last_error = None
        self.connected_as = None

        self.spool_path = self._prepare_spool(
            SPOOL_PATH if spool_path is None else spool_path)

    # ---- spool -----------------------------------------------------------

    def _prepare_spool(self, path):
        """Return a usable spool path, or None to run without spooling.

        An unwritable /var/lib/fpms (a laptop selftest, a read-only rootfs) must
        degrade to "no spool", never crash the uplink. Losing buffered events is
        bad; losing the whole uplink over it is worse.
        """
        if not path:
            return None
        try:
            parent = os.path.dirname(os.path.abspath(path))
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(path, "a", encoding="utf-8"):
                pass
            return path
        except Exception as e:  # noqa: BLE001
            log(f"uplink: spooling disabled ({path}: {e})")
            return None

    def _spool_events(self, items):
        if not self.spool_path or not items:
            return
        try:
            with open(self.spool_path, "a", encoding="utf-8") as f:
                for it in items:
                    f.write(json.dumps(it) + "\n")
            self._trim_spool()
        except Exception as e:  # noqa: BLE001
            log(f"uplink: spool write failed: {e}")

    def _trim_spool(self):
        """Cap the spool by dropping the OLDEST half once it exceeds the limit.

        Half rather than one line at a time so a long outage does not rewrite
        the file on every append.
        """
        try:
            if os.path.getsize(self.spool_path) <= SPOOL_MAX_BYTES:
                return
            with open(self.spool_path, "r", encoding="utf-8") as f:
                lines = f.readlines()
            keep = lines[len(lines) // 2:]
            with open(self.spool_path, "w", encoding="utf-8") as f:
                f.writelines(keep)
            log(f"uplink: spool trimmed to {len(keep)} events")
        except Exception as e:  # noqa: BLE001
            log(f"uplink: spool trim failed: {e}")

    def _drain_spool(self):
        """Send spooled events after a reconnect, oldest first.

        The file is only truncated once a chunk is accepted, so a failure
        mid-drain leaves the backlog on disk for the next attempt.
        """
        if not self.spool_path:
            return
        try:
            if not os.path.getsize(self.spool_path):
                return
        except Exception:  # noqa: BLE001
            return
        try:
            with open(self.spool_path, "r", encoding="utf-8") as f:
                lines = [ln for ln in f.read().splitlines() if ln.strip()]
        except Exception as e:  # noqa: BLE001
            log(f"uplink: spool read failed: {e}")
            return
        if not lines:
            return

        log(f"uplink: draining {len(lines)} spooled events")
        remaining = list(lines)
        while remaining and not self._stop.is_set():
            chunk, remaining = remaining[:MAX_ITEMS], remaining[MAX_ITEMS:]
            items = []
            for ln in chunk:
                try:
                    items.append(json.loads(ln))
                except Exception:  # noqa: BLE001
                    continue        # a torn line from a power cut; skip it
            if items and not self._send_batch(items, spool_on_fail=False):
                remaining = chunk + remaining
                break
        try:
            with open(self.spool_path, "w", encoding="utf-8") as f:
                f.writelines(ln + "\n" for ln in remaining)
        except Exception as e:  # noqa: BLE001
            log(f"uplink: spool rewrite failed: {e}")

    # ---- ingest ----------------------------------------------------------

    @property
    def enabled(self):
        return bool(self.url and self.token)

    def offer(self, subtype, payload, kind="telemetry"):
        """Queue one reading. Called from sensor loops: never blocks, never raises."""
        try:
            if not self.enabled:
                return
            kind, subtype = _clean_subtype(subtype, kind)
            # Shallow copy + the same ts/thing defaulting Bus.publish does, so
            # an item looks identical whichever path carried it. Shallow is
            # deliberate: a deep copy of a 28 KB frame on the hot loop is real
            # CPU, and the agent does not mutate a payload after publishing it.
            data = dict(payload) if isinstance(payload, dict) else {"value": payload}
            data.setdefault("ts", time.time())
            data.setdefault("thing", self.thing)
            item = {"thing": self.thing, "subtype": subtype,
                    "kind": kind, "data": data}

            with self._lock:
                self.offered += 1
                if kind == "events":
                    if len(self._events) == EVENT_QUEUE_MAX:
                        self.dropped_events += 1
                    self._events.append(item)
                else:
                    # Newest wins: overwriting is the point, not a loss.
                    if subtype in self._latest:
                        self.dropped_telemetry += 1
                    self._latest[subtype] = item
        except Exception:  # noqa: BLE001
            # Deliberately silent and total. Nothing about an optional uplink
            # justifies interrupting a sensor loop, including logging failures.
            try:
                self.errors += 1
            except Exception:  # noqa: BLE001
                pass

    # ---- rate ------------------------------------------------------------

    def _camera_interval(self):
        """Seconds between camera frames, honouring both caps.

        The local `fps` is a ceiling the operator set; the server's camera_fps
        is what the dashboard actually needs right now. The lower of the two
        wins, and zero means trickle rather than stop.
        """
        eff = self.fps
        if self._server_fps is not None:
            eff = min(eff, self._server_fps)
        if eff <= 0:
            return TRICKLE_INTERVAL_S
        return max(1.0 / eff, 0.0)

    def _handle_server_message(self, msg):
        text = (msg or "").strip()
        if not text:
            return
        if text == "ping":
            if isinstance(self._transport, _WsTransport):
                self._transport.pong()
            return
        try:
            obj = json.loads(text)
        except Exception:  # noqa: BLE001
            return
        if isinstance(obj, dict) and obj.get("cmd") == "rate":
            try:
                fps = max(0.0, float(obj.get("camera_fps", 0)))
            except (TypeError, ValueError):
                return
            if fps != self._server_fps:
                log(f"uplink: server rate command camera_fps={fps}")
            self._server_fps = fps
            with self._lock:
                self.rate_commands.append(fps)

    # ---- batching --------------------------------------------------------

    def _collect(self):
        """Build one tick's array: all events, plus rate-permitted telemetry."""
        now = time.monotonic()
        camera_due = (now - self._last_camera_send) >= self._camera_interval()
        items = []
        camera_taken = 0

        with self._lock:
            # Events first and unconditionally — a fire alert must never wait
            # out a camera rate cap, and putting them at the head of the array
            # means a byte-trim drops frames rather than alerts.
            while self._events and len(items) < MAX_ITEMS:
                items.append(self._events.popleft())

            for subtype in sorted(self._latest):
                if len(items) >= MAX_ITEMS:
                    break
                if _is_camera(subtype):
                    # Gated: leave it in the slot so it keeps being replaced by
                    # fresher frames, and we send the newest when the gate opens.
                    if not camera_due or camera_taken >= MAX_CAMERA_ITEMS:
                        continue
                    camera_taken += 1
                items.append(self._latest.pop(subtype))

        if camera_taken:
            self._last_camera_send = now
        return items

    def _encode(self, items):
        """Serialise, dropping camera items until the body fits the server's cap.

        Trimming from the tail drops telemetry before events because _collect()
        puts events first. Returns (body_bytes, items_actually_encoded).
        """
        keep = list(items)
        while keep:
            body = json.dumps(keep, separators=(",", ":")).encode("utf-8")
            if len(body) <= BODY_TARGET_BYTES:
                return body, keep

            # Shed the largest camera frame first: frames are the only items big
            # enough to matter and the only ones we are willing to lose.
            victims = [i for i, it in enumerate(keep)
                       if _is_camera(it.get("subtype", ""))]
            if victims:
                drop = max(victims, key=lambda i: len(json.dumps(keep[i])))
                keep.pop(drop)
                with self._lock:
                    self.dropped_telemetry += 1
                continue

            # No frames left and still over the cap. Drop ONLY the single
            # largest item, never a slice: an earlier version halved the list
            # here, which silently discarded events sitting in the tail and so
            # lost exactly the alerts this uplink exists to deliver.
            drop = max(range(len(keep)), key=lambda i: len(json.dumps(keep[i])))
            lost = keep.pop(drop)
            log(f"uplink: dropping oversized {lost.get('kind')}/{lost.get('subtype')} "
                f"item; it cannot fit the {MAX_BODY_BYTES}-byte ingest limit")
            with self._lock:
                if lost.get("kind") == "events":
                    self.dropped_events += 1
                else:
                    self.dropped_telemetry += 1
        return None, []

    # ---- transport -------------------------------------------------------

    def _make_transport(self):
        """Pick a transport per mode, preferring the WebSocket when allowed."""
        if self.mode in ("ws", "auto") and _WS_AVAILABLE and self.ws_url:
            try:
                t = _WsTransport(self.ws_url, self.token, self.thing)
                t.connect()
                return t
            except Exception as e:  # noqa: BLE001
                self.last_error = f"ws connect: {e}"
                if self.mode == "ws":
                    raise
                log(f"uplink: websocket failed ({e}); falling back to HTTPS")
        elif self.mode == "ws":
            raise RuntimeError(
                f"mode=ws but websocket unavailable (ws_url={self.ws_url!r}, "
                f"import={_WS_IMPORT_ERROR})")

        t = _HttpTransport(self.url, self.token)
        t.connect()
        return t

    def _send_batch(self, items, spool_on_fail=True):
        """Encode and send one array. Returns True on acceptance."""
        if not items or self._transport is None:
            return True
        body, encoded = self._encode(items)
        if not body:
            return True
        try:
            ok, detail = self._transport.send(body)
        except Exception as e:  # noqa: BLE001
            self.last_error = str(e)
            with self._lock:
                self.errors += 1
            if spool_on_fail:
                self._spool_events([i for i in encoded if i.get("kind") == "events"])
            self._drop_transport()
            return False

        if ok:
            with self._lock:
                self.sent += len(encoded)
                self.batches += 1
            self.last_error = None
            return True

        # An application-level rejection. Retrying the same bytes will fail the
        # same way, so the batch is dropped rather than looped — except events,
        # which go to the spool so the audit trail survives a transient 5xx.
        self.last_error = detail
        with self._lock:
            self.errors += 1
        if detail and ("401" in detail or "403" in detail):
            log(f"uplink: rejected ({detail}) - check FPMS_INGEST_TOKEN")
            self._backoff = BACKOFF_MAX_S
            self._drop_transport()
        else:
            log(f"uplink: batch rejected ({detail})")
        if spool_on_fail:
            self._spool_events([i for i in encoded if i.get("kind") == "events"])
        return False

    def _drop_transport(self):
        if self._transport is not None:
            self._transport.close()
            self._transport = None
        self.connected_as = None

    def _sleep_backoff(self):
        """Exponential 1s -> 30s with jitter.

        Jitter matters with more than one rover: without it a fleet that lost
        the same tower reconnects in lockstep and self-inflicts a thundering
        herd on the Worker.
        """
        delay = min(self._backoff, BACKOFF_MAX_S)
        delay *= 0.5 + random.random()          # +/- 50%
        self._stop.wait(min(delay, BACKOFF_MAX_S))
        self._backoff = min(self._backoff * 2.0, BACKOFF_MAX_S)

    # ---- worker ----------------------------------------------------------

    def _run(self):
        while not self._stop.is_set():
            try:
                if self._transport is None:
                    try:
                        self._transport = self._make_transport()
                    except Exception as e:  # noqa: BLE001
                        self.last_error = str(e)
                        with self._lock:
                            self.errors += 1
                        log(f"uplink: connect failed ({e}); retrying")
                        self._sleep_backoff()
                        continue
                    self.connected_as = self._transport.name
                    self._backoff = BACKOFF_MIN_S
                    log(f"uplink: connected via {self._transport.name}")
                    self._drain_spool()

                for msg in self._transport.poll():
                    self._handle_server_message(msg)

                items = self._collect()
                if items:
                    self._send_batch(items)

                self._stop.wait(TICK_S)
            except Exception as e:  # noqa: BLE001
                # The thread is the last line of defence. Anything unhandled
                # here would end the uplink silently for the rest of the run.
                self.last_error = str(e)
                with self._lock:
                    self.errors += 1
                log(f"uplink: loop error ({e}); resetting transport")
                try:
                    self._drop_transport()
                except Exception:  # noqa: BLE001
                    pass
                self._sleep_backoff()

        self._drop_transport()

    def start(self):
        try:
            if not self.enabled:
                log("uplink: not configured (need URL + token); staying off")
                return False
            if self._thread is not None and self._thread.is_alive():
                return True
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True,
                                            name="fpms-cloud-uplink")
            self._thread.start()
            log(f"uplink: started thing={self.thing} mode={self.mode} "
                f"fps={self.fps} url={self.url} "
                f"ws={'yes' if (_WS_AVAILABLE and self.ws_url) else 'no'} "
                f"spool={self.spool_path or 'disabled'}")
            return True
        except Exception as e:  # noqa: BLE001
            log(f"uplink: start failed: {e}")
            return False

    def stop(self, timeout=2.0):
        try:
            self._stop.set()
            t = self._thread
            if t is not None and t.is_alive():
                t.join(timeout=timeout)
            self._thread = None
            # Anything still queued is events-only worth keeping; frames are
            # stale the moment we stop.
            with self._lock:
                pending = [i for i in self._events]
                self._events.clear()
            self._spool_events(pending)
            return True
        except Exception as e:  # noqa: BLE001
            log(f"uplink: stop failed: {e}")
            return False

    def status(self):
        try:
            with self._lock:
                return {
                    "enabled": self.enabled,
                    "transport": self.connected_as,
                    "mode": self.mode,
                    "url": self.url,
                    "ws_url": self.ws_url,
                    "fps": self.fps,
                    "server_camera_fps": self._server_fps,
                    "offered": self.offered,
                    "sent": self.sent,
                    "batches": self.batches,
                    "errors": self.errors,
                    "dropped_events": self.dropped_events,
                    "superseded_telemetry": self.dropped_telemetry,
                    "queued_events": len(self._events),
                    "queued_telemetry": len(self._latest),
                    "rate_commands": list(self.rate_commands),
                    "spool": self.spool_path,
                    "last_error": self.last_error,
                }
        except Exception as e:  # noqa: BLE001
            return {"enabled": False, "last_error": f"status failed: {e}"}


def from_config():
    """Build an uplink from /etc/fpms/config.env, or None if not opted in.

    This is the intended integration point: the agent calls it, gets None
    unless FPMS_CLOUD_UPLINK is set, and treats None as "no uplink".
    """
    try:
        if not UPLINK_ENABLED:
            return None
        if not INGEST_URL or not INGEST_TOKEN:
            log("uplink: FPMS_CLOUD_UPLINK set but URL/token missing; staying off")
            return None
        return CloudUplink(THING, INGEST_URL, INGEST_WS_URL, INGEST_TOKEN,
                           mode=UPLINK_MODE, fps=UPLINK_FPS,
                           spool_path=SPOOL_PATH)
    except Exception as e:  # noqa: BLE001
        log(f"uplink: from_config failed: {e}")
        return None


# -------------------------------------------------------------- selftest ---

def _fake_frame(kb=20):
    """Base64 of random bytes, sized like a real JPEG frame.

    Deliberately not cv2/numpy: the whole point of the selftest is that it runs
    on a laptop with plain CPython while the Orange Pi is offline.
    """
    return base64.b64encode(random.randbytes(kb * 1024)).decode("ascii")


def _selftest(args):
    url = args.url
    ws_url = args.ws_url or _derive_ws_url(url)
    spool = args.spool
    if spool is None:
        # Never write to /var/lib/fpms from a laptop selftest.
        spool = os.path.join(os.path.expanduser("~"), ".fpms-uplink-selftest.jsonl")

    log(f"selftest: url={url} ws={ws_url or 'none'} mode={args.mode} "
        f"fps={args.fps} seconds={args.seconds}")
    log(f"selftest: websocket-client {'available' if _WS_AVAILABLE else 'NOT available (' + str(_WS_IMPORT_ERROR) + ')'}")

    up = CloudUplink(args.thing, url, ws_url, args.token,
                     mode=args.mode, fps=args.fps, spool_path=spool)
    up.start()

    t0 = time.monotonic()
    deadline = t0 + args.seconds
    interval = 1.0 / max(args.fps, 0.1)
    frames = 0
    # One event up front: proves the events path and, on failure, the spool.
    up.offer("events/fire", {"alert": True, "source": "selftest",
                             "detail": "synthetic fire event"}, kind="events")

    while time.monotonic() < deadline:
        tick = time.monotonic()
        frames += 1
        up.offer("telemetry/camera", {
            "format": "jpeg", "frame": _fake_frame(20),
            "width": 640, "height": 480,
            "detections": [{"cls": 0, "label": "person", "conf": 0.9,
                            "box": [10, 10, 100, 200], "kind": "person"}],
            "npu": False, "fire_like": False, "fire_ratio": 0.0,
        })
        up.offer("telemetry/pose", {
            "uptime_s": round(time.monotonic() - t0, 1),
            "camera_ok": True, "lidar_ok": False,
            "frames": frames, "scans": 0, "streaming": True,
        })
        time.sleep(max(0.0, interval - (time.monotonic() - tick)))

    # Give the send loop a moment to flush whatever is pending.
    time.sleep(0.5)
    st = up.status()
    up.stop()
    elapsed = time.monotonic() - t0

    log("---- selftest summary ----")
    log(f"  elapsed          : {elapsed:.1f}s")
    log(f"  transport        : {st['transport'] or 'never connected'}")
    log(f"  items offered    : {st['offered']}")
    log(f"  items sent       : {st['sent']}")
    log(f"  batches          : {st['batches']}")
    log(f"  errors           : {st['errors']}")
    log(f"  observed rate    : {st['sent'] / elapsed:.2f} items/s "
        f"({st['batches'] / elapsed:.2f} batches/s)")
    log(f"  frames generated : {frames}")
    log(f"  superseded (newest-wins drops) : {st['superseded_telemetry']}")
    log(f"  events dropped (FIFO overflow) : {st['dropped_events']}")
    log(f"  still queued     : {st['queued_events']} events, "
        f"{st['queued_telemetry']} telemetry")
    log(f"  rate commands    : {st['rate_commands'] or 'none received'}")
    log(f"  spool            : {st['spool'] or 'disabled'}")
    log(f"  last error       : {st['last_error']}")

    spooled = 0
    if st["spool"] and os.path.exists(st["spool"]):
        try:
            with open(st["spool"], encoding="utf-8") as f:
                spooled = sum(1 for ln in f if ln.strip())
        except Exception:  # noqa: BLE001
            pass
    log(f"  spooled events   : {spooled}")

    reachable = st["sent"] > 0
    log(f"  verdict          : {'uplink delivered' if reachable else 'unreachable - degraded without raising'}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="FPMS rover cloud uplink")
    ap.add_argument("--selftest", action="store_true",
                    help="generate synthetic telemetry and report what got through")
    ap.add_argument("--url", default=INGEST_URL, help="https://.../ingest")
    ap.add_argument("--ws-url", default=INGEST_WS_URL,
                    help="wss://.../ingest/ws (derived from --url if omitted)")
    ap.add_argument("--token", default=INGEST_TOKEN)
    ap.add_argument("--thing", default=THING)
    ap.add_argument("--mode", default="auto", choices=["auto", "ws", "http"])
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--spool", default=None)
    args = ap.parse_args(argv)

    if not args.selftest:
        ap.error("nothing to do; pass --selftest (this module is a library)")
    if not args.url:
        ap.error("--url is required")
    if not args.token:
        args.token = "selftest-no-token"
        log("selftest: no --token given; using a placeholder (expect 401)")

    try:
        return _selftest(args)
    except KeyboardInterrupt:
        log("selftest: interrupted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
