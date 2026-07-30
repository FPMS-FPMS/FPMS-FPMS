"""Forward rover telemetry from the edge app up to the FPMS cloud app.

The rover publishes to the local broker at full rate — 2 camera frames and 2
LiDAR summaries a second. Sending all of that to the cloud would be roughly
345,000 requests a day, well past the Workers free tier's 100,000, and would
burn cellular data for no benefit: nobody watching a remote dashboard needs
every frame.

So telemetry is throttled per channel while **events are always forwarded
immediately** — a fire alert must never wait out a rate limit.

Configure via settings (~/.fpms/settings.json) or environment:
    cloud_ingest_url / FPMS_CLOUD_INGEST_URL
    cloud_ingest_token / FPMS_CLOUD_INGEST_TOKEN
    cloud_min_interval_s / FPMS_CLOUD_MIN_INTERVAL_S   (default 15)
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
import urllib.error
import urllib.request
from queue import Empty, Full, Queue
from typing import Any

from . import settings_store

log = logging.getLogger("fpms.cloud")

_BATCH_MAX = 20
_POST_TIMEOUT_S = 15
_QUEUE_MAX = 200          # bounded: a cloud outage must not grow memory forever


def _cfg(key: str, env: str, default: Any = None) -> Any:
    value = os.environ.get(env)
    if value not in (None, ""):
        return value
    stored = settings_store.get(key)
    return stored if stored not in (None, "") else default


class CloudForwarder:
    def __init__(self) -> None:
        self.queue: Queue = Queue(maxsize=_QUEUE_MAX)
        self._last_sent: dict[str, float] = {}
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.sent = 0
        self.dropped = 0
        self.failures = 0
        self.last_error: str | None = None

    # ---- configuration ---------------------------------------------------

    @property
    def url(self) -> str | None:
        return _cfg("cloud_ingest_url", "FPMS_CLOUD_INGEST_URL")

    @property
    def token(self) -> str | None:
        return _cfg("cloud_ingest_token", "FPMS_CLOUD_INGEST_TOKEN")

    @property
    def min_interval(self) -> float:
        try:
            return float(_cfg("cloud_min_interval_s", "FPMS_CLOUD_MIN_INTERVAL_S", 15))
        except (TypeError, ValueError):
            return 15.0

    @property
    def enabled(self) -> bool:
        return bool(self.url and self.token)

    # ---- ingest ----------------------------------------------------------

    def offer(self, thing: str, subtype: str, kind: str, data: dict[str, Any]) -> None:
        """Called from the MQTT thread. Never blocks, never raises."""
        if not self.enabled:
            return

        key = f"{kind}:{subtype}:{thing}"
        now = time.monotonic()
        if kind != "events":
            last = self._last_sent.get(key, 0.0)
            if now - last < self.min_interval:
                return          # throttled — the local dashboard still gets it live
            self._last_sent[key] = now

        try:
            self.queue.put_nowait({"thing": thing, "subtype": subtype, "kind": kind, "data": data})
        except Full:
            self.dropped += 1
            if self.dropped % 50 == 1:
                log.warning("cloud queue full; dropped %d readings so far", self.dropped)

    # ---- worker ----------------------------------------------------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="fpms-cloud-forwarder")
        self._thread.start()
        log.info("cloud forwarder started (url=%s, min_interval=%ss)", self.url, self.min_interval)

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            batch = []
            try:
                batch.append(self.queue.get(timeout=1.0))
            except Empty:
                continue
            # Opportunistically batch whatever else is waiting.
            while len(batch) < _BATCH_MAX:
                try:
                    batch.append(self.queue.get_nowait())
                except Empty:
                    break
            self._post(batch)

    def _post(self, batch: list[dict[str, Any]]) -> None:
        url, token = self.url, self.token
        if not url or not token:
            return
        req = urllib.request.Request(
            url,
            data=json.dumps(batch).encode("utf-8"),
            method="POST",
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {token}",
                # Cloudflare answers the default Python UA with a 403.
                "User-Agent": "FPMS-Edge/1.0",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=_POST_TIMEOUT_S) as r:
                if 200 <= r.status < 300:
                    self.sent += len(batch)
                    self.last_error = None
                    return
                self.failures += 1
                self.last_error = f"HTTP {r.status}"
        except urllib.error.HTTPError as e:
            self.failures += 1
            detail = ""
            try:
                detail = e.read().decode("utf-8", "replace")[:200]
            except Exception:  # noqa: BLE001
                pass
            self.last_error = f"HTTP {e.code}: {detail}" if detail else f"HTTP {e.code}"
        except Exception as e:  # noqa: BLE001
            self.failures += 1
            self.last_error = str(e)

        if self.failures % 20 == 1:
            log.warning("cloud forward failed: %s", self.last_error)

    def status(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "url": self.url,
            "min_interval_s": self.min_interval,
            "queued": self.queue.qsize(),
            "sent": self.sent,
            "dropped": self.dropped,
            "failures": self.failures,
            "last_error": self.last_error,
        }


forwarder = CloudForwarder()
