"""Per-topic WebSocket fanout.

The MQTT bridge produces messages; connected browsers subscribe to a named
channel (`lidar:rover1`, `camera:rover1`, `thermal:rover1`, ...) and receive
each new message as JSON. Slow clients get dropped rather than blocking the
publisher — the dashboard is a live-view tool, not a delivery guarantee.
"""
from __future__ import annotations

import asyncio
import logging
from collections import defaultdict
from typing import Any

from fastapi import WebSocket

log = logging.getLogger("fpms.hub")


class Hub:
    def __init__(self) -> None:
        self._subscribers: dict[str, set[WebSocket]] = defaultdict(set)
        self._latest: dict[str, Any] = {}
        self._lock = asyncio.Lock()

    async def connect(self, channel: str, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self._subscribers[channel].add(ws)
        # Immediately replay the last known frame so the UI isn't blank while
        # waiting for the next tick from the rover.
        latest = self._latest.get(channel)
        if latest is not None:
            try:
                await ws.send_json(latest)
            except Exception:  # noqa: BLE001
                await self.disconnect(channel, ws)

    async def disconnect(self, channel: str, ws: WebSocket) -> None:
        async with self._lock:
            self._subscribers[channel].discard(ws)

    async def broadcast(self, channel: str, payload: Any) -> None:
        self._latest[channel] = payload
        # Snapshot subscribers under the lock so a slow client can't hold up
        # the MQTT loop while we iterate.
        async with self._lock:
            targets = list(self._subscribers[channel])
        dead: list[WebSocket] = []
        for ws in targets:
            try:
                await ws.send_json(payload)
            except Exception:  # noqa: BLE001
                dead.append(ws)
        if dead:
            async with self._lock:
                for ws in dead:
                    self._subscribers[channel].discard(ws)

    def subscriber_count(self, channel: str) -> int:
        return len(self._subscribers.get(channel, ()))

    def channels(self) -> list[str]:
        return sorted(self._subscribers.keys() | self._latest.keys())


hub = Hub()
