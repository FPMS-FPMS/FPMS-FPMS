"""On-the-fly analysis of thermal frames.

This isn't a neural net — it's a small, explainable statistics + connected-
component pass that answers three questions per frame:

  1. What are the temperature statistics right now?
  2. Where are the hotspots (contiguous regions above a threshold)?
  3. Given a short history, are the hotspots growing, shrinking, or steady?

The output is human-readable JSON that the dashboard uses to render a live
"Analyst" panel. Cross-validation with the RGB camera still happens on the
rover — this is a summary layer, not a decision layer.
"""
from __future__ import annotations

import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np


HOT_THRESHOLD_C = 55.0        # anything above this is a candidate hotspot
CRITICAL_THRESHOLD_C = 80.0   # anything above this triggers a critical flag
HISTORY_SECONDS = 30          # trend window


@dataclass
class Frame:
    ts: float
    hot_pixel_count: int
    max_c: float
    mean_c: float


@dataclass
class ThermalAnalyzer:
    history: deque[Frame] = field(default_factory=lambda: deque(maxlen=600))

    def analyze(self, grid: list[list[float]]) -> dict[str, Any]:
        arr = np.asarray(grid, dtype=np.float32)
        if arr.ndim != 2 or arr.size == 0:
            return {"error": "empty or malformed thermal grid"}

        stats = {
            "min_c": float(arr.min()),
            "max_c": float(arr.max()),
            "mean_c": float(arr.mean()),
            "std_c": float(arr.std()),
            "shape": list(arr.shape),
        }

        hot_mask = arr >= HOT_THRESHOLD_C
        hot_pixel_count = int(hot_mask.sum())

        hotspots = _find_hotspots(arr, hot_mask)

        frame = Frame(
            ts=time.time(),
            hot_pixel_count=hot_pixel_count,
            max_c=stats["max_c"],
            mean_c=stats["mean_c"],
        )
        self.history.append(frame)
        trend = self._trend()

        severity = _severity(stats["max_c"], hot_pixel_count, arr.size)
        report = _report(stats, hotspots, trend, severity)

        return {
            "stats": stats,
            "hotspots": hotspots,
            "trend": trend,
            "severity": severity,
            "report": report,
            "thresholds": {
                "hot_c": HOT_THRESHOLD_C,
                "critical_c": CRITICAL_THRESHOLD_C,
            },
        }

    def _trend(self) -> dict[str, Any]:
        if len(self.history) < 4:
            return {"direction": "unknown", "delta_max_c": 0.0, "samples": len(self.history)}
        cutoff = time.time() - HISTORY_SECONDS
        recent = [f for f in self.history if f.ts >= cutoff]
        if len(recent) < 4:
            recent = list(self.history)[-4:]

        # Split window in halves; compare mean max temp.
        mid = len(recent) // 2
        first = statistics.mean(f.max_c for f in recent[:mid]) if mid else recent[0].max_c
        second = statistics.mean(f.max_c for f in recent[mid:])
        delta = second - first

        if delta > 1.5:
            direction = "rising"
        elif delta < -1.5:
            direction = "falling"
        else:
            direction = "steady"
        return {
            "direction": direction,
            "delta_max_c": round(delta, 2),
            "samples": len(recent),
            "window_s": HISTORY_SECONDS,
        }


def _find_hotspots(arr: np.ndarray, mask: np.ndarray) -> list[dict[str, Any]]:
    """Tiny 4-connected-component labeler — no scipy dep."""
    if not mask.any():
        return []
    h, w = mask.shape
    labels = np.zeros_like(mask, dtype=np.int32)
    next_label = 0
    stack: list[tuple[int, int]] = []
    components: list[list[tuple[int, int]]] = []
    for y in range(h):
        for x in range(w):
            if not mask[y, x] or labels[y, x]:
                continue
            next_label += 1
            stack.append((y, x))
            comp: list[tuple[int, int]] = []
            while stack:
                cy, cx = stack.pop()
                if cy < 0 or cy >= h or cx < 0 or cx >= w:
                    continue
                if labels[cy, cx] or not mask[cy, cx]:
                    continue
                labels[cy, cx] = next_label
                comp.append((cy, cx))
                stack.extend(((cy + 1, cx), (cy - 1, cx), (cy, cx + 1), (cy, cx - 1)))
            components.append(comp)

    out = []
    for i, comp in enumerate(components, start=1):
        ys = [p[0] for p in comp]
        xs = [p[1] for p in comp]
        vals = [float(arr[y, x]) for y, x in comp]
        out.append({
            "id": i,
            "pixels": len(comp),
            "centroid": [round(sum(xs) / len(xs), 2), round(sum(ys) / len(ys), 2)],
            "bbox": [min(xs), min(ys), max(xs), max(ys)],
            "peak_c": round(max(vals), 2),
            "mean_c": round(sum(vals) / len(vals), 2),
        })
    out.sort(key=lambda h: h["peak_c"], reverse=True)
    return out[:6]  # cap noise — top 6 by peak temp


def _severity(peak_c: float, hot_pixels: int, total_pixels: int) -> str:
    fraction = hot_pixels / total_pixels if total_pixels else 0.0
    if peak_c >= CRITICAL_THRESHOLD_C or fraction > 0.15:
        return "critical"
    if peak_c >= HOT_THRESHOLD_C or fraction > 0.03:
        return "elevated"
    return "nominal"


def _report(stats: dict[str, Any], hotspots: list[dict[str, Any]],
            trend: dict[str, Any], severity: str) -> str:
    lines = [
        f"Thermal scene {severity.upper()}. "
        f"Peak {stats['max_c']:.1f} °C, mean {stats['mean_c']:.1f} °C.",
    ]
    if hotspots:
        top = hotspots[0]
        lines.append(
            f"{len(hotspots)} hotspot region(s). Largest peak {top['peak_c']} °C at "
            f"({top['centroid'][0]}, {top['centroid'][1]}) covering {top['pixels']} px."
        )
    else:
        lines.append("No pixels above the hot threshold — scene is cool.")
    if trend["direction"] == "rising":
        lines.append(f"Trend: peak temperature RISING (+{trend['delta_max_c']} °C over {trend['window_s']}s).")
    elif trend["direction"] == "falling":
        lines.append(f"Trend: cooling (Δ {trend['delta_max_c']} °C).")
    elif trend["direction"] == "steady":
        lines.append("Trend: steady.")
    if severity == "critical":
        lines.append("Recommendation: cross-check with RGB camera; prepare suppression pump.")
    elif severity == "elevated":
        lines.append("Recommendation: continue observation; log event if hotspot persists.")
    else:
        lines.append("Recommendation: continue passive monitoring.")
    return " ".join(lines)


analyzer = ThermalAnalyzer()
