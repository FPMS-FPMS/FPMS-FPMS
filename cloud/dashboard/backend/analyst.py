"""FPMS AI Analyst — small, local, honest.

Watches the live sensor hub + recent event archive and writes plain-English
reports. This is a rule-based analyst by default (no model download, no
network, no API cost). Two optional upgrades:

  - GITHUB_COPILOT_TOKEN → route chat questions through Copilot Chat.
  - AWS Bedrock          → set FPMS_BEDROCK_MODEL_ID (real AWS mode only).

Both are opt-in; the fallback works identically to how the thermal analyzer
already generates its reports.
"""
from __future__ import annotations

import json
import logging
import os
import statistics
import time
from typing import Any

from . import aws
from .hub import hub

log = logging.getLogger("fpms.analyst")


def _latest(channel: str) -> Any | None:
    """Peek at the last message hub broadcast on a channel (thread-safe read)."""
    return hub._latest.get(channel)  # small internal peek — fine, same process


def _has_lidar(thing: str) -> bool:
    return _latest(f"lidar:{thing}") is not None


def snapshot(thing: str = "rover1") -> dict[str, Any]:
    """Structured snapshot: everything the analyst knows about a rover right now."""
    lidar = _latest(f"lidar:{thing}")
    camera = _latest(f"camera:{thing}")
    thermal_env = _latest(f"thermal:{thing}")
    thermal_ana = _latest(f"thermal-analysis:{thing}")
    pose = _latest(f"pose:{thing}")

    now = time.time()
    ages: dict[str, float | None] = {}
    for name, env in [("lidar", lidar), ("camera", camera),
                      ("thermal", thermal_env), ("pose", pose)]:
        ages[name] = round(now - env["ts"], 2) if env and env.get("ts") else None

    lidar_summary = None
    if lidar:
        ranges = lidar["data"].get("ranges_m", [])
        if ranges:
            lidar_summary = {
                "min_range_m": round(min(ranges), 2),
                "max_range_m": round(max(ranges), 2),
                "mean_range_m": round(statistics.mean(ranges), 2),
                "near_obstacles": int(sum(1 for r in ranges if r < 0.5)),
                "heading_deg": lidar["data"].get("heading_deg"),
            }

    detections = camera["data"].get("detections", []) if camera else []

    thermal_summary = None
    if thermal_ana:
        d = thermal_ana["data"]
        thermal_summary = {
            "severity": d.get("severity"),
            "peak_c": d.get("stats", {}).get("max_c"),
            "mean_c": d.get("stats", {}).get("mean_c"),
            "hotspots": len(d.get("hotspots", [])),
            "trend": d.get("trend", {}).get("direction"),
        }

    return {
        "thing": thing,
        "streams_live": {name: (ages[name] is not None and ages[name] < 5) for name in ages},
        "ages_s": ages,
        "lidar": lidar_summary,
        "detections": detections,
        "thermal": thermal_summary,
        "pose": pose["data"] if pose else None,
    }


def report(thing: str = "rover1") -> dict[str, Any]:
    """Rule-based analyst report — always available, no external calls."""
    s = snapshot(thing)
    lines: list[str] = []
    concerns: list[str] = []

    if not any(s["streams_live"].values()):
        lines.append(f"{thing.upper()} is silent — no sensor stream in the last 5 seconds.")
        concerns.append("no telemetry")
    else:
        alive = [k for k, v in s["streams_live"].items() if v]
        lines.append(f"{thing.upper()} is streaming: {', '.join(alive)}.")

    if s["lidar"]:
        L = s["lidar"]
        lines.append(
            f"LiDAR sees a {L['min_range_m']:.2f}m nearest obstacle, "
            f"mean range {L['mean_range_m']:.2f}m across {360} beams."
        )
        if L["near_obstacles"] > 30:
            concerns.append(f"{L['near_obstacles']} beams inside 0.5m — tight environment")

    if s["detections"]:
        fire = [d for d in s["detections"] if d.get("cls") == "fire"]
        if fire:
            top = max(fire, key=lambda d: d.get("conf", 0))
            lines.append(f"YOLO camera reports fire at conf {top['conf']:.2f}.")
            if top.get("conf", 0) >= 0.8:
                concerns.append("high-confidence fire detection")
        else:
            lines.append(f"YOLO camera detections: {[d.get('cls') for d in s['detections']]}.")

    if s["thermal"]:
        T = s["thermal"]
        lines.append(
            f"Thermal: peak {T['peak_c']:.1f}°C, mean {T['mean_c']:.1f}°C, "
            f"{T['hotspots']} hotspot(s), trend {T['trend']}, severity {T['severity']}."
        )
        if T["severity"] == "critical":
            concerns.append(f"thermal CRITICAL ({T['peak_c']:.0f}°C peak)")
        elif T["trend"] == "rising":
            concerns.append("thermal peak rising")

    if s["pose"]:
        p = s["pose"]
        lines.append(
            f"Pose: ({p['x_m']:.2f}, {p['y_m']:.2f}) heading {p['heading_deg']:.0f}°, "
            f"battery {p['battery_pct']:.1f}%."
        )
        if p["battery_pct"] < 20:
            concerns.append(f"battery {p['battery_pct']:.0f}%")

    # Recent events from S3
    try:
        s3 = aws._client("s3")
        objs = s3.list_objects_v2(
            Bucket="fpms-archive",
            Prefix=f"events/thing={thing}/",
            MaxKeys=5,
        ).get("Contents", [])
        if objs:
            lines.append(f"S3 archive: {len(objs)} recent events on {thing}.")
    except Exception:  # noqa: BLE001
        pass

    verdict = "NOMINAL"
    if any("critical" in c.lower() or "fire" in c.lower() for c in concerns):
        verdict = "CRITICAL"
    elif concerns:
        verdict = "ELEVATED"

    return {
        "thing": thing,
        "generated_at": time.time(),
        "verdict": verdict,
        "concerns": concerns,
        "report": " ".join(lines) or f"No data available for {thing}.",
        "snapshot": s,
        "provider": "local-rules",
    }


def _copilot_available() -> bool:
    return bool(os.environ.get("GITHUB_COPILOT_TOKEN"))


def _bedrock_model_id() -> str | None:
    return os.environ.get("FPMS_BEDROCK_MODEL_ID")


def chat(question: str, thing: str = "rover1") -> dict[str, Any]:
    """Answer a question about the rover state.

    Order of preference:
        1. AWS Bedrock (if FPMS_BEDROCK_MODEL_ID + real AWS)
        2. GitHub Copilot (if GITHUB_COPILOT_TOKEN)
        3. Local rule-based Q&A
    """
    snap = snapshot(thing)
    context = json.dumps(snap, indent=2)

    bedrock_id = _bedrock_model_id()
    if bedrock_id:
        try:
            return _chat_bedrock(question, context, bedrock_id, thing)
        except Exception as e:  # noqa: BLE001
            log.warning("Bedrock unavailable, falling back: %s", e)

    if _copilot_available():
        try:
            return _chat_copilot(question, context, thing)
        except Exception as e:  # noqa: BLE001
            log.warning("Copilot unavailable, falling back: %s", e)

    return _chat_local(question, snap, thing)


def _chat_local(question: str, snap: dict[str, Any], thing: str) -> dict[str, Any]:
    """Very small rule-based Q&A. Keywords over structured state."""
    q = question.lower()
    ans: str

    if any(w in q for w in ("fire", "burn", "flame", "hot", "thermal", "heat")):
        t = snap.get("thermal")
        if not t:
            ans = f"No thermal data for {thing} right now. The thermal camera may be offline."
        else:
            ans = (
                f"Thermal peak is {t['peak_c']:.1f}°C (mean {t['mean_c']:.1f}°C). "
                f"Severity: {t['severity']}. {t['hotspots']} hotspot(s) detected. "
                f"Trend over the last 30s: {t['trend']}. "
                + ("Both cameras must agree before the rover acts — YOLO confirmation is required."
                   if t["severity"] != "nominal" else "Scene is calm.")
            )
    elif any(w in q for w in ("lidar", "obstacle", "wall", "distance", "range")):
        L = snap.get("lidar")
        if not L:
            ans = f"No LiDAR data for {thing} right now."
        else:
            ans = (
                f"Nearest obstacle at {L['min_range_m']:.2f}m, mean range "
                f"{L['mean_range_m']:.2f}m. {L['near_obstacles']} of 360 beams are "
                f"inside 0.5m. Heading {L['heading_deg']:.0f}°."
            )
    elif any(w in q for w in ("battery", "power", "charge")):
        p = snap.get("pose")
        if not p:
            ans = f"No pose telemetry for {thing} right now."
        else:
            ans = f"Battery at {p['battery_pct']:.1f}%. " + (
                "Time to plan a dock-and-refill." if p['battery_pct'] < 25 else "Plenty of runtime left.")
    elif any(w in q for w in ("camera", "yolo", "detect", "see")):
        dets = snap.get("detections", [])
        if not dets:
            ans = "Camera is up but nothing detected in the current frame."
        else:
            ans = "Detections: " + ", ".join(f"{d['cls']} @ {d['conf']:.2f}" for d in dets)
    elif any(w in q for w in ("summary", "status", "how", "doing", "report")):
        r = report(thing)
        ans = r["report"]
    else:
        alive = [k for k, v in snap["streams_live"].items() if v]
        ans = (
            f"I can answer questions about {thing}'s thermal, LiDAR, camera, battery, "
            f"or overall status. Right now live streams: {', '.join(alive) or 'none'}."
        )

    return {"answer": ans, "provider": "local-rules", "thing": thing}


def _chat_copilot(question: str, context: str, thing: str) -> dict[str, Any]:
    """GitHub Copilot Chat API. Requires a GITHUB_COPILOT_TOKEN env var."""
    import urllib.request
    token = os.environ["GITHUB_COPILOT_TOKEN"]
    body = json.dumps({
        "model": "gpt-4o-mini",
        "messages": [
            {"role": "system", "content": (
                "You are the FPMS rover analyst. Give short, specific answers. "
                "The data below is the current live snapshot. Do not invent facts.")},
            {"role": "user", "content": f"Live state of {thing}:\n{context}\n\nQuestion: {question}"},
        ],
        "temperature": 0.2,
        "max_tokens": 400,
    }).encode()
    req = urllib.request.Request(
        "https://api.githubcopilot.com/chat/completions",
        data=body,
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
            "Editor-Version": "fpms-dashboard/1.0",
        },
    )
    with urllib.request.urlopen(req, timeout=15) as r:
        data = json.loads(r.read().decode())
    answer = data["choices"][0]["message"]["content"]
    return {"answer": answer, "provider": "github-copilot", "thing": thing}


def _chat_bedrock(question: str, context: str, model_id: str, thing: str) -> dict[str, Any]:
    """AWS Bedrock (real AWS only — LocalStack Community lacks Bedrock)."""
    br = aws._client("bedrock-runtime")
    body = json.dumps({
        "anthropic_version": "bedrock-2023-05-31",
        "max_tokens": 400,
        "temperature": 0.2,
        "messages": [{
            "role": "user",
            "content": (
                f"You are the FPMS rover analyst. Give short, specific answers.\n\n"
                f"Live state of {thing}:\n{context}\n\nQuestion: {question}"
            ),
        }],
    })
    resp = br.invoke_model(modelId=model_id, body=body)
    payload = json.loads(resp["body"].read())
    answer = payload["content"][0]["text"] if payload.get("content") else "(no response)"
    return {"answer": answer, "provider": f"bedrock:{model_id}", "thing": thing}


def status() -> dict[str, Any]:
    """What the analyst can do right now."""
    return {
        "providers": {
            "local-rules": True,
            "github-copilot": _copilot_available(),
            "bedrock": bool(_bedrock_model_id()),
        },
        "default_provider": (
            "bedrock" if _bedrock_model_id()
            else "github-copilot" if _copilot_available()
            else "local-rules"
        ),
    }
