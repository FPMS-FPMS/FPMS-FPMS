"""HQ email alerts — fires when a rover publishes an alert-class event.

Two providers, pick whichever the user can actually get:

  SMTP (Gmail App Password, Outlook, any relay) — works with any email host
       that accepts SMTP AUTH. Requires an App Password for Gmail.
  Resend (https://resend.com) — one API key, no App Password games, free tier
       generous. Best fallback when Gmail App Passwords are blocked (Family
       Link accounts, Workspace policy, etc).

Runtime state (cooldown + recent-alerts log) is in-memory; restarting the
app clears it. Credentials persist in ~/.fpms/settings.json.
"""
from __future__ import annotations

import json
import logging
import os
import smtplib
import ssl
import threading
import time
import urllib.request
from collections import deque
from email.message import EmailMessage
from typing import Any

from . import settings_store

log = logging.getLogger("fpms.alerts")

DEFAULT_TO = "aryan0419wadhawan@gmail.com"

_lock = threading.Lock()
_last_sent_at: dict[str, float] = {}     # dedupe key -> last send timestamp
_recent: deque[dict[str, Any]] = deque(maxlen=25)


def _cfg() -> dict[str, Any]:
    """Effective config for whichever provider is selected."""
    saved_smtp = settings_store.get("smtp", {}) or {}
    saved_resend = settings_store.get("resend", {}) or {}
    provider = (
        os.environ.get("FPMS_EMAIL_PROVIDER")
        or settings_store.get("email_provider")
        or ("resend" if saved_resend.get("api_key") else "smtp")
    )
    return {
        "provider": provider,
        # SMTP
        "smtp_host": os.environ.get("FPMS_SMTP_HOST") or saved_smtp.get("host") or "smtp.gmail.com",
        "smtp_port": int(os.environ.get("FPMS_SMTP_PORT") or saved_smtp.get("port") or 587),
        "smtp_user": (os.environ.get("FPMS_SMTP_USER") or saved_smtp.get("user") or "").strip(),
        "smtp_pw":   (os.environ.get("FPMS_SMTP_PASS") or saved_smtp.get("pw") or "").strip(),
        # Resend
        "resend_api_key": (os.environ.get("FPMS_RESEND_API_KEY") or saved_resend.get("api_key") or "").strip(),
        "resend_from":    (os.environ.get("FPMS_RESEND_FROM")    or saved_resend.get("from") or "onboarding@resend.dev").strip(),
        # Recipient + shared
        "to": (os.environ.get("FPMS_ALERT_TO") or saved_smtp.get("to") or saved_resend.get("to") or DEFAULT_TO).strip(),
        "cooldown_s": int(os.environ.get("FPMS_ALERT_COOLDOWN_S") or saved_smtp.get("cooldown_s") or saved_resend.get("cooldown_s") or 60),
    }


def save_smtp_credentials(user: str, pw: str, to: str | None = None,
                          host: str | None = None, port: int | None = None) -> dict[str, Any]:
    user = (user or "").strip()
    pw = (pw or "").strip()
    if not user or "@" not in user:
        return {"ok": False, "error": "Gmail address required (e.g. you@gmail.com)"}
    if not pw:
        return {"ok": False, "error": "Google App Password required"}
    smtp = {"user": user, "pw": pw}
    if to:   smtp["to"] = to.strip()
    if host: smtp["host"] = host.strip()
    if port: smtp["port"] = int(port)
    settings_store.set_many({"smtp": smtp, "email_provider": "smtp"})
    return {"ok": True, "provider": "smtp", "user": user}


def save_resend_credentials(api_key: str, to: str | None = None,
                            from_: str | None = None) -> dict[str, Any]:
    api_key = (api_key or "").strip()
    if not api_key.startswith("re_"):
        return {"ok": False,
                "error": "Resend API key required (starts with 're_'). Get one at https://resend.com/api-keys"}
    resend = {"api_key": api_key}
    if to:    resend["to"] = to.strip()
    if from_: resend["from"] = from_.strip()
    settings_store.set_many({"resend": resend, "email_provider": "resend"})
    return {"ok": True, "provider": "resend"}


def clear_credentials() -> dict[str, Any]:
    settings_store.unset("smtp")
    settings_store.unset("resend")
    settings_store.unset("email_provider")
    return {"ok": True}


# Backward-compat alias for the older /api/alerts/signin endpoint.
save_credentials = save_smtp_credentials


def status() -> dict[str, Any]:
    c = _cfg()
    smtp_ok = bool(c["smtp_user"] and c["smtp_pw"])
    resend_ok = bool(c["resend_api_key"])
    configured = (c["provider"] == "smtp" and smtp_ok) or (c["provider"] == "resend" and resend_ok)
    return {
        "configured": configured,
        "provider": c["provider"],
        "providers": {
            "smtp":   {"ready": smtp_ok,   "user": _mask(c["smtp_user"])},
            "resend": {"ready": resend_ok, "from": c["resend_from"]},
        },
        "smtp_host": c["smtp_host"],
        "smtp_port": c["smtp_port"],
        "smtp_user": _mask(c["smtp_user"]),
        "alert_to": c["to"],
        "cooldown_s": c["cooldown_s"],
        "settings_path": settings_store.path(),
        "recent": list(_recent),
    }


def _mask(email: str) -> str:
    if "@" not in email:
        return email
    name, dom = email.split("@", 1)
    if len(name) <= 2:
        return f"{name[:1]}*@{dom}"
    return f"{name[:2]}***@{dom}"


def send(subject: str, body: str, *, to: str | None = None,
         dedupe_key: str | None = None) -> dict[str, Any]:
    """Send one email via the active provider."""
    c = _cfg()
    to = (to or c["to"]).strip()

    if dedupe_key:
        now = time.time()
        with _lock:
            last = _last_sent_at.get(dedupe_key)
        if last and now - last < c["cooldown_s"]:
            return _record({
                "ok": False,
                "skipped": True,
                "reason": f"dedupe: same alert sent {int(now - last)}s ago",
                "to": to,
                "subject": subject,
            })

    if c["provider"] == "resend":
        result = _send_resend(subject, body, to, c)
    else:
        result = _send_smtp(subject, body, to, c)

    if result.get("ok") and dedupe_key:
        with _lock:
            _last_sent_at[dedupe_key] = time.time()
    return _record({**result, "subject": subject, "to": to})


def _send_smtp(subject: str, body: str, to: str, c: dict[str, Any]) -> dict[str, Any]:
    if not c["smtp_user"] or not c["smtp_pw"]:
        return {"ok": False, "provider": "smtp",
                "error": "SMTP not signed in — click 'Sign in with Gmail' in the app."}
    frm = os.environ.get("FPMS_ALERT_FROM", c["smtp_user"]).strip() or c["smtp_user"]
    msg = EmailMessage()
    msg["From"] = frm
    msg["To"] = to
    msg["Subject"] = subject
    msg.set_content(body)
    try:
        ctx = ssl.create_default_context()
        with smtplib.SMTP(c["smtp_host"], c["smtp_port"], timeout=15) as s:
            s.ehlo()
            s.starttls(context=ctx)
            s.ehlo()
            s.login(c["smtp_user"], c["smtp_pw"])
            s.send_message(msg)
    except Exception as e:  # noqa: BLE001
        log.exception("SMTP send failed")
        return {"ok": False, "provider": "smtp", "error": f"SMTP send failed: {e}"}
    return {"ok": True, "provider": "smtp", "from": frm, "sent_at": time.time()}


def _send_resend(subject: str, body: str, to: str, c: dict[str, Any]) -> dict[str, Any]:
    if not c["resend_api_key"]:
        return {"ok": False, "provider": "resend",
                "error": "Resend not signed in — paste your Resend API key in the app."}
    payload = json.dumps({
        "from": c["resend_from"],
        "to": [to],
        "subject": subject,
        "text": body,
    }).encode()
    req = urllib.request.Request(
        "https://api.resend.com/emails",
        data=payload,
        headers={
            "Authorization": f"Bearer {c['resend_api_key']}",
            "Content-Type": "application/json",
            "User-Agent": "FPMS-Dashboard/1.0 (+https://github.com/FPMS-FPMS)",
            "Accept": "application/json",
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            data = json.loads(r.read().decode())
    except urllib.error.HTTPError as e:  # noqa: BLE001
        detail = e.read().decode(errors="replace")[:400]
        log.warning("Resend HTTP error: %s %s", e.code, detail)
        return {"ok": False, "provider": "resend",
                "error": f"Resend {e.code}: {detail}"}
    except Exception as e:  # noqa: BLE001
        log.exception("Resend send failed")
        return {"ok": False, "provider": "resend", "error": f"Resend send failed: {e}"}
    return {"ok": True, "provider": "resend",
            "from": c["resend_from"], "message_id": data.get("id"), "sent_at": time.time()}


def _record(entry: dict[str, Any]) -> dict[str, Any]:
    with _lock:
        _recent.append({**entry, "ts": time.time()})
    return entry


# ---- MQTT hook -----------------------------------------------------------

ALERT_SUBTYPES = {"fire-detected", "alert", "critical", "sos"}


def on_event(thing: str, subtype: str, payload: dict[str, Any]) -> None:
    """Called from mqtt_bridge for each fpms/<thing>/events/<subtype>."""
    if subtype not in ALERT_SUBTYPES and "alert" not in subtype.lower() and "fire" not in subtype.lower():
        return
    severity = str(payload.get("severity", "high")).lower()
    loc = payload.get("location") or {}
    lat, lon = loc.get("lat"), loc.get("lon")
    place = f"{lat:.4f}, {lon:.4f}" if isinstance(lat, (int, float)) and isinstance(lon, (int, float)) else "unknown"

    subject = f"🔥 FIRE IN YOUR AREA — {thing} ({severity.upper()})"
    body = (
        "FPMS emergency alert.\n\n"
        f"Rover:       {thing}\n"
        f"Event:       {subtype}\n"
        f"Severity:    {severity}\n"
        f"Location:    {place}\n"
        f"Event ID:    {payload.get('event_id', '?')}\n"
        f"Timestamp:   {payload.get('timestamp', '?')}\n\n"
        "This alert was auto-generated by the FPMS HQ backend when the rover\n"
        "published an event on fpms/<thing>/events/<subtype>. Cross-check with\n"
        "the live dashboard and, if confirmed, take appropriate action.\n\n"
        "— FPMS Robotics Operations Console\n"
    )
    # Dedupe on (thing, subtype, event_id) so a retry storm doesn't spam.
    key = f"{thing}:{subtype}:{payload.get('event_id', 'no-id')}"
    send(subject, body, dedupe_key=key)


def send_test(to: str | None = None) -> dict[str, Any]:
    """Fire a probe email to verify SMTP + inbox reach end-to-end."""
    subject = "✅ FPMS test email — dashboard is reachable"
    body = (
        "This is a test email from the FPMS Robotics Operations Console.\n\n"
        "If you're reading this, the HQ backend can reach your inbox and\n"
        "future fire/alert events from the Orange Pi rovers will land here\n"
        "automatically.\n\n"
        "— FPMS Robotics Operations Console\n"
    )
    return send(subject, body, to=to)
