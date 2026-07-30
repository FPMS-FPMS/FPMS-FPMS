"""LAN & public reachability — one source of truth.

Answers three questions the UI needs to show to make sharing painless:
  1. What URLs is this app reachable at? (loopback + every LAN IP)
  2. Is Windows Firewall letting inbound TCP 8000 through?
  3. Is a Cloudflare Tunnel currently running, and if so what's its URL?
"""
from __future__ import annotations

import asyncio
import io
import json
import logging
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import psutil

from . import settings_store
from .config import settings

log = logging.getLogger("fpms.network")

_FIREWALL_RULE_NAME = "FPMS Dashboard Inbound 8000"


# ---- reachable URLs --------------------------------------------------------

_VIRTUAL_INTERFACE_HINTS = (
    "vethernet", "vmware", "virtualbox", "hyper-v", "loopback",
    "docker", "wsl", "bluetooth", "npcap",
)


def _is_virtual(name: str) -> bool:
    n = name.lower()
    return any(h in n for h in _VIRTUAL_INTERFACE_HINTS)


def reachable_urls() -> list[dict[str, Any]]:
    """Return only URLs that other devices on the same WiFi can actually
    reach. Filters out WSL, Hyper-V, VMware, Bluetooth PAN, loopback
    virtual adapters — those look like private IPs but aren't routable
    from other machines."""
    port = settings.bind_port
    out: list[dict[str, Any]] = [{
        "kind": "loopback",
        "url": f"http://localhost:{port}",
        "description": "This laptop only",
        "recommended": False,
    }]

    stats = psutil.net_if_stats()
    seen: set[str] = set()
    real_lan_first: list[dict[str, Any]] = []

    for name, addrs in psutil.net_if_addrs().items():
        st = stats.get(name)
        if not st or not st.isup:
            continue
        if _is_virtual(name):
            continue
        for a in addrs:
            if a.family != socket.AF_INET:
                continue
            ip = a.address
            if ip.startswith("127.") or ip in seen:
                continue
            if ip.startswith("169.254"):
                continue  # APIPA / link-local — not useful for sharing
            seen.add(ip)
            real_lan_first.append({
                "kind": "lan",
                "url": f"http://{ip}:{port}",
                "description": f"{name} — reachable from any device on the same WiFi/network",
                "interface": name,
                "ip": ip,
                "recommended": True,   # marked; UI shows first one prominently
            })

    # Prefer Wi-Fi over Ethernet in the ordering.
    real_lan_first.sort(key=lambda u: (0 if "wi-fi" in u["interface"].lower() else 1, u["ip"]))
    if real_lan_first:
        real_lan_first[0]["recommended"] = True
    return out + real_lan_first


# ---- Windows Firewall ------------------------------------------------------

def firewall_status() -> dict[str, Any]:
    """Is inbound TCP `bind_port` allowed?"""
    if sys.platform != "win32":
        return {"platform": sys.platform, "supported": False,
                "allowed": True, "rule_present": False,
                "note": "Firewall auto-manage is only implemented on Windows."}
    try:
        out = subprocess.run(
            ["netsh", "advfirewall", "firewall", "show", "rule",
             f"name={_FIREWALL_RULE_NAME}"],
            capture_output=True, text=True, timeout=5,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )
    except Exception as e:  # noqa: BLE001
        return {"platform": "win32", "supported": True, "allowed": None,
                "rule_present": False, "error": str(e)}
    text = out.stdout + out.stderr
    present = "No rules match" not in text and (out.returncode == 0)
    # A more thorough check: the rule references our port.
    port_ok = str(settings.bind_port) in text if present else False
    return {
        "platform": "win32",
        "supported": True,
        "rule_name": _FIREWALL_RULE_NAME,
        "rule_present": present and port_ok,
        "allowed": present and port_ok,
        "port": settings.bind_port,
    }


def firewall_add_command() -> str:
    """The PowerShell one-liner that adds the rule.

    Suitable for a UAC-elevated Start-Process; also shown in the UI so a
    savvy user can paste it into an Admin PowerShell if they prefer."""
    return (
        f'netsh advfirewall firewall add rule name="{_FIREWALL_RULE_NAME}" '
        f'dir=in action=allow protocol=TCP localport={settings.bind_port} profile=any'
    )


def firewall_add_elevated() -> dict[str, Any]:
    """Trigger a UAC prompt and (if approved) add the rule.

    We can't add the rule from this non-elevated process, but we CAN ask
    Windows to relaunch a tiny privileged helper via Start-Process -Verb RunAs.
    User sees the UAC dialog. If they click Yes, the rule lands.
    """
    if sys.platform != "win32":
        return {"ok": False, "error": "not supported on this platform"}
    cmd = firewall_add_command()
    # Use ShellExecuteW with "runas" verb — this pops UAC.
    ps_script = (
        f'Start-Process -Verb RunAs -Wait -WindowStyle Hidden -FilePath cmd.exe '
        f'-ArgumentList \'/c\', \'{cmd}\''
    )
    try:
        r = subprocess.run(
            ["powershell.exe", "-NoProfile", "-Command", ps_script],
            capture_output=True, text=True, timeout=60,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": str(e), "manual_command": cmd}
    # After UAC returns, re-check.
    status = firewall_status()
    return {
        "ok": bool(status.get("allowed")),
        "elevation_exit_code": r.returncode,
        "manual_command": cmd,
        "status": status,
    }


# ---- Cloudflare Tunnel -----------------------------------------------------

@dataclass
class TunnelState:
    proc: subprocess.Popen | None = None
    url: str | None = None
    started_at: float | None = None
    output: list[str] = field(default_factory=list)  # tail of stdout/stderr
    gateway_published: bool = False   # is the permanent URL pointing at us?
    gateway_error: str | None = None  # last publish failure, for the UI


_TUNNEL_LOCK = threading.Lock()
_TUNNEL: TunnelState = TunnelState()


# ---- permanent-URL gateway -------------------------------------------------
#
# Quick-tunnel hostnames rotate on every restart, which is why a shared link
# eventually dies. The gateway is a tiny Cloudflare Worker on a stable
# workers.dev address (see ../gateway/worker.js) that redirects to whichever
# tunnel is current. We push our URL to it on startup, then re-push on a
# heartbeat so the Worker can tell "rotated" apart from "HQ is gone".

_GATEWAY_HEARTBEAT_SECONDS = 300  # 5 min → ~288 KV writes/day, under the 1000 free
_GATEWAY_TIMEOUT_SECONDS = 10


def gateway_config() -> dict[str, Any]:
    """Read the persisted gateway settings from ~/.fpms/settings.json."""
    url = str(settings_store.get("gateway_url") or "").strip().rstrip("/")
    secret = str(settings_store.get("gateway_secret") or "").strip()
    return {"url": url or None, "secret": secret or None}


def _gateway_post(path: str, payload: dict[str, Any] | None) -> tuple[bool, str | None]:
    """POST to the Worker. Best-effort: never raises, returns (ok, error)."""
    cfg = gateway_config()
    if not cfg["url"] or not cfg["secret"]:
        return False, "gateway not configured"

    body = json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(
        cfg["url"] + path,
        data=body,
        method="POST",
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {cfg['secret']}",
            # Cloudflare's bot check answers the default "Python-urllib/3.x"
            # signature with a 403 (error 1010), so identify ourselves properly.
            "User-Agent": "FPMS-Dashboard/1.0 (+gateway-register)",
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=_GATEWAY_TIMEOUT_SECONDS) as r:
            if 200 <= r.status < 300:
                return True, None
            return False, f"gateway returned HTTP {r.status}"
    except urllib.error.HTTPError as e:
        detail = ""
        try:
            detail = e.read().decode("utf-8", "replace")[:200]
        except Exception:  # noqa: BLE001
            pass
        return False, f"gateway HTTP {e.code}: {detail}" if detail else f"gateway HTTP {e.code}"
    except Exception as e:  # noqa: BLE001
        return False, str(e)


def _publish_to_gateway(url: str) -> None:
    """Point the permanent URL at `url`. Failures are logged, never fatal —
    a broken gateway must not take down a tunnel that's otherwise working."""
    ok, err = _gateway_post("/_register", {"url": url})
    with _TUNNEL_LOCK:
        _TUNNEL.gateway_published = ok
        _TUNNEL.gateway_error = err
    if ok:
        log.info("gateway now points at %s", url)
    elif err != "gateway not configured":
        log.warning("could not publish tunnel URL to gateway: %s", err)


def _offline_gateway() -> None:
    """Tell the Worker we're going down, so visitors get the offline page
    immediately instead of waiting out the staleness window."""
    ok, err = _gateway_post("/_offline", None)
    with _TUNNEL_LOCK:
        _TUNNEL.gateway_published = False
        _TUNNEL.gateway_error = None if ok else err
    if not ok and err != "gateway not configured":
        log.warning("could not mark gateway offline: %s", err)


def _gateway_heartbeat(proc: subprocess.Popen) -> None:
    """Re-register the same URL periodically so the Worker knows HQ is alive.
    Exits as soon as this tunnel process does."""
    while proc.poll() is None:
        time.sleep(_GATEWAY_HEARTBEAT_SECONDS)
        if proc.poll() is not None:
            break
        with _TUNNEL_LOCK:
            url = _TUNNEL.url
            same_proc = _TUNNEL.proc is proc
        # A newer tunnel took over; that one owns the heartbeat now.
        if not same_proc:
            break
        if url:
            _publish_to_gateway(url)


def _cloudflared_path() -> str | None:
    """Locate cloudflared.exe. Search order:
    1. PATH
    2. bin/ next to the running .exe (frozen bundle)
    3. bin/ in the source tree (dev)
    """
    p = shutil.which("cloudflared")
    if p:
        return p
    candidates: list[Path] = []
    if getattr(sys, "frozen", False):
        exe_dir = Path(sys.executable).resolve().parent
        # Installed layout: bin/ sits beside the exe.
        #   C:\Program Files (x86)\FPMS Dashboard\{FPMS-Dashboard.exe, bin\}
        candidates.append(exe_dir / "bin" / "cloudflared.exe")
        # Dev layout: the exe lives in dist/, so bin/ is one level up.
        #   cloud\dashboard\{dist\FPMS-Dashboard.exe, bin\}
        candidates.append(exe_dir.parent / "bin" / "cloudflared.exe")
    candidates.append(Path(__file__).resolve().parent.parent / "bin" / "cloudflared.exe")
    for c in candidates:
        if c.is_file():
            return str(c)
    return None


def primary_lan_ip() -> str | None:
    """This laptop's LAN address as other devices on the WiFi see it.

    That's what a rover must point its MQTT publisher at — "localhost" on the Pi
    means the Pi itself, so the address has to be resolved here and baked into
    the provisioning script.
    """
    lan = [u for u in reachable_urls() if u.get("kind") == "lan" and u.get("ip")]
    if not lan:
        return None
    preferred = next((u for u in lan if u.get("recommended")), lan[0])
    return preferred.get("ip")


def tunnel_status() -> dict[str, Any]:
    cfg = gateway_config()
    with _TUNNEL_LOCK:
        alive = _TUNNEL.proc is not None and _TUNNEL.proc.poll() is None
        return {
            "running": alive,
            "url": _TUNNEL.url,
            "started_at": _TUNNEL.started_at,
            "password_set": bool(os.environ.get("FPMS_PASSWORD", "").strip()),
            "cloudflared_available": _cloudflared_path() is not None,
            "recent_output": list(_TUNNEL.output[-30:]),
            # The permanent URL people actually bookmark.
            "gateway_url": cfg["url"],
            "gateway_configured": bool(cfg["url"] and cfg["secret"]),
            "gateway_published": _TUNNEL.gateway_published,
            "gateway_error": _TUNNEL.gateway_error,
        }


_QUICK_URL_RE = re.compile(r"https?://[a-z0-9-]+\.trycloudflare\.com", re.IGNORECASE)

# Keep well above the 30 lines the UI shows, but bounded.
_TUNNEL_OUTPUT_LIMIT = 200


def _watch_tunnel_output(proc: subprocess.Popen) -> None:
    """Tail stdout for the quick-tunnel URL and record recent lines."""
    assert proc.stdout is not None
    for raw in iter(proc.stdout.readline, ""):
        line = raw.rstrip()
        found: str | None = None
        with _TUNNEL_LOCK:
            _TUNNEL.output.append(line)
            # Only the tail is ever shown, and this process is expected to run
            # for months — an unbounded list would grow for the life of the
            # tunnel and never be read.
            if len(_TUNNEL.output) > _TUNNEL_OUTPUT_LIMIT:
                del _TUNNEL.output[:-_TUNNEL_OUTPUT_LIMIT]
            if not _TUNNEL.url:
                m = _QUICK_URL_RE.search(line)
                if m:
                    _TUNNEL.url = m.group(0)
                    found = _TUNNEL.url
                    log.info("cloudflare tunnel URL: %s", _TUNNEL.url)
        # Publish outside the lock: _publish_to_gateway acquires it too, and
        # threading.Lock is not reentrant.
        if found:
            _publish_to_gateway(found)


def start_tunnel() -> dict[str, Any]:
    with _TUNNEL_LOCK:
        if _TUNNEL.proc and _TUNNEL.proc.poll() is None:
            return {"ok": True, "already_running": True, **tunnel_status()}

    if not os.environ.get("FPMS_PASSWORD", "").strip():
        return {
            "ok": False,
            "error": "Refusing to publish without FPMS_PASSWORD set.",
            "hint": "Set FPMS_PASSWORD before starting the app, then try again.",
        }

    cfd = _cloudflared_path()
    if not cfd:
        return {
            "ok": False,
            "error": "cloudflared not found on PATH or in bundled bin/. Install from "
                     "https://github.com/cloudflare/cloudflared/releases",
        }

    port = settings.bind_port
    try:
        proc = subprocess.Popen(
            [cfd, "tunnel", "--url", f"http://localhost:{port}", "--no-autoupdate"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
            creationflags=(subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0),
        )
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": str(e)}

    with _TUNNEL_LOCK:
        _TUNNEL.proc = proc
        _TUNNEL.url = None
        _TUNNEL.started_at = time.time()
        _TUNNEL.output = []
        _TUNNEL.gateway_published = False
        _TUNNEL.gateway_error = None

    threading.Thread(target=_watch_tunnel_output, args=(proc,), daemon=True).start()
    threading.Thread(target=_gateway_heartbeat, args=(proc,), daemon=True).start()

    # Wait a couple of seconds for the URL to appear so the UI has it immediately.
    for _ in range(60):
        if tunnel_status()["url"] or proc.poll() is not None:
            break
        time.sleep(0.25)

    return {"ok": True, **tunnel_status()}


_SUPERVISOR_STARTED = threading.Event()
_SUPERVISOR_POLL_SECONDS = 15

# Whether the tunnel is *supposed* to be up. Lets the supervisor tell a crash
# (restart it) from the user clicking "Stop public URL" (leave it down).
_TUNNEL_WANTED = False


def _tunnel_supervisor() -> None:
    """Restart the tunnel whenever cloudflared exits.

    The .bat restart loop only covers the app process. Without this, a crashed
    cloudflared leaves a perfectly healthy dashboard that nobody outside the LAN
    can reach, and the permanent URL serves the offline page until a human
    notices — which historically meant finding out via a phone that couldn't
    load it.
    """
    consecutive_failures = 0
    while True:
        time.sleep(_SUPERVISOR_POLL_SECONDS)
        try:
            if not _TUNNEL_WANTED or tunnel_status()["running"]:
                consecutive_failures = 0
                continue
            log.warning("tunnel is down — restarting it")
            result = start_tunnel()
            if result.get("ok"):
                log.info("tunnel restarted: %s", result.get("url"))
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                log.error("tunnel restart failed: %s", result.get("error"))
                # Back off instead of retrying every 15s forever. A missing
                # binary or bad config will not fix itself, and on a machine
                # meant to run for months the retry log alone becomes the
                # problem. Caps at ~30 min between attempts.
                if consecutive_failures >= 3:
                    backoff = min(_SUPERVISOR_POLL_SECONDS * 2 ** (consecutive_failures - 2), 1800)
                    log.warning("tunnel failed %d times; next attempt in %ds",
                                consecutive_failures, int(backoff))
                    time.sleep(backoff)
        except Exception:  # noqa: BLE001
            log.exception("tunnel supervisor iteration failed")


def start_tunnel_supervised() -> dict[str, Any]:
    """start_tunnel(), plus a watchdog that keeps it up for the process's life."""
    global _TUNNEL_WANTED
    _TUNNEL_WANTED = True
    result = start_tunnel()
    if not _SUPERVISOR_STARTED.is_set():
        _SUPERVISOR_STARTED.set()
        threading.Thread(target=_tunnel_supervisor, daemon=True,
                         name="fpms-tunnel-supervisor").start()
    return result


def stop_tunnel() -> dict[str, Any]:
    # Deliberate stop — tell the supervisor to leave it down.
    global _TUNNEL_WANTED
    _TUNNEL_WANTED = False
    with _TUNNEL_LOCK:
        p = _TUNNEL.proc
        _TUNNEL.proc = None
        url_was = _TUNNEL.url
        _TUNNEL.url = None
        _TUNNEL.started_at = None
    if p and p.poll() is None:
        try:
            p.terminate()
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        except Exception:  # noqa: BLE001
            pass
    # Deliberate shutdown → show the offline page now rather than redirecting
    # visitors to a hostname that no longer answers.
    _offline_gateway()
    return {"ok": True, "was_url": url_was}


# ---- QR code ---------------------------------------------------------------

def qr_png(data: str) -> bytes:
    """Small QR PNG for a URL. Uses `qrcode` if available (falls back to
    Google Charts if not — same visual output)."""
    try:
        import qrcode  # type: ignore
    except ImportError:
        # Fallback: no dep, but requires internet. Better than nothing.
        import urllib.request
        url = f"https://chart.googleapis.com/chart?chs=320x320&cht=qr&chl={data}"
        return urllib.request.urlopen(url, timeout=5).read()

    img = qrcode.make(data, box_size=8, border=2)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()
