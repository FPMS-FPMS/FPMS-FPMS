"""Shared-password gate.

Behaviour:
  - If FPMS_PASSWORD is empty, everything is open (LAN-only default).
  - If FPMS_PASSWORD is set, every request must carry one of:
      • Cookie `fpms_auth=<token>`         (set by /api/login, used by browsers)
      • Header `Authorization: Bearer <token>`  (used by API clients)
    Unauthenticated requests to protected paths get 401.

Public paths (no auth):
  /api/login, /api/auth-status, /favicon.svg, /manifest.webmanifest,
  /sw.js, /pwa-*, /assets/*, and the SPA shell / (so the login page renders).

Tokens are HMAC-signed with a per-run secret; nothing hits disk.
"""
from __future__ import annotations

import hmac
import hashlib
import ipaddress
import logging
import os
import secrets
import time
from typing import Callable

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

log = logging.getLogger("fpms.auth")

def _load_signing_secret() -> bytes:
    """Signing key for session cookies, persisted across restarts.

    This used to be `secrets.token_bytes(32)` at import time, which meant every
    restart silently invalidated every session. The cookie advertises 30 days,
    but with autostart at boot, at logon, a 30-minute health trigger and a
    crash-restart loop, real lifetime was often minutes — so the stated 30 days
    was never what you got.

    Stored in ~/.fpms/settings.json alongside the other local secrets. To force
    every session to log out, delete `session_secret` from that file.
    """
    from . import settings_store

    existing = settings_store.get("session_secret")
    if isinstance(existing, str) and len(existing) >= 64:
        try:
            return bytes.fromhex(existing)
        except ValueError:
            log.warning("stored session_secret is not valid hex; regenerating")

    fresh = secrets.token_bytes(32)
    settings_store.set_many({"session_secret": fresh.hex()})
    log.info("generated a new session signing secret")
    return fresh


_SIGNING_SECRET = _load_signing_secret()
_COOKIE = "fpms_auth"
_TOKEN_TTL_S = 30 * 24 * 3600  # 30 days

# Paths that never require auth (login flow + PWA shell resources).
_PUBLIC_PREFIXES = (
    "/api/login",
    "/api/auth-status",
    "/favicon",
    "/manifest",
    "/sw.js",
    "/pwa-",
    "/assets/",
)


def _configured_password() -> str:
    return os.environ.get("FPMS_PASSWORD", "").strip()


def make_token() -> str:
    exp = int(time.time()) + _TOKEN_TTL_S
    payload = f"{exp}".encode()
    sig = hmac.new(_SIGNING_SECRET, payload, hashlib.sha256).hexdigest()
    return f"{exp}.{sig}"


def valid_token(tok: str) -> bool:
    try:
        exp_s, sig = tok.split(".", 1)
        exp = int(exp_s)
        if exp < time.time():
            return False
        expected = hmac.new(_SIGNING_SECRET, exp_s.encode(), hashlib.sha256).hexdigest()
        return hmac.compare_digest(sig, expected)
    except Exception:  # noqa: BLE001
        return False


def is_lan(client_host: str | None) -> bool:
    """Is the caller on the local network (or the same laptop)?"""
    if not client_host:
        return False
    try:
        ip = ipaddress.ip_address(client_host)
    except ValueError:
        return False
    return (
        ip.is_loopback
        or ip.is_private
        or ip.is_link_local
    )


# Hostnames that only ever appear when a request arrived from the internet:
# the Cloudflare quick tunnel, and the permanent gateway Worker in front of it.
_REMOTE_HOST_SUFFIXES = (".trycloudflare.com", ".workers.dev")

# Routes that can act on this laptop or its network, as opposed to just showing
# rover data. Blocked for remote visitors when safe mode is on.
_PRIVILEGED_PREFIXES = (
    "/ws/term/",                    # live PowerShell + SSH terminals
    "/api/discovery/scan",          # LAN sweep from this machine
    "/api/discovery/ssh",           # SSH into LAN hosts
    "/api/discovery/provision",     # push scripts onto devices
    "/api/discovery/interfaces",    # enumerates this machine's networks
    "/api/network/firewall",        # changes Windows Firewall
    "/api/network/gateway",         # holds the gateway secret
    "/api/network/tunnel",          # a visitor could otherwise kill the link
    "/api/terminal",
)


def _hosts_are_remote(forwarded: str, host: str) -> bool:
    return any(
        candidate.split(":")[0].lower().endswith(_REMOTE_HOST_SUFFIXES)
        for candidate in (forwarded, host)
        if candidate
    )


def request_is_remote(request: Request) -> bool:
    """True when the request came in over the public tunnel.

    Client IP can't answer this: cloudflared connects to the app over loopback,
    so a visitor from the other side of the world looks identical to the local
    window. The Host header is what differs — the browser asked for the tunnel
    or workers.dev hostname, and that value survives the hop.
    """
    return _hosts_are_remote(
        request.headers.get("x-forwarded-host") or "",
        request.headers.get("host") or "",
    )


def is_privileged_path(path: str) -> bool:
    return path.startswith(_PRIVILEGED_PREFIXES)


class PublicSafeModeMiddleware:
    """Deny machine-control routes to visitors arriving over the tunnel.

    Deliberately raw ASGI rather than BaseHTTPMiddleware: Starlette only runs
    BaseHTTPMiddleware for http scopes, and the terminals are WebSocket routes —
    an http-only guard would leave the most dangerous surface wide open.
    """

    def __init__(self, app) -> None:
        self.app = app

    async def __call__(self, scope, receive, send) -> None:
        if scope["type"] in ("http", "websocket"):
            from .config import settings
            if is_privileged_path(scope.get("path", "")):
                # The cloud app refuses machine control outright. It runs on a
                # custom domain, so there's no tunnel hostname to detect — and a
                # Host header can be forged anyway. Role is the trustworthy
                # signal because it comes from this process's own environment.
                blocked = settings.is_cloud
                reason = "cloud role"

                if not blocked and settings.public_safe_mode:
                    headers = {
                        k.decode("latin-1").lower(): v.decode("latin-1")
                        for k, v in scope.get("headers", [])
                    }
                    blocked = _hosts_are_remote(headers.get("x-forwarded-host", ""),
                                                headers.get("host", ""))
                    reason = "public visitor"

                if blocked:
                    log.warning("safe mode blocked %s (%s)", scope.get("path"), reason)
                    if scope["type"] == "websocket":
                        # 1008 = policy violation.
                        await send({"type": "websocket.close", "code": 1008})
                        return
                    response = JSONResponse(
                        {
                            "error": "unavailable_remotely",
                            "reason": "This control is disabled over the public link. "
                                      "Use the dashboard on the HQ laptop.",
                        },
                        status_code=403,
                    )
                    await response(scope, receive, send)
                    return
        await self.app(scope, receive, send)


def _is_public_path(path: str) -> bool:
    if any(path.startswith(p) for p in _PUBLIC_PREFIXES):
        return True
    # SPA shell (/, /devices, /aws, /lidar, ...) — the React app renders the
    # login page for unauthenticated users itself. Only /api/* + /ws/* are
    # actually gated at the HTTP layer.
    return not (path.startswith("/api/") or path.startswith("/ws/"))


class PasswordAuthMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        password = _configured_password()

        # No password configured → wide open (LAN-default mode).
        if not password:
            return await call_next(request)

        if _is_public_path(request.url.path):
            return await call_next(request)

        token = (
            request.cookies.get(_COOKIE)
            or _bearer_from_header(request.headers.get("Authorization"))
        )
        if token and valid_token(token):
            return await call_next(request)

        return JSONResponse(
            {"error": "unauthorized", "reason": "password required"},
            status_code=401,
        )


def _bearer_from_header(h: str | None) -> str | None:
    if not h:
        return None
    parts = h.split(" ", 1)
    if len(parts) == 2 and parts[0].lower() == "bearer":
        return parts[1].strip()
    return None


def issue_cookie(response: Response, token: str) -> None:
    response.set_cookie(
        _COOKIE, token,
        max_age=_TOKEN_TTL_S,
        httponly=True,
        samesite="lax",
        secure=False,  # allow HTTP over LAN; Cloudflare Tunnel terminates TLS anyway
        path="/",
    )


def clear_cookie(response: Response) -> None:
    response.delete_cookie(_COOKIE, path="/")


def auth_enabled() -> bool:
    return bool(_configured_password())


def check_password(supplied: str) -> bool:
    actual = _configured_password()
    if not actual:
        return True
    return hmac.compare_digest(supplied.encode(), actual.encode())
