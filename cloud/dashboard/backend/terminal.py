"""Interactive terminal sessions over WebSocket.

Two backends:

  SSH        — always available. paramiko connects to a target host (IP +
               credentials) and shuttles bytes both directions.
  Local      — a real PowerShell subprocess on the laptop running the app.
               Gated hard: only served when the WS request's remote address
               is on the local network (loopback / RFC1918 / link-local),
               so a public visitor over Cloudflare Tunnel never gets it.

Wire format is trivial: browser sends UTF-8 strings (keystrokes), server
sends UTF-8 strings (shell output). No JSON, no framing — xterm.js writes
whatever bytes we send it.

Special control messages the browser CAN send are JSON strings starting with
'\\x1b\\x00' — used for terminal resize. Anything else is treated as input.
"""
from __future__ import annotations

import asyncio
import json
import logging
import shutil
import subprocess
import sys
from typing import Any

from fastapi import WebSocket, WebSocketDisconnect

log = logging.getLogger("fpms.terminal")

# Best-effort import — pywinpty is optional; if missing we fall back to a
# non-PTY subprocess (fine for basic PowerShell interaction, less nice for
# TUI apps).
try:
    from winpty import PtyProcess  # type: ignore
    HAS_PTY = True
except Exception:  # noqa: BLE001
    HAS_PTY = False


# ---- SSH session -----------------------------------------------------------

async def run_ssh_session(
    ws: WebSocket,
    host: str,
    port: int,
    username: str,
    password: str | None,
    key_path: str | None,
) -> None:
    import paramiko  # imported lazily so cold-start of the app stays fast

    loop = asyncio.get_running_loop()

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        await loop.run_in_executor(
            None,
            lambda: client.connect(
                hostname=host, port=port, username=username,
                password=password if password else None,
                key_filename=key_path if key_path else None,
                timeout=6, banner_timeout=6, auth_timeout=6,
                look_for_keys=False, allow_agent=False,
            ),
        )
    except Exception as e:  # noqa: BLE001
        await ws.send_text(f"\r\n\x1b[31mSSH connect failed: {e}\x1b[0m\r\n")
        return

    channel = client.invoke_shell(term="xterm-256color", width=120, height=32)
    channel.settimeout(0.0)
    await ws.send_text(f"\r\n\x1b[32m✓ connected to {username}@{host}\x1b[0m\r\n")

    async def read_from_ssh() -> None:
        try:
            while True:
                if channel.recv_ready():
                    data = channel.recv(65536).decode("utf-8", errors="replace")
                    await ws.send_text(data)
                elif channel.exit_status_ready():
                    break
                else:
                    await asyncio.sleep(0.03)
        except Exception:  # noqa: BLE001
            pass

    async def read_from_ws() -> None:
        try:
            while True:
                msg = await ws.receive_text()
                ctrl = _parse_control(msg)
                if ctrl and ctrl.get("kind") == "resize":
                    try:
                        channel.resize_pty(width=int(ctrl["cols"]), height=int(ctrl["rows"]))
                    except Exception:  # noqa: BLE001
                        pass
                else:
                    channel.send(msg.encode("utf-8"))
        except WebSocketDisconnect:
            pass
        except Exception:  # noqa: BLE001
            pass

    reader = asyncio.create_task(read_from_ssh())
    writer = asyncio.create_task(read_from_ws())
    done, pending = await asyncio.wait({reader, writer}, return_when=asyncio.FIRST_COMPLETED)
    for t in pending:
        t.cancel()
    try:
        channel.close()
        client.close()
    except Exception:  # noqa: BLE001
        pass


# ---- Local shell session ---------------------------------------------------

async def run_local_shell(ws: WebSocket) -> None:
    """Spawn a local PowerShell. LAN-only — caller must gate this."""
    if sys.platform != "win32":
        await ws.send_text("\r\n\x1b[31mLocal shell only supported on Windows.\x1b[0m\r\n")
        return

    exe = shutil.which("pwsh") or shutil.which("powershell") or r"C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe"

    if HAS_PTY:
        await _run_pty_local(ws, exe)
    else:
        await _run_pipe_local(ws, exe)


async def _run_pty_local(ws: WebSocket, exe: str) -> None:
    proc = PtyProcess.spawn([exe, "-NoLogo"], dimensions=(32, 120))
    await ws.send_text("\r\n\x1b[33m⚠ local PowerShell — commands run as you on this laptop.\x1b[0m\r\n")

    async def pump_out() -> None:
        loop = asyncio.get_running_loop()
        try:
            while proc.isalive():
                data = await loop.run_in_executor(None, lambda: proc.read(4096))
                if not data:
                    break
                await ws.send_text(data if isinstance(data, str) else data.decode("utf-8", "replace"))
        except Exception:  # noqa: BLE001
            pass

    async def pump_in() -> None:
        try:
            while True:
                msg = await ws.receive_text()
                ctrl = _parse_control(msg)
                if ctrl and ctrl.get("kind") == "resize":
                    try:
                        proc.setwinsize(int(ctrl["rows"]), int(ctrl["cols"]))
                    except Exception:  # noqa: BLE001
                        pass
                else:
                    proc.write(msg)
        except WebSocketDisconnect:
            pass
        except Exception:  # noqa: BLE001
            pass

    o = asyncio.create_task(pump_out())
    i = asyncio.create_task(pump_in())
    _, pending = await asyncio.wait({o, i}, return_when=asyncio.FIRST_COMPLETED)
    for t in pending:
        t.cancel()
    try:
        proc.terminate(force=True)
    except Exception:  # noqa: BLE001
        pass


async def _run_pipe_local(ws: WebSocket, exe: str) -> None:
    """Fallback without a PTY. Line-buffered but real: ipconfig, arp, get-process,
    all work — you just don't get fancy TUIs (no vim, no htop-like widgets)."""
    # Drop -NonInteractive; that flag refuses stdin. We want stdin.
    proc = await asyncio.create_subprocess_exec(
        exe, "-NoLogo",
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        creationflags=(subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0),
    )
    await ws.send_text(
        "\r\n\x1b[33m⚠ Local PowerShell (line-buffered fallback — no PTY).\r\n"
        "  Real commands: ipconfig, arp -a, Get-Process, Get-Service, netstat -ano, etc.\r\n"
        "  For a full PTY (Vim, ncurses), install pywinpty in the runtime environment.\x1b[0m\r\n\r\n"
    )

    # Line buffer so we send input to PowerShell only on Enter.
    input_buf = ""

    async def pump_out() -> None:
        assert proc.stdout is not None
        try:
            while True:
                chunk = await proc.stdout.read(4096)
                if not chunk:
                    break
                await ws.send_text(chunk.decode("utf-8", "replace"))
        except Exception:  # noqa: BLE001
            pass

    async def pump_in() -> None:
        nonlocal input_buf
        assert proc.stdin is not None
        try:
            while True:
                msg = await ws.receive_text()
                if _parse_control(msg):
                    continue
                for ch in msg:
                    # Local echo + line-editing (no PTY, so we do it ourselves)
                    if ch in ("\r", "\n"):
                        await ws.send_text("\r\n")
                        proc.stdin.write((input_buf + "\n").encode("utf-8"))
                        await proc.stdin.drain()
                        input_buf = ""
                    elif ch in ("\x7f", "\b"):  # backspace
                        if input_buf:
                            input_buf = input_buf[:-1]
                            await ws.send_text("\b \b")
                    elif ch == "\x03":  # Ctrl+C
                        input_buf = ""
                        await ws.send_text("^C\r\n")
                        try:
                            proc.send_signal(subprocess.signal.SIGINT)  # type: ignore[attr-defined]
                        except Exception:  # noqa: BLE001
                            pass
                    else:
                        input_buf += ch
                        await ws.send_text(ch)
        except WebSocketDisconnect:
            pass
        except Exception:  # noqa: BLE001
            pass

    o = asyncio.create_task(pump_out())
    i = asyncio.create_task(pump_in())
    _, pending = await asyncio.wait({o, i}, return_when=asyncio.FIRST_COMPLETED)
    for t in pending:
        t.cancel()
    try:
        proc.terminate()
    except Exception:  # noqa: BLE001
        pass


def _parse_control(msg: str) -> dict[str, Any] | None:
    if not msg.startswith("\x1b\x00"):
        return None
    try:
        return json.loads(msg[2:])
    except Exception:  # noqa: BLE001
        return None
