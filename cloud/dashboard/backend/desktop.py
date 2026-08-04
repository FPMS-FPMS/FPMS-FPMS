"""Native desktop launcher.

Two modes:

  windowed (default) — starts uvicorn + opens a native WebView2 window.
                       If the window fails to open (e.g. no WebView2, blocked
                       by antivirus, missing display), falls back to
                       *headless* mode instead of exiting — the server keeps
                       serving so the tunnel URL and LAN URL keep working.

  headless (FPMS_HEADLESS=1)  — start uvicorn only, no window. Runs until Ctrl+C
                                or the process is killed.

Also writes a launch log to %LOCALAPPDATA%\\FPMS\\launch.log so a silent crash
is never invisible.
"""
from __future__ import annotations

import atexit
import logging
import os
from logging.handlers import RotatingFileHandler
import socket
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import uvicorn

from .config import settings


def _log_path() -> Path:
    base = os.environ.get("LOCALAPPDATA") or str(Path.home())
    d = Path(base) / "FPMS"
    d.mkdir(parents=True, exist_ok=True)
    return d / "launch.log"


def _session_marker_path() -> Path:
    return _log_path().with_name("session.running")


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(),
        # Rotating, not plain FileHandler: this machine is meant to run for
        # months with an every-30-minutes health trigger, and a log that only
        # ever grows is a slow disk leak nobody notices until it matters.
        # 5 MB x 3 backups caps the whole thing at 20 MB.
        RotatingFileHandler(_log_path(), maxBytes=5 * 1024 * 1024,
                            backupCount=3, encoding="utf-8"),
    ],
)
log = logging.getLogger("fpms.desktop")


def _claim_session() -> None:
    """Record that this process is running, and report how the last one ended.

    Every exit path in this file logs its reason, so a launch.log that shows a
    start with no matching reason means the process was killed from outside
    (Task Manager, a taskkill, a reboot) or died at the C level without ever
    reaching Python — a WebView2 fault, for instance.

    That distinction previously had to be reconstructed by hand from the shape
    of the log, and "it silently exited" was unanswerable because a clean quit
    and an external kill left the same evidence: nothing. The marker turns it
    into one line at the next start.
    """
    marker = _session_marker_path()
    try:
        if marker.is_file():
            stale = marker.read_text(encoding="utf-8").strip()
            log.warning(
                "PREVIOUS SESSION DID NOT SHUT DOWN CLEANLY — it left no exit "
                "reason in this log. That means it was killed from outside "
                "(Task Manager/taskkill/reboot/power loss) or crashed below "
                "Python. Previous session: %s",
                stale or "<unrecorded>",
            )
        marker.write_text(
            f"pid={os.getpid()} started={time.strftime('%Y-%m-%d %H:%M:%S')}",
            encoding="utf-8",
        )
    except OSError:
        # Diagnostics must never be able to stop the app from starting.
        log.debug("could not write session marker", exc_info=True)


def _release_session(reason: str) -> None:
    """Log why we are exiting and clear the crash marker."""
    log.info("EXITING: %s", reason)
    try:
        _session_marker_path().unlink(missing_ok=True)
    except OSError:
        log.debug("could not clear session marker", exc_info=True)


def _server_alive() -> bool:
    """True if an FPMS backend is already answering on our port.

    Uses /api/auth-status because it's the one endpoint that's always public
    (needs no cookie even when FPMS_PASSWORD is set). Any HTTP response =
    the server is alive, regardless of status code.
    """
    try:
        req = urllib.request.Request(
            f"http://127.0.0.1:{settings.bind_port}/api/auth-status",
            headers={"User-Agent": "fpms-desktop-probe"},
        )
        with urllib.request.urlopen(req, timeout=1) as r:
            return r.status < 500
    except urllib.error.HTTPError:
        # Non-2xx counts as alive — the server is answering, it just doesn't
        # like this request. That's enough for us to attach a window to it.
        return True
    except (urllib.error.URLError, TimeoutError, ConnectionError, OSError):
        return False


def _port_free(host: str, port: int) -> bool:
    """True if nothing is listening on `port`."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        try:
            s.connect((host, port))
            return False
        except OSError:
            return True


def _wait_for_server(timeout_s: float = 25) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if _server_alive():
            return True
        time.sleep(0.25)
    return False



def _start_url() -> str:
    """Window URL, optionally deep-linked to a page.

    FPMS_START_PATH=/camera opens straight on the camera view. Driving the app
    by simulated Tab presses was unreliable — focus order shifts with the layout
    and it kept landing on the wrong tab.
    """
    path = (os.environ.get("FPMS_START_PATH") or "/").strip()
    if not path.startswith("/"):
        path = "/" + path
    return f"http://127.0.0.1:{settings.bind_port}{path}"

def _open_window() -> bool:
    """Open the app as an embedded native window.

    Uses pywebview → WebView2 (Edge's rendering engine hosted INSIDE this
    process — Task Manager sees FPMS-Dashboard.exe, not msedge.exe). No
    browser dependency at runtime beyond the WebView2 runtime, which ships
    with Windows 10 21H2+ and Windows 11 by default.
    """
    try:
        import webview
    except Exception as e:  # noqa: BLE001
        log.warning("pywebview unavailable (%s) — falling back to --app browser mode", e)
        return _open_window_browser_fallback()

    url = _start_url()
    started = time.time()
    try:
        webview.create_window(
            title="FPMS · Robotics Operations Console",
            url=url,
            width=1400,
            height=900,
            min_size=(1024, 700),
            background_color="#07090d",
            confirm_close=False,
        )
        webview.start(gui="edgechromium")
    except Exception as e:  # noqa: BLE001
        log.error("webview crashed: %s — falling back to --app browser mode", e)
        return _open_window_browser_fallback()

    dur = time.time() - started
    if dur < 2:
        log.warning("webview returned in %.1fs — likely failed, falling back to --app browser", dur)
        return _open_window_browser_fallback()
    log.info("webview session ended cleanly after %.1fs", dur)
    return True


def _open_window_browser_fallback() -> bool:
    """Fallback: launch Edge/Chrome in --app mode. Used if pywebview breaks."""
    import shutil as _shutil, subprocess as _subprocess
    candidates: list[tuple[str, str]] = [
        (r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe", "Edge"),
        (r"C:\Program Files\Microsoft\Edge\Application\msedge.exe", "Edge"),
        (r"C:\Program Files\Google\Chrome\Application\chrome.exe", "Chrome"),
        (r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe", "Chrome"),
    ]
    exe = next((p for p, _ in candidates if os.path.isfile(p)),
               _shutil.which("msedge") or _shutil.which("chrome"))
    if not exe:
        log.error("no browser found for fallback")
        return False
    profile = Path(os.environ.get("LOCALAPPDATA") or str(Path.home())) / "FPMS" / "browser-profile"
    profile.mkdir(parents=True, exist_ok=True)
    args = [exe, f"--app={_start_url()}",
            f"--user-data-dir={profile}", "--no-first-run", "--no-default-browser-check",
            "--window-size=1400,900"]
    log.info("launching browser fallback: %s", exe)
    started = time.time()
    try:
        p = _subprocess.Popen(args)
        p.wait()
    except Exception as e:  # noqa: BLE001
        log.error("browser fallback failed: %s", e)
        return False
    return time.time() - started >= 2


def _start_uvicorn() -> tuple[uvicorn.Server, threading.Thread]:
    # Importing the app runs every module-level side effect in the backend, so
    # a bad import surfaces here. Log the traceback before dying — otherwise the
    # launcher exits between "starting embedded uvicorn" and "ready" with no
    # explanation, which is exactly how a cold-start failure looked in the wild.
    try:
        from .main import app
    except Exception:
        log.exception("failed to import the FPMS app — server cannot start")
        raise

    # Bind to 0.0.0.0 so LAN devices (phones, other laptops) can reach us.
    # The webview window uses 127.0.0.1 to talk to us; both work.
    config = uvicorn.Config(
        app,
        host=settings.bind_host,       # default 0.0.0.0
        port=settings.bind_port,
        log_level="warning",
        access_log=False,
        timeout_graceful_shutdown=3,
    )
    server = uvicorn.Server(config)

    def _run() -> None:
        # A daemon thread that raises dies silently — Python prints the
        # traceback to a stderr nobody is reading when we're windowed/frozen.
        #
        # "Nobody is reading" is literal in the frozen build: PyInstaller links
        # it against the runw.exe bootloader, which has NO CONSOLE AT ALL. Every
        # print(), every stderr traceback, every uvicorn startup banner is
        # written to a handle that goes nowhere. The log file is the only
        # channel that survives, so anything worth diagnosing has to go through
        # `log`, and the exit has to be recorded even when it is clean —
        # otherwise a server that stops on its own is indistinguishable from one
        # that is running fine, which is exactly how this looked in the wild:
        # "starting embedded uvicorn on :8010" and then silence forever.
        try:
            server.run()
        except BaseException:  # noqa: BLE001 - SystemExit/KeyboardInterrupt too
            log.exception("uvicorn terminated with an exception")
        finally:
            log.info("uvicorn thread exited (started=%s, should_exit=%s)",
                     getattr(server, "started", None),
                     getattr(server, "should_exit", None))

    thread = threading.Thread(target=_run, daemon=True, name="fpms-uvicorn")
    thread.start()
    return server, thread


def _run_forever() -> None:
    """Block the calling thread until the process is killed."""
    log.info("running headless — leave this window/process open to keep the "
             "backend + tunnel URL alive. Ctrl+C or close to stop.")
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        log.info("Ctrl+C received")


def _maybe_autostart_tunnel() -> None:
    """Raise the public tunnel as part of launch when FPMS_AUTO_TUNNEL is set.

    This process has to be the one that starts it. start_tunnel() is what parses
    the fresh *.trycloudflare.com hostname and registers it with the permanent
    gateway URL, then heartbeats to prove HQ is still alive. A cloudflared
    launched outside the app publishes nothing, so the permanent link would go on
    pointing at a tunnel that no longer exists — which is the exact failure this
    whole gateway exists to prevent.
    """
    if os.environ.get("FPMS_AUTO_TUNNEL", "").strip().lower() not in ("1", "true", "yes"):
        return
    try:
        from . import network
        result = network.start_tunnel_supervised()
        if result.get("ok"):
            # Registration happens on the watcher thread a moment later and logs
            # itself ("gateway now points at ..."), so don't report it from here
            # — reading the flag now races and reads a stale False.
            log.info("public tunnel up at %s", result.get("url"))
        else:
            log.error("could not start public tunnel: %s", result.get("error"))
    except Exception:
        # Never let this take down an otherwise healthy local dashboard.
        log.exception("tunnel autostart failed")


def main() -> None:
    headless_env = os.environ.get("FPMS_HEADLESS", "").strip().lower() in ("1", "true", "yes")
    headless_arg = any(a.lower() in ("--headless", "-headless", "/headless") for a in sys.argv[1:])
    headless = headless_env or headless_arg

    log.info("FPMS launcher starting — headless=%s, log=%s", headless, _log_path())
    _claim_session()

    # A. If another FPMS is already up, just attach a window (or run headless).
    if _server_alive():
        log.info("existing FPMS backend detected on :%d — attaching only",
                 settings.bind_port)
        if not headless:
            ran = _open_window()
            if not ran:
                # Window failed but there's already a server owned by someone
                # else — nothing more we can do. Exit cleanly.
                log.info("window unavailable and server owned externally — exiting")
        _release_session("attached to an FPMS backend owned by another process; "
                         "that server keeps running, only this window is gone")
        return

    # B. Port in TIME_WAIT from a previous run?
    for wait_s in (0, 0.5, 1.0, 2.0, 3.0):
        if _port_free("127.0.0.1", settings.bind_port):
            break
        log.info("port %d busy — waiting %ss for TIME_WAIT to clear",
                 settings.bind_port, wait_s)
        time.sleep(wait_s)
    else:
        log.error("port %d held by another process; exiting", settings.bind_port)
        _release_session(f"port {settings.bind_port} is held by another process")
        raise SystemExit(
            f"Port {settings.bind_port} is held by another process. "
            f"Close any previous FPMS instance and try again."
        )

    # C. Fresh boot.
    log.info("starting embedded uvicorn on :%d", settings.bind_port)
    server, thread = _start_uvicorn()

    def _shutdown() -> None:
        server.should_exit = True

    atexit.register(_shutdown)

    # Each step below gets its own line, because in the frozen build the log
    # file is the ONLY output channel (runw.exe bootloader, no console) and a
    # silent gap between two log lines is unattributable — the app could be
    # waiting on the port, blocked raising the tunnel, or already dead, and all
    # three look identical from outside. They are separated here so the next
    # person reads a location instead of guessing one.
    log.info("waiting for the server to accept connections on %s:%d",
             settings.bind_host, settings.bind_port)
    if not _wait_for_server(timeout_s=25):
        _shutdown()
        log.error("backend never came up on %s:%d within 25s — the uvicorn "
                  "thread either failed to bind or is still starting. Check "
                  "for an 'uvicorn terminated' line above.",
                  settings.bind_host, settings.bind_port)
        _release_session("the backend never accepted connections within 25s")
        raise SystemExit(f"backend never came up on port {settings.bind_port}")
    log.info("server is accepting connections on %s:%d",
             settings.bind_host, settings.bind_port)

    _maybe_autostart_tunnel()
    log.info("tunnel step complete; headless=%s", headless)

    if headless:
        _run_forever()
        _shutdown()
        thread.join(timeout=5)
        _release_session("headless run was interrupted (Ctrl+C or process stop)")
        return

    log.info("opening native window")
    window_ran = _open_window()
    if window_ran:
        # The single most common "the app silently exited" report is this line:
        # closing the window closes the whole application, backend included, and
        # nothing on screen says so. Naming it here is what makes the log
        # answer the question instead of just ending.
        log.info("the native window was closed — the backend shuts down with it, "
                 "so :%d is no longer served. Relaunch with "
                 "Start-FPMS-Dashboard.cmd to bring it back.", settings.bind_port)
    if not window_ran:
        # WebView failed. Rather than kill the server the user is relying on
        # (Cloudflare Tunnel + LAN peers), keep it alive headless. Also open
        # the default browser as a fallback so the user always sees something.
        log.warning("window did not open — falling back to default browser + headless server")
        try:
            import webbrowser
            webbrowser.open(f"http://localhost:{settings.bind_port}/")
        except Exception:  # noqa: BLE001
            pass
        _run_forever()

    _shutdown()
    thread.join(timeout=5)
    log.info("shut down cleanly")
    _release_session("the operator closed the window" if window_ran else
                     "no window could be opened and the headless run ended")


if __name__ == "__main__":
    main()
