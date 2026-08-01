"""Frozen-app entry point.

When bundled by PyInstaller, sys.path is set up so that `backend` is
importable directly (see fpms.spec — the whole `backend/` package is added
as a hidden import and the `frontend/dist/` folder is copied in as data).

This launcher:
  1. Sets a Windows AppUserModelID so the taskbar treats us as a distinct app
     (that's why pinning a raw .bat currently shows "python" — no AUMID).
  2. Points pywebview at the bundled frontend/dist by rewriting the
     FRONTEND_DIST env var if we detect we're frozen.
  3. Delegates to backend.desktop.main().
"""
from __future__ import annotations

import os
import sys
from pathlib import Path


def _set_aumid() -> None:
    """Give Windows a stable App User Model ID so taskbar-pinning works.

    Without this, Windows groups the app under python.exe. With it, the
    taskbar shows FPMS as its own pinnable entry with our icon.
    """
    if sys.platform != "win32":
        return
    try:
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(
            "com.fpms.robotics.dashboard"
        )
    except Exception:  # noqa: BLE001
        pass


def _bundled_frontend_dir() -> Path | None:
    """When frozen, PyInstaller extracts data to sys._MEIPASS."""
    meipass = getattr(sys, "_MEIPASS", None)
    if not meipass:
        return None
    p = Path(meipass) / "frontend" / "dist"
    return p if p.is_dir() else None


def _repair_std_streams() -> None:
    """Give the windowed build real stdout/stderr objects.

    THIS IS WHY THE PACKAGED APP WOULD NOT START, and the failure was perfectly
    self-concealing.

    A PyInstaller build with console=False links against the runw.exe
    bootloader, which starts the process with NO console attached. Python then
    sets `sys.stdout` and `sys.stderr` to None. Anything that writes to them
    raises AttributeError — including logging's StreamHandler, which resolves
    `sys.stderr` lazily at emit time, and uvicorn's startup logging.

    So the app died during startup, and the mechanism that would have reported
    why was the very thing that was broken: the log file showed a normal boot
    up to the last line written before the first stderr write, then nothing.
    Building the identical tree with console=True made it work, which is what
    finally identified it.

    Pointing the streams at os.devnull is enough. The file log is configured
    separately in backend/desktop.py and keeps working, so nothing diagnostic
    is lost — only the writes that had nowhere to go anyway.
    """
    import io
    import os

    for name in ("stdout", "stderr"):
        stream = getattr(sys, name, None)
        if stream is not None:
            continue
        try:
            handle = open(os.devnull, "w", encoding="utf-8")
        except Exception:  # noqa: BLE001 - last resort, must never raise here
            handle = io.StringIO()
        setattr(sys, name, handle)
        # __stdout__/__stderr__ are None too, and library code reaches for them
        # when it wants "the real one" — notably logging.shutdown and several
        # traceback paths.
        if getattr(sys, "__%s__" % name, None) is None:
            setattr(sys, "__%s__" % name, handle)


def main() -> None:
    # ORDER MATTERS. Streams first: everything below this line, including the
    # imports, may log.
    _repair_std_streams()

    # MUST be the first thing that runs in a frozen build, before any import
    # that might spawn a process. PyInstaller's child processes re-execute this
    # very exe from the top, and without freeze_support() they re-run main()
    # instead of the worker payload — which on Windows means a second copy of
    # the app racing the first for the same port, and neither one explaining
    # itself. Harmless and free when not frozen.
    import multiprocessing
    multiprocessing.freeze_support()

    _set_aumid()

    # Point the backend at the UI frozen into this exe — but only if the
    # operator has not already chosen one. An explicit FPMS_FRONTEND_DIST is a
    # deliberate act ("serve this newer build without reinstalling"), and
    # clobbering it here made the override silently do nothing: the app kept
    # serving whatever UI it was built with while the environment said
    # otherwise, which is a confusing failure to diagnose from the outside.
    # backend/main.py already falls back to the bundled copy if the override
    # path does not exist, so honouring it here cannot leave the app with no UI.
    override = os.environ.get("FPMS_FRONTEND_DIST")
    if override and Path(override).is_dir():
        pass  # operator's choice wins
    else:
        if override:
            # Never silent: an override that points nowhere means the operator
            # believes they are looking at a build they are not.
            print(
                f"[FPMS] FPMS_FRONTEND_DIST={override!r} is not a directory — "
                f"ignoring it and using the UI bundled into this exe.",
                flush=True,
            )
        dist = _bundled_frontend_dir()
        if dist:
            os.environ["FPMS_FRONTEND_DIST"] = str(dist)
        elif override:
            # Not frozen and the override is bad: drop it so backend/main.py
            # falls back to frontend/dist instead of re-deciding on garbage.
            os.environ.pop("FPMS_FRONTEND_DIST", None)

    # Import late so the AUMID and env are set before FastAPI/webview spin up.
    from backend.desktop import main as run
    run()


if __name__ == "__main__":
    main()
