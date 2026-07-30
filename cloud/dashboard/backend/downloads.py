"""Where to find downloadable installers for each platform.

Windows we can serve directly (the .exe lives next to us or as
sys._MEIPASS/../.. in the frozen bundle). macOS / Linux we point at the
source tree — building a native bundle on those platforms needs to happen
on those platforms.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Any


def _windows_installer_path() -> Path | None:
    """Preferred download: proper Windows installer (Setup.exe) — registers
    in Add/Remove Programs, creates Start Menu + taskbar-pinnable shortcut."""
    root = Path(__file__).resolve().parent.parent
    p = root / "build" / "installer-dist" / "FPMS-Dashboard-Setup.exe"
    return p if p.is_file() else None


def _windows_portable_path() -> Path | None:
    """Fallback: raw portable .exe, no installer."""
    if getattr(sys, "frozen", False):
        p = Path(sys.executable)
        if p.exists() and p.name.lower().endswith(".exe"):
            return p
    p = Path(__file__).resolve().parent.parent / "dist" / "FPMS-Dashboard.exe"
    return p if p.is_file() else None


def _windows_exe_path() -> Path | None:
    """Whatever we can serve for Windows — installer preferred, portable as fallback."""
    return _windows_installer_path() or _windows_portable_path()


def catalog(request_host: str) -> dict[str, Any]:
    installer = _windows_installer_path()
    portable = _windows_portable_path()
    primary = installer or portable
    primary_bytes = primary.stat().st_size if primary else None
    return {
        "platforms": [
            {
                "id": "windows",
                "name": "Windows 10 / 11",
                "type": "installer" if installer else "portable-exe",
                "size_bytes": primary_bytes,
                "available": primary is not None,
                "download_url": "/download/windows" if primary else None,
                "download_filename": primary.name if primary else None,
                "install_notes": (
                    [
                        "Click Download — you get FPMS-Dashboard-Setup.exe (a Windows installer).",
                        "Double-click Setup.exe. Windows may show 'unrecognized app' — click More info → Run anyway (unsigned installer, safe).",
                        "Wizard installs to %LOCALAPPDATA%\\Programs\\FPMS Dashboard\\ — no admin required.",
                        "After install: Start Menu → FPMS Dashboard, or right-click the Desktop shortcut → Pin to taskbar.",
                    ] if installer else [
                        "Download the portable .exe.",
                        "Right-click → Properties → Unblock if Windows quarantines it.",
                        "Double-click to launch. For proper install with taskbar-pin support, use the Setup.exe path instead.",
                    ]
                ),
            },
            {
                "id": "macos",
                "name": "macOS 12+ (Intel/Apple Silicon)",
                "type": "pwa-or-source",
                "available": True,
                "download_url": None,
                "install_notes": [
                    f"Open http://{request_host} in Safari or Chrome.",
                    "In Safari: File → Add to Dock (Sonoma+) — installs as a native-feeling app.",
                    "In Chrome/Edge: click the install icon in the URL bar.",
                    "For a signed .app bundle, build from source on macOS: `python -m PyInstaller build/fpms.spec` (needs the platform-specific dependencies).",
                ],
            },
            {
                "id": "linux",
                "name": "Linux (Ubuntu 22.04+, Fedora, Arch)",
                "type": "pwa-or-source",
                "available": True,
                "download_url": None,
                "install_notes": [
                    f"Open http://{request_host} in Chrome or Firefox.",
                    "Install as PWA — Chrome shows an install icon in the URL bar.",
                    "For a bundled binary, build from source: clone repo, `bash scripts/start-backend.sh`.",
                ],
            },
            {
                "id": "ios",
                "name": "iPhone / iPad",
                "type": "pwa",
                "available": True,
                "download_url": None,
                "install_notes": [
                    f"Open http://{request_host} in Safari (must be Safari on iOS).",
                    "Tap the Share button.",
                    "Tap 'Add to Home Screen'.",
                    "Tap Add. The app appears on your Home Screen with the FPMS icon and opens fullscreen.",
                ],
            },
            {
                "id": "android",
                "name": "Android 10+",
                "type": "pwa",
                "available": True,
                "download_url": None,
                "install_notes": [
                    f"Open http://{request_host} in Chrome.",
                    "Chrome shows an 'Install app' banner, or tap ⋮ → 'Install app'.",
                    "The app appears in your launcher and opens fullscreen.",
                ],
            },
        ],
        "server_host": request_host,
        "public_note": (
            "The dashboard is running on this laptop. For anyone anywhere in the world to "
            "reach it, run Publish-Public.bat (Cloudflare Tunnel). See GO-PUBLIC.md."
        ),
    }


def windows_download_path() -> Path | None:
    return _windows_exe_path()
