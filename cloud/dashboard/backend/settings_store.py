"""Persistent per-user settings for the FPMS Dashboard.

Stored at ~/.fpms/settings.json — user-scoped, survives app restarts, doesn't
leak into git. On Windows this lands in C:\\Users\\<you>\\.fpms\\.

Values are stored in plaintext. This is a single-user laptop app; anyone with
read access to your home directory can already see your credentials via
countless other means. If you need at-rest encryption, wrap the value in
DPAPI (win32crypt.CryptProtectData) — not enabled by default because it
adds a Windows-only pywin32 dep.
"""
from __future__ import annotations

import json
import os
import stat
import threading
from pathlib import Path
from typing import Any

_lock = threading.Lock()
_PATH = Path(os.environ.get("FPMS_SETTINGS_PATH")
             or (Path.home() / ".fpms" / "settings.json"))


def _load_raw() -> dict[str, Any]:
    if not _PATH.is_file():
        return {}
    try:
        return json.loads(_PATH.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return {}


def get(key: str, default: Any = None) -> Any:
    with _lock:
        return _load_raw().get(key, default)


def set_many(updates: dict[str, Any]) -> None:
    with _lock:
        data = _load_raw()
        data.update(updates)
        _PATH.parent.mkdir(parents=True, exist_ok=True)
        _PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")
        # Best-effort: restrict to owner on POSIX; Windows inherits ACL from parent.
        try:
            _PATH.chmod(stat.S_IRUSR | stat.S_IWUSR)
        except Exception:  # noqa: BLE001
            pass


def unset(key: str) -> None:
    with _lock:
        data = _load_raw()
        if key in data:
            data.pop(key)
            _PATH.parent.mkdir(parents=True, exist_ok=True)
            _PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")


def path() -> str:
    return str(_PATH)
