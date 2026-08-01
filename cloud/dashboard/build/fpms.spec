import os
# PyInstaller spec for FPMS Dashboard — single-file Windows app.
# Build: pyinstaller build/fpms.spec  (from cloud/dashboard/)

from pathlib import Path

ROOT = Path.cwd()

block_cipher = None

from PyInstaller.utils.hooks import collect_dynamic_libs, collect_data_files, collect_submodules

_winpty_bins = collect_dynamic_libs('winpty')
_winpty_data = collect_data_files('winpty', include_py_files=False)

# pywebview is the critical piece — earlier builds omitted its JS injection
# files and platform submodules, so the WebView2 window died silently.
_pywv_data = collect_data_files('webview', include_py_files=False)
_pywv_subs = collect_submodules('webview')

# clr_loader (pythonnet) — pywebview's edgechromium backend uses it to bridge
# to WinForms. It has C-extension binaries that need bundling.
_clr_bins = collect_dynamic_libs('clr_loader')
_clr_data = collect_data_files('clr_loader', include_py_files=False)

a = Analysis(
    [str(ROOT / 'build' / 'launcher.py')],
    pathex=[str(ROOT)],
    binaries=_winpty_bins + _clr_bins,
    datas=[
        (str(ROOT / 'frontend' / 'dist'), 'frontend/dist'),
    ] + _winpty_data + _pywv_data + _clr_data,
    hiddenimports=_pywv_subs + [
        'backend',
        'backend.main',
        'backend.desktop',
        'backend.mqtt_bridge',
        'backend.hub',
        'backend.thermal_analysis',
        'backend.aws',
        'backend.discovery',
        'backend.config',
        'backend.iot_registry',
        'backend.auth',
        'backend.terminal',
        'backend.analyst',
        'backend.downloads',
        'backend.network',
        'backend.settings_store',
        'backend.email_alerts',
        'winpty',
        'qrcode',
        'PIL',
        'webview.platforms.edgechromium',
        'webview.platforms.winforms',
        'clr_loader',
        'clr_loader.hostfxr',
        'clr_loader.mono',
        'clr_loader.util.find',
        'uvicorn.logging',
        'uvicorn.loops',
        'uvicorn.loops.auto',
        'uvicorn.protocols',
        'uvicorn.protocols.http',
        'uvicorn.protocols.http.auto',
        'uvicorn.protocols.websockets',
        'uvicorn.protocols.websockets.auto',
        'uvicorn.lifespan',
        'uvicorn.lifespan.on',
        'webview.platforms.edgechromium',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['tkinter', 'matplotlib', 'pytest'],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

# ONE-FOLDER, not one-file, and the difference is not cosmetic.
#
# One-file mode packs all ~57 MB of binaries and data INTO the exe and unpacks
# them to a fresh %TEMP%\_MEIxxxxx directory on every single launch. That is
# what shipped, and the frozen app hung on startup because of it: the launcher
# logged "starting embedded uvicorn", the backend module logged the frontend
# path, and then nothing happened for the entire timeout while Windows churned
# through the extracted tree. The identical code path completes in 31 ms
# unfrozen. Nothing in the app was wrong; the packaging was.
#
# One-folder keeps the payload beside the exe, so startup is a load rather than
# an unpack-then-load, and it is also the shape a real installed Windows
# application has. `exclude_binaries=True` on EXE plus a COLLECT below is what
# switches modes -- both halves are required.
#
# UPX is off deliberately. Compressed executables are slower to start, and
# antivirus heuristics treat UPX-packed binaries as suspicious, which on this
# machine means the app is scanned harder every launch for no benefit.
exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='FPMS-Dashboard',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=(os.environ.get('FPMS_BUILD_CONSOLE')=='1'),  # windowed app, no cmd window
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=str(ROOT / 'build' / 'fpms.ico'),
    version=str(ROOT / 'build' / 'version_info.txt'),
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name='FPMS-Dashboard',
)
