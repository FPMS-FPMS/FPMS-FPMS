#!/usr/bin/env python3
"""Deploy rover code from this repo to the Orange Pi, without breaking the
two things on this rover that are expensive to recover from.

WHAT THIS SCRIPT IS FOR

  Copy fpms_teleop.py, deadband_sweep.py and fpms_rover_agent.py from
  cloud/dashboard/rover/ (this directory) onto the Pi, restart the services
  that need it, and leave the rover in a state you can trust without having
  to SSH in and look. Runs from a developer machine (this repo's checkout),
  not on the Pi.

THE TWO THINGS THIS SCRIPT MUST NEVER DO

  1. Restart micro-ros-agent. It is not one of the files this script
     deploys, and there is no code path here that touches it, but the
     restart helper below still refuses by name (FORBIDDEN_SERVICES) as a
     second line of defense. Bouncing that service costs a 90-225s board
     reconnect on this hardware (Yahboom MicroROS Board V2.0 / ESP32-S3) and
     has cost hours of debugging time when it happened by accident. If you
     ever add a fourth deployed file that maps to micro-ros-agent, that is a
     bug in this script, not a reason to remove the guard.
  2. Overwrite fpms_teleop.py (or restart its service) while the rover might
     still be mid-command. fpms-teleop owns the only writer to /cmd_vel; if
     systemd kills the running process out from under an in-flight command,
     nothing guarantees the motors saw a stop first. So before that file is
     touched, this script runs `systemctl stop fpms-teleop` — whose SIGTERM
     shutdown path (see fpms-teleop.service: KillSignal=SIGTERM,
     TimeoutStopSec=15) already publishes zero Twist x10 before exiting —
     and then polls `systemctl is-active` until it actually reports
     inactive. If it does not go inactive in time, this script aborts
     before touching the file. It does not guess.

OTHER SAFETY PROPERTIES (see the numbered requirements this fulfils)

  * Every remote file this script is about to overwrite is backed up first,
    to "<path>.bak-<timestamp>" on the Pi, and the backup path is printed —
    that is the rollback plan, in full, always available.
  * Nothing is installed straight from this laptop onto the live path.
    Each file is uploaded via SFTP to a staging directory on the Pi first
    (/home/ubuntu/.fpms_deploy_stage/), then moved into place with `sudo
    cp`. This also sidesteps /usr/local/bin not being SFTP-writable by the
    ubuntu user.
  * After a file lands on its live path, it is syntax-checked ON THE PI
    (ast.parse over the actual bytes that will run) before any service is
    restarted. A parse failure restores that file's backup immediately and
    aborts the whole run — no service is ever restarted against code that
    doesn't even parse.
  * If fpms-teleop was stopped to protect an in-flight command and the
    overall deploy then fails for an unrelated reason (a different file
    fails its syntax check), this script still restarts fpms-teleop before
    exiting. A failed deploy is a fine reason to stop; leaving the rover
    permanently uncommandable is not an acceptable side effect of that.
  * --dry-run is the DEFAULT. You must pass --live to make this script
    touch the network. --dry-run always wins if both are given.
  * The Pi password is never hardcoded and never printed. It comes from
    FPMS_PI_PASSWORD in the environment, or --password as a fallback: if
    neither is present (and this isn't a dry run) the script refuses to
    guess and exits with a clear message.

CONNECTING TO THE PI

  Host is unstable by design right now (hardware bring-up in progress):
  fpms-pi.local over IPv6 link-local, and it has answered on 192.168.137.181
  and, previously, .201 and .73. This script tries a short list of
  candidates, and for each one tries a normal paramiko connect first (works
  for mDNS names and ordinary IPv4), then falls back to an explicit IPv6
  link-local connect (paramiko's own resolver does not reliably handle IPv6
  link-local scope on Windows, which is where this is usually run from).

USAGE

  python deploy_rover.py                          # dry run, all 3 files
  python deploy_rover.py --only fpms_teleop.py     # dry run, one file
  python deploy_rover.py --live                    # do it, all 3 files
  python deploy_rover.py --live --only deadband_sweep.py
  python deploy_rover.py --live --host 192.168.137.181

  Password: set FPMS_PI_PASSWORD, or pass --password (avoid the latter on a
  shared machine — it lands in shell history).
"""
from __future__ import annotations

import argparse
import os
import shlex
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

# ============================================================== WHAT / WHERE

@dataclass(frozen=True)
class DeployFile:
    local: str        # filename in this directory (cloud/dashboard/rover/)
    remote: str        # absolute destination path on the Pi
    service: str | None  # systemd unit to restart after install, or None
    executable: bool    # chmod +x after install (binaries dropped in PATH)


DEPLOY_FILES: tuple[DeployFile, ...] = (
    DeployFile("fpms_teleop.py", "/home/ubuntu/fpms_teleop.py", "fpms-teleop", False),
    DeployFile("deadband_sweep.py", "/home/ubuntu/deadband_sweep.py", None, False),
    DeployFile("fpms_rover_agent.py", "/usr/local/bin/fpms-rover-agent", "fpms-rover-agent", True),
    # Odometry/TF for Nav2: position from /odom_raw pose deltas, heading from
    # integrated gyro Z. NOT yet installed on the rover — the unit file
    # (fpms-odom-tf.service) still has to be placed and enabled by hand the
    # first time; after that this entry keeps the code current.
    DeployFile("fpms_odom_tf.py", "/home/ubuntu/fpms_odom_tf.py", "fpms-odom-tf", False),
)

# Services that exist on this rover but that this script must NEVER restart,
# no matter what a future edit to DEPLOY_FILES might map to them. Restarting
# micro-ros-agent costs a 90-225s board reconnect — see module docstring.
FORBIDDEN_SERVICES: frozenset[str] = frozenset({"micro-ros-agent"})

# Try these, in order, until one accepts an SSH connection. fpms-pi.local is
# the mDNS name (answers over IPv6 link-local); the rest are IPv4 addresses
# this rover has answered on before, kept as fallbacks because DHCP on this
# network has reassigned it more than once during bring-up.
DEFAULT_HOSTS: tuple[str, ...] = (
    "fpms-pi.local",
    "192.168.137.181",
    "192.168.137.201",
    "192.168.137.73",
)

DEFAULT_USER = "ubuntu"
STAGE_DIR = "/home/ubuntu/.fpms_deploy_stage"

STOP_VERIFY_TIMEOUT_S = 25   # > TimeoutStopSec=15 in fpms-teleop.service, plus margin
RESTART_VERIFY_TIMEOUT_S = 25


# =============================================================== CONNECTION

def connect(hosts: tuple[str, ...], user: str, password: str, timeout: int):
    """Try each candidate host; for each, try a normal connect, then an
    explicit IPv6 link-local connect. Returns (client, host_used) or
    (None, None) if nothing answered."""
    import paramiko  # imported lazily so --dry-run never requires it installed

    attempts: list[str] = []
    for host in hosts:
        print(f"  trying {host} ...")

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(
                hostname=host, username=user, password=password,
                timeout=timeout, banner_timeout=timeout, auth_timeout=timeout,
                allow_agent=False, look_for_keys=False,
            )
            print(f"    connected ({host})")
            return client, host
        except Exception as e_direct:  # noqa: BLE001
            pass

        # Fall back to an explicit IPv6 link-local connect. A plain
        # paramiko.connect() often can't resolve/route a link-local address
        # (needs a scope id) the way this raw-socket dance does.
        try:
            sa = socket.getaddrinfo(host, 22, socket.AF_INET6, socket.SOCK_STREAM)[0][-1]
            sock = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect(sa)

            client_v6 = paramiko.SSHClient()
            client_v6.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client_v6.connect(
                hostname=host, username=user, password=password,
                sock=sock, allow_agent=False, look_for_keys=False,
            )
            print(f"    connected ({host}, IPv6 link-local)")
            return client_v6, host
        except Exception as e_v6:  # noqa: BLE001
            attempts.append(f"{host}: direct={e_direct!r}  ipv6={e_v6!r}")
            continue

    print("  no candidate host answered:")
    for line in attempts:
        print(f"    {line}")
    return None, None


def run(client, cmd: str, timeout: int = 60) -> tuple[int, str, str]:
    """Run one command over the existing SSH connection, block for exit
    status, return (rc, stdout, stderr) as text."""
    _stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    rc = stdout.channel.recv_exit_status()
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    return rc, out, err


# =================================================================== STEPS

def backup_remote_file(client, remote_path: str) -> str | None:
    """Copy remote_path to remote_path.bak-<timestamp> before it gets
    overwritten. Returns the backup path, or None if remote_path did not
    exist yet (nothing to back up — this is a first-time deploy of a new
    file, not an error)."""
    rc, _out, _err = run(client, f"sudo test -f {shlex.quote(remote_path)}")
    if rc != 0:
        return None

    ts = time.strftime("%Y%m%dT%H%M%S")
    backup_path = f"{remote_path}.bak-{ts}"
    rc, _out, err = run(client, f"sudo cp -p {shlex.quote(remote_path)} {shlex.quote(backup_path)}")
    if rc != 0:
        raise RuntimeError(f"backup of {remote_path} failed: {err.strip()}")
    return backup_path


def install_staged_file(client, staged_path: str, remote_path: str, executable: bool) -> None:
    """Move a file that is already sitting in STAGE_DIR into its live path.
    Uses sudo unconditionally: /usr/local/bin needs it, and it is harmless
    for the /home/ubuntu targets (ubuntu has passwordless sudo on this Pi,
    matching the rest of this rover's tooling — see bench_npu.py)."""
    rc, _out, err = run(client, f"sudo cp {shlex.quote(staged_path)} {shlex.quote(remote_path)}")
    if rc != 0:
        raise RuntimeError(f"install of {remote_path} failed: {err.strip()}")
    if executable:
        rc, _out, err = run(client, f"sudo chmod +x {shlex.quote(remote_path)}")
        if rc != 0:
            raise RuntimeError(f"chmod +x {remote_path} failed: {err.strip()}")


def syntax_check_remote(client, remote_path: str) -> tuple[bool, str]:
    """ast.parse the file as it actually sits on the Pi, using the Pi's own
    python3 — never trust that what parsed on the laptop is byte-identical
    to what an SFTP transfer + sudo cp actually produced."""
    py_snippet = f"import ast; ast.parse(open({remote_path!r}, encoding='utf-8').read())"
    cmd = "sudo python3 -c " + shlex.quote(py_snippet)
    rc, out, err = run(client, cmd)
    return rc == 0, (err.strip() or out.strip())


def restore_backup(client, remote_path: str, backup_path: str | None) -> str:
    """Undo a failed install. If there was no prior version (backup_path is
    None), there is nothing to restore TO — the only safe move is to remove
    the broken file rather than leave it live."""
    if backup_path is None:
        run(client, f"sudo rm -f {shlex.quote(remote_path)}")
        return "removed (no prior version existed to restore)"
    rc, _out, err = run(client, f"sudo cp -p {shlex.quote(backup_path)} {shlex.quote(remote_path)}")
    if rc != 0:
        # This is as bad as it gets: the live file is broken AND we could not
        # put the known-good one back. Surface it loudly rather than retry
        # blindly against an unknown filesystem state.
        raise RuntimeError(
            f"CRITICAL: restore of {remote_path} from {backup_path} FAILED: {err.strip()} "
            f"— the Pi may be left with a broken {remote_path}. Fix by hand."
        )
    return f"restored from {backup_path}"


def service_state(client, service: str) -> str:
    rc, out, err = run(client, f"systemctl is-active {shlex.quote(service)}")
    return out.strip() or err.strip() or "unknown"


def stop_and_verify(client, service: str, timeout: int = STOP_VERIFY_TIMEOUT_S) -> tuple[bool, str]:
    """systemctl stop, then poll until systemd actually reports the unit
    inactive. Idempotent: calling this on an already-stopped unit is a
    harmless no-op, so callers never need to check current state first."""
    run(client, f"sudo systemctl stop {shlex.quote(service)}")
    deadline = time.time() + timeout
    state = service_state(client, service)
    while time.time() < deadline:
        state = service_state(client, service)
        if state in ("inactive", "failed"):
            return True, state
        time.sleep(1)
    return False, state


def restart_and_verify(client, service: str, timeout: int = RESTART_VERIFY_TIMEOUT_S):
    """systemctl restart, then poll for 'active'. Returns
    (ok, final_state, journal_tail_or_None). journal_tail is populated only
    when ok is False, so callers get the diagnostic they need without
    fetching logs on the happy path."""
    if service in FORBIDDEN_SERVICES:
        # Second line of defense — see module docstring. DEPLOY_FILES never
        # maps anything to this service, so this should be unreachable; if
        # it ever fires, that is a bug in DEPLOY_FILES, not a false alarm.
        raise RuntimeError(
            f"refusing to restart {service!r}: it is in FORBIDDEN_SERVICES "
            f"(micro-ros-agent reconnects cost 90-225s — never restart it here)"
        )
    run(client, f"sudo systemctl restart {shlex.quote(service)}")
    deadline = time.time() + timeout
    state = service_state(client, service)
    while time.time() < deadline:
        state = service_state(client, service)
        if state == "active":
            return True, state, None
        if state == "failed":
            break
        time.sleep(1)
    _rc, tail, _err = run(client, f"sudo journalctl -u {shlex.quote(service)} -n 40 --no-pager")
    return False, state, tail


# ==================================================================== PLAN

def select_files(only: str | None) -> list[DeployFile]:
    if only is None:
        return list(DEPLOY_FILES)
    key = only[:-3] if only.endswith(".py") else only
    matches = [f for f in DEPLOY_FILES if f.local == only or f.local[:-3] == key]
    if not matches:
        choices = ", ".join(f.local for f in DEPLOY_FILES)
        raise SystemExit(f"--only {only!r} matches nothing. Choices: {choices}")
    return matches


def print_plan(files: list[DeployFile], dry_run: bool) -> None:
    print("=" * 72)
    print("DRY RUN — no connection will be made, nothing will change."
          if dry_run else
          "LIVE RUN — this will connect to the rover and change it.")
    print("=" * 72)
    for f in files:
        local_path = SCRIPT_DIR / f.local
        exists = local_path.is_file()
        size = f"{local_path.stat().st_size} bytes" if exists else "MISSING LOCALLY"
        print(f"  {local_path}  ({size})")
        print(f"    -> {f.remote}")
        print(f"       backup written to {f.remote}.bak-<timestamp> before overwrite (if a prior version exists)")
        if f.executable:
            print("       + chmod +x after install")
        if f.service == "fpms-teleop":
            print(f"       service: fpms-teleop — STOPPED and verified inactive BEFORE this file is touched,")
            print(f"                then restarted after all files pass their syntax check")
        elif f.service:
            print(f"       service: {f.service} — restarted after all files pass their syntax check")
        else:
            print("       service: (none — file only)")
    services = [f.service for f in files if f.service]
    print(f"  services that would be restarted: {', '.join(dict.fromkeys(services)) or '(none)'}")
    print(f"  NEVER restarted by this script, under any flag combination: {', '.join(sorted(FORBIDDEN_SERVICES))}")
    print("=" * 72)


# ==================================================================== MAIN

def get_password(cli_password: str | None) -> str:
    pw = os.environ.get("FPMS_PI_PASSWORD") or cli_password
    if not pw:
        raise SystemExit(
            "No Pi password available. Set FPMS_PI_PASSWORD in the environment, "
            "or pass --password explicitly. Refusing to hardcode or invent one."
        )
    return pw


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Deploy rover code to the Orange Pi. Dry-run by default; pass --live to act.",
    )
    parser.add_argument("--host", action="append", default=None,
                         help="Try this host first (repeatable). Falls back to the usual candidates.")
    parser.add_argument("--user", default=DEFAULT_USER)
    parser.add_argument("--password", default=None,
                         help="Pi password. Prefer FPMS_PI_PASSWORD env var over this (shell history).")
    parser.add_argument("--only", default=None, metavar="NAME",
                         help="Deploy a single file, e.g. --only fpms_teleop.py (or just fpms_teleop).")
    parser.add_argument("--live", action="store_true",
                         help="Actually connect and change the rover. Without this, always a dry run.")
    parser.add_argument("--dry-run", action="store_true",
                         help="Force a dry run even if --live is also given (dry-run always wins).")
    parser.add_argument("--timeout", type=int, default=15, help="SSH connect timeout, seconds.")
    args = parser.parse_args()

    dry_run = args.dry_run or not args.live
    files = select_files(args.only)
    print_plan(files, dry_run)

    if dry_run:
        print("\nDry run complete. Re-run with --live to actually deploy.")
        return 0

    # ---- everything below this line can touch the network -----------------
    password = get_password(args.password)
    hosts = tuple(args.host) if args.host else DEFAULT_HOSTS

    print(f"\nConnecting as {args.user}@<one of {', '.join(hosts)}> ...")
    client, host_used = connect(hosts, args.user, password, args.timeout)
    if client is None:
        print("\nABORT: could not reach the rover. Nothing was changed.")
        return 1

    backups: dict[str, str | None] = {}
    installed: list[DeployFile] = []
    teleop_stopped = False
    failure: str | None = None

    try:
        run(client, f"mkdir -p {shlex.quote(STAGE_DIR)}")

        # ---- upload every file to a staging dir first; this never touches
        # a live path, so it is safe to do before any of the stop/backup
        # dance below and lets us fail fast on a missing local file.
        sftp = client.open_sftp()
        staged_paths: dict[str, str] = {}
        try:
            for f in files:
                local_path = SCRIPT_DIR / f.local
                if not local_path.is_file():
                    raise RuntimeError(f"local file missing: {local_path}")
                staged = f"{STAGE_DIR}/{f.local}"
                print(f"\nUploading {local_path.name} -> {host_used}:{staged}")
                sftp.put(str(local_path), staged)
                staged_paths[f.local] = staged
        finally:
            sftp.close()

        # ---- install loop: stop-before-copy for teleop, backup, copy,
        # chmod, syntax-check. Abort on the first failure — do not pile a
        # second risky operation on top of a file we already know is bad.
        for f in files:
            if f.service == "fpms-teleop" and not teleop_stopped:
                print("\nStopping fpms-teleop and verifying it is inactive before touching its file "
                      "(its shutdown path publishes zero Twist x10 first)...")
                ok, state = stop_and_verify(client, "fpms-teleop")
                teleop_stopped = True  # attempted, regardless of outcome — drives the safety-net below
                if not ok:
                    failure = f"fpms-teleop did not report inactive within {STOP_VERIFY_TIMEOUT_S}s (state={state}); refusing to overwrite its file while it may still be running"
                    break
                print(f"  fpms-teleop is {state}")

            print(f"\nInstalling {f.remote}")
            backup_path = backup_remote_file(client, f.remote)
            backups[f.remote] = backup_path
            print(f"  backup: {backup_path or '(none — file did not exist before)'}")

            install_staged_file(client, staged_paths[f.local], f.remote, f.executable)
            ok, message = syntax_check_remote(client, f.remote)
            if not ok:
                restored = restore_backup(client, f.remote, backup_path)
                failure = f"{f.remote} failed to parse on the Pi ({message}); {restored}"
                break

            print(f"  {f.remote} parses OK on the Pi")
            installed.append(f)

        if failure:
            print(f"\nABORT: {failure}")
            if teleop_stopped:
                # A failed deploy is a fine reason to stop; leaving the rover
                # permanently uncommandable is not an acceptable side effect
                # of that. Bring teleop back on whatever is now on disk for
                # it (either untouched, or just restored from backup above).
                print("Restarting fpms-teleop so the rover is not left uncommandable...")
                ok, state, tail = restart_and_verify(client, "fpms-teleop")
                print(f"  fpms-teleop: {state}")
                if not ok and tail:
                    print("  --- journalctl -u fpms-teleop -n 40 ---")
                    print(tail)
            return 1

        # ---- every installed file parsed OK: now, and only now, restart
        # services. Order follows DEPLOY_FILES / the files given, deduped.
        results: dict[str, tuple[bool, str, str | None]] = {}
        services = list(dict.fromkeys(f.service for f in installed if f.service))
        for service in services:
            print(f"\nRestarting {service} ...")
            ok, state, tail = restart_and_verify(client, service)
            results[service] = (ok, state, tail)
            print(f"  {service}: {state}")
            if not ok and tail:
                print(f"  --- journalctl -u {service} -n 40 ---")
                print(tail)

        run(client, f"rm -rf {shlex.quote(STAGE_DIR)}")

        # ---- summary -------------------------------------------------------
        print("\n" + "=" * 72)
        print("SUMMARY")
        print("=" * 72)
        for f in installed:
            print(f"  {f.local} -> {f.remote}")
            print(f"    backed up to: {backups.get(f.remote) or '(none — new file)'}")
        if services:
            print("  services restarted:")
            for service in services:
                ok, state, _tail = results[service]
                print(f"    {service}: {state}{'' if ok else '  <-- NOT ACTIVE, see journal above'}")
        else:
            print("  services restarted: (none)")
        print("=" * 72)

        return 0 if all(ok for ok, _state, _tail in results.values()) else 1

    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
