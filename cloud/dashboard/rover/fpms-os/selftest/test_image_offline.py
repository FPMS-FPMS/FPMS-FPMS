#!/usr/bin/env python3
"""Offline checks on the FPMS-OS tree. No Pi, no hardware, no image required.

    python3 selftest/test_image_offline.py

In the spirit of the project's existing test_stack.py, which caught two real
bugs in the stop-escalation logic before any of it ran on hardware.

THE CHECK THAT JUSTIFIES THIS FILE is `exec_paths_exist`: every ExecStart and
ExecStartPre must point at something the image actually contains. That is
exactly the class of defect that shipped for months -- fpms-wait-net was named
by three units as an ExecStartPre with no "-" prefix, and no repository
contained it. Two units could not start at all, and the only symptom was a
missing LiDAR and a missing TF tree.
"""

import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
OVL = os.path.join(ROOT, "overlay")
UNITS = os.path.join(OVL, "etc/systemd/system")

failures, warnings = [], []


def fail(check, msg):
    failures.append((check, msg))


def warn(check, msg):
    warnings.append((check, msg))


def units():
    if not os.path.isdir(UNITS):
        return []
    return sorted(f for f in os.listdir(UNITS)
                  if f.endswith((".service", ".target")))


def unit_text(name):
    with open(os.path.join(UNITS, name), encoding="utf-8") as fh:
        return fh.read()


def unit_directives(name):
    """Unit text with comments stripped.

    These units are heavily commented, and the comments legitimately DISCUSS
    directives they deliberately omit -- fpms-rover-agent.service explains at
    length why it sets no ROS_DOMAIN_ID (it is an MQTT client, not a ROS node).
    Matching on raw text turns every such explanation into a false failure.
    """
    return "\n".join(l for l in unit_text(name).splitlines()
                     if not l.lstrip().startswith("#"))


# Binaries the build installs from the rover source tree (stage 30) rather than
# from the overlay. They are real dependencies, they are just not in overlay/.
INSTALLED_BY_BUILD = {
    "/usr/local/bin/fpms-rover-agent",     # from fpms_rover_agent.py
    "/usr/local/bin/fpms-uros-supervisor",  # from stack/fpms-uros-supervisor
}


# --------------------------------------------------------------------------
def test_units_parse():
    """Every unit has the sections and directives systemd requires."""
    for u in units():
        body = unit_text(u)
        if "[Unit]" not in body:
            fail("units_parse", "%s has no [Unit] section" % u)
        if u.endswith(".service"):
            if "[Service]" not in body:
                fail("units_parse", "%s has no [Service] section" % u)
            if "ExecStart=" not in body:
                fail("units_parse", "%s has no ExecStart=" % u)
        # A Type=simple service with no Restart= silently stays dead after a
        # crash. Every FPMS service is meant to be Restart=always.
        if u.endswith(".service") and "Type=oneshot" not in body:
            if "Restart=" not in body:
                fail("units_parse", "%s has no Restart=" % u)


def test_exec_paths_exist():
    """ExecStart/ExecStartPre binaries must exist in the overlay or be system paths.

    THIS IS THE CHECK THAT WOULD HAVE CAUGHT fpms-wait-net.
    """
    system_ok = ("/bin/bash", "/bin/sh", "/bin/sleep", "/bin/systemctl",
                 "/usr/bin/python3", "/usr/bin/env", "/bin/true", "/bin/echo")
    src_root = os.path.abspath(os.path.join(ROOT, ".."))
    for u in units():
        for line in unit_directives(u).splitlines():
            line = line.strip()
            m = re.match(r"^(ExecStart|ExecStartPre|ExecStartPost|ExecStopPost)=(-?)(\S+)", line)
            if not m:
                continue
            directive, dash, binary = m.groups()
            if not binary.startswith("/"):
                continue
            if binary in system_ok:
                continue
            local = os.path.join(OVL, binary.lstrip("/"))
            if os.path.exists(local):
                continue
            if binary in INSTALLED_BY_BUILD:
                # Installed by stage 30 from the rover source tree. Confirm the
                # source is actually there -- otherwise the unit still cannot
                # start, the file is just missing one level further back.
                stem = os.path.basename(binary)
                found = any(
                    os.path.exists(os.path.join(src_root, c))
                    for c in (stem, "stack/" + stem,
                              stem.replace("-", "_") + ".py",
                              stem.replace("fpms-", "fpms_").replace("-", "_") + ".py")
                )
                if not found:
                    fail("exec_paths",
                         "%s: %s=%s is expected from the rover source tree, "
                         "but no matching source file was found" % (u, directive, binary))
                continue
            if binary.startswith(("/usr/bin/", "/bin/", "/sbin/", "/usr/sbin/")):
                # Provided by apt inside the image; we cannot check it here.
                continue
            # No "-" prefix means a missing binary STOPS THE UNIT STARTING.
            if dash == "-":
                warn("exec_paths", "%s: %s=%s not in overlay (optional)"
                     % (u, directive, binary))
            else:
                fail("exec_paths",
                     "%s: %s=%s does not exist in the overlay, and has no '-' "
                     "prefix, so the unit will never start"
                     % (u, directive, binary))


def test_ros_units_have_dds_profile():
    """Every ROS unit must carry the full env block.

    A unit missing FASTRTPS_DEFAULT_PROFILES_FILE falls back to shared memory
    and reintroduces the silent no-data bug. A unit missing RMW_IMPLEMENTATION
    can end up on a different RMW from the rest of the stack, which discovers
    nothing and reports no error -- fpms-teleop shipped like that.
    """
    for u in units():
        body = unit_directives(u)
        if "ROS_DOMAIN_ID" not in body:
            continue  # not a ROS unit
        for key in ("RMW_IMPLEMENTATION",
                    "FASTRTPS_DEFAULT_PROFILES_FILE"):
            if key not in body:
                fail("dds_env", "%s sets ROS_DOMAIN_ID but not %s" % (u, key))
        if "ROS_DOMAIN_ID=20" not in body:
            fail("dds_env", "%s does not set ROS_DOMAIN_ID=20" % u)


def test_env_file_before_environment():
    """Ordering decides who wins, and the correct answer differs by unit type.

    systemd applies EnvironmentFile= and Environment= in the order they appear,
    later overriding earlier. So:

      ROS units   Environment= must come AFTER EnvironmentFile=, so the UNIT
                  wins. ROS_DOMAIN_ID and RMW_IMPLEMENTATION are safety
                  properties -- a stale value in config.env must not be able to
                  put the rover on a domain where every topic list reads empty,
                  which is indistinguishable from dead hardware.

      non-ROS     EnvironmentFile= may come last, so CONFIG.ENV wins. Every
                  FPMS_NPU_* value is a tuning knob, and config.env is
                  documented as the single configuration point. A knob set in
                  the documented place that silently has no effect is its own
                  failure mode -- fpms-npud shipped that way for one revision.

    What is never acceptable is a unit whose comment claims one and whose
    directive order does the other, so this also checks the two agree.
    """
    for u in units():
        body = unit_directives(u)
        if "EnvironmentFile=" not in body or "Environment=" not in body:
            continue
        lines = body.splitlines()
        first_file = next(i for i, l in enumerate(lines)
                          if l.strip().startswith("EnvironmentFile="))
        last_env = max(i for i, l in enumerate(lines)
                       if l.strip().startswith("Environment="))
        is_ros = "ROS_DOMAIN_ID" in body

        if is_ros and first_file > last_env:
            fail("env_order",
                 "%s is a ROS unit but EnvironmentFile= comes after the last "
                 "Environment=, so config.env can override ROS_DOMAIN_ID and "
                 "RMW_IMPLEMENTATION. A stale domain there reads as dead "
                 "hardware." % u)

        # For a non-ROS unit either order is defensible, but the file must not
        # claim the opposite of what it does. Look for the claim in the prose.
        if not is_ros:
            raw = unit_text(u).lower()
            claims_cfg_wins = ("config.env wins" in raw
                               or "config.env does win" in raw
                               or "config.env overrides" in raw)
            cfg_actually_wins = first_file > last_env
            if claims_cfg_wins and not cfg_actually_wins:
                fail("env_order",
                     "%s says config.env wins, but EnvironmentFile= comes "
                     "before the Environment= defaults, so the UNIT wins and "
                     "every knob set in config.env is silently ignored." % u)


def test_dds_profile():
    p = os.path.join(OVL, "etc/fpms/fastdds_udp_only.xml")
    if not os.path.exists(p):
        fail("dds_profile", "fastdds_udp_only.xml missing")
        return
    try:
        ET.parse(p)
    except ET.ParseError as exc:
        fail("dds_profile",
             "does not parse (%s) -- Fast DDS SILENTLY IGNORES a malformed "
             "profile and falls back to shared memory" % exc)
        return
    body = open(p, encoding="utf-8").read()
    if "<useBuiltinTransports>false</useBuiltinTransports>" not in body:
        fail("dds_profile",
             "useBuiltinTransports is not false -- the builtin set includes "
             "SHM and Fast DDS still prefers it for same-host peers, so the "
             "profile would fix nothing while looking correct")
    if "UDPv4" not in body:
        fail("dds_profile", "no UDPv4 transport declared")


def test_sudoers():
    p = os.path.join(OVL, "etc/sudoers.d/fpms-cored")
    if not os.path.exists(p):
        fail("sudoers", "fpms-cored sudoers rule missing")
        return
    body = open(p, encoding="utf-8").read()
    if "fpms-missions.service" not in body:
        fail("sudoers",
             "rule does not name fpms-missions.service. sudo matches literal "
             "argv, and a mismatch turns the last-resort STOP into a silent "
             "password prompt.")
    if "ALL=(ALL) NOPASSWD:ALL" in body or "*" in body:
        fail("sudoers", "rule is too broad -- it must be exactly one command")
    try:
        r = subprocess.run(["visudo", "-cf", p], capture_output=True, text=True)
        if r.returncode != 0:
            fail("sudoers", "visudo -cf rejects it: %s" % r.stderr.strip())
    except FileNotFoundError:
        warn("sudoers", "visudo not available here; not syntax-checked")


def test_rosbridge_whitelist():
    p = os.path.join(OVL, "etc/fpms/rosbridge_params.yaml")
    if not os.path.exists(p):
        fail("rosbridge", "rosbridge_params.yaml missing -- rosbridge would "
                          "start with NO whitelist")
        return
    body = open(p, encoding="utf-8").read()
    m = re.search(r'^\s*topics_glob:\s*"(.*)"', body, re.M)
    if not m:
        fail("rosbridge", "no topics_glob found")
        return
    globs = m.group(1)
    for forbidden in ("/cmd_vel", "/cmd_duty"):
        if forbidden in globs:
            fail("rosbridge",
                 "%s is in topics_glob. Drive topics must be ABSENT so a "
                 "stray click or a stale browser tab cannot turn a wheel."
                 % forbidden)
    if re.search(r'^\s*params_glob:\s*"\[\]"', body, re.M) is None:
        fail("rosbridge", "params_glob is not empty -- a browser could "
                          "reconfigure a live node")


def test_masked_and_enabled_disjoint():
    """A unit cannot be both in the boot set and masked."""
    p = os.path.join(ROOT, "scripts/60-enable-units.sh")
    if not os.path.exists(p):
        fail("enable", "60-enable-units.sh missing")
        return
    body = open(p, encoding="utf-8").read()
    enabled = set(re.findall(r"^\s+(fpms-[\w-]+\.(?:service|target)|micro-ros-agent\.service)",
                             body, re.M))
    masked = set(re.findall(r"for u in (fpms-[\w.-]+ fpms-[\w.-]+); do", body))
    masked = set(masked.pop().split()) if masked else set()
    overlap = enabled & masked
    if overlap:
        fail("enable", "units both enabled and masked: %s" % ", ".join(overlap))
    for u in masked:
        if not os.path.exists(os.path.join(UNITS, u)):
            fail("enable",
                 "%s is masked but not shipped. Masking a unit that does not "
                 "exist is not the same guarantee -- a restore could put an "
                 "unmasked /cmd_vel writer back on the robot." % u)


def test_no_secrets():
    """Nothing secret-shaped may be baked into the overlay."""
    patterns = [
        (re.compile(r"(?i)\bpassword\s*=\s*['\"]?[A-Za-z0-9!@#$%^&*_-]{8,}"), "password literal"),
        (re.compile(r"(?i)bearer\s+[A-Za-z0-9._-]{20,}"), "bearer token"),
        (re.compile(r"eyJ[A-Za-z0-9_-]{20,}\."), "JWT"),
    ]
    allow = ("__FPMS_FIRSTBOOT_WILL_REPLACE_THIS__", "changeme", "fpmsrover",
             "<REDACTED", "MyMobileHotspot", "MyHomeWiFi")
    for base, _, files in os.walk(OVL):
        for f in files:
            path = os.path.join(base, f)
            try:
                body = open(path, encoding="utf-8", errors="ignore").read()
            except OSError:
                continue
            for rx, label in patterns:
                for hit in rx.findall(body):
                    if any(a in hit or a in body[:0] for a in allow):
                        continue
                    if any(a in body for a in allow) and "password" in hit.lower():
                        continue
                    fail("secrets", "%s in %s: %s"
                         % (label, os.path.relpath(path, ROOT), hit[:40]))


def test_config_env_covers_units():
    """Every ${FPMS_*} a unit expands should be defined in config.env."""
    cfg_path = os.path.join(OVL, "etc/fpms/config.env")
    if not os.path.exists(cfg_path):
        fail("config", "config.env missing -- every service silently falls "
                       "back to a broker at 192.168.137.1 and retries forever")
        return
    defined = set(re.findall(r"^([A-Z_][A-Z0-9_]*)=", open(cfg_path, encoding="utf-8").read(), re.M))
    for u in units():
        for var in re.findall(r"\$\{(FPMS_[A-Z0-9_]+)(?::-[^}]*)?\}", unit_directives(u)):
            if var not in defined:
                # Units use ${VAR:-default}; a missing definition is survivable
                # but means the documented value lives in only one place.
                warn("config", "%s expands %s, which config.env does not define"
                     % (u, var))


def test_lidar_port_not_ttyusb():
    cfg = os.path.join(OVL, "etc/fpms/config.env")
    if not os.path.exists(cfg):
        return
    body = open(cfg, encoding="utf-8").read()
    m = re.search(r"^FPMS_LIDAR_PORT=(.*)$", body, re.M)
    if m and "/dev/ttyUSB" in m.group(1):
        fail("serial",
             "FPMS_LIDAR_PORT is %s. Both CP2102 adapters report ID_SERIAL "
             "0001, and ttyUSBn is enumeration order, so this can silently "
             "become the drive board -- putting motor frames into the LiDAR."
             % m.group(1).strip())


def test_no_calibration_shipped():
    """A guessed calibration profile is worse than none."""
    p = os.path.join(OVL, "etc/fpms/calibration.json")
    if os.path.exists(p):
        fail("calibration",
             "a real calibration.json is in the overlay. A missing profile "
             "changes nothing; a guessed one is silent and total. Ship only "
             "calibration.json.example.")


# --------------------------------------------------------------------------
def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        try:
            t()
        except Exception as exc:  # noqa: BLE001
            fail(t.__name__, "test itself raised: %r" % exc)

    print("=" * 72)
    print(" FPMS-OS offline image checks")
    print("=" * 72)
    print("  %d checks run" % len(tests))
    for check, msg in warnings:
        print("  [WARN] %s: %s" % (check, msg))
    for check, msg in failures:
        print("  [FAIL] %s: %s" % (check, msg))
    print("-" * 72)
    if failures:
        print("  %d FAILED, %d warnings" % (len(failures), len(warnings)))
    else:
        print("  all passed, %d warnings" % len(warnings))
    print("=" * 72)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
