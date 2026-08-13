#!/usr/bin/env python3
"""FPMS raw-duty drive-board driver — the path that makes a slow rover possible.

WHY THIS FILE EXISTS
====================
The board is on Yahboom FACTORY firmware (microROS_Robot v2.0.0). Its `cmd_vel`
path adds `PWM_MOTOR_DEAD_ZONE` (200 of a 400-tick scale) as FEED-FORWARD, so
every non-zero velocity setpoint becomes >=50 % duty. Measured 2026-08-04:
`linear.x = 0.06` for 1.5 s travelled OVER A METRE (~0.7 m/s). There is no slow
on that path, and the 1052 mm M1 diagonal at 0.7 m/s is how the rover hit a wall.

The golden B8B driver (`golden_backup/phase6_latest.py`, `_p5_navdrive`) never
commanded a velocity. It commanded RAW DUTY — `set_motor(m1,m2,m3,m4)` in
-100..100 — over Yahboom's framed serial protocol, where 26/100 is 26 % of full
scale and the dead-zone feed-forward (which lives in the velocity path) is not in
the way. That is 0.6 % distance error and +/-1-4 deg turns, on this chassis, in
this arena. This module is that wire, and nothing else: duty out, counts in.

WHAT THIS MODULE IS NOT
=======================
It is NOT the executor. It holds no control loop, no distance integration, no
guards, no gyro trim. Those belong to `fpms_segment_executor` per AGENT_DESIGN.md
— on the Pi, at 20 Hz, where the LiDAR is. This module refuses to be a place
where policy can accumulate, because "the board is a dumb duty amplifier" is the
one architectural rule (ARCHITECTURE_V2.md) that every failure this month came
from breaking.

The one exception is the deadman, and the deadman only ever sends ZERO. It never
repeats a duty command. A watchdog that re-sent the last duty would be a loop
that outlives its caller, which is the exact hazard it exists to prevent.

*** THREE THINGS YOU MUST READ BEFORE THIS TOUCHES HARDWARE ***
===============================================================

1. THE FRAME BYTES ARE INFERRED, NOT VERIFIED.  See `FrameSpec` below. Today's
   binary analysis of the factory image proves a checksummed framed protocol is
   PRESENT (`PROTOCOL`, `Protocol_Task`, `Check sum error!, CalSum:%d,
   recvSum:%d` — a sum-based checksum). It does NOT prove the header bytes or
   the motor function code, which are inline immediates. Confidence in the
   framing SHAPE is high; in the exact bytes, moderate. So the first hardware
   action MUST be `--probe`, which asks for a firmware version and MOVES
   NOTHING. If the board does not answer a version, do not send it a duty.

2. THIS CANNOT SHARE THE UART WITH micro-ros-agent. See COEXISTENCE below. The
   driver refuses to open a port another process holds. Do not add a `force`
   habit; the failure mode is silent frame corruption on a motor link.

3. THE PORT ASSIGNMENT DISAGREES WITH THE GOLDEN CODE. See DRIVE_PORT below.
   Getting this wrong sends motor frames to the LiDAR.

COEXISTENCE WITH micro-ros-agent — THE ANSWER IS NO
===================================================
Four independent reasons, any one of which is disqualifying:

  BAUD.       `micro-ros-agent.service` pins `-b 921600` on this exact device.
              The framed protocol is documented at 115200 (CLAUDE.md,
              NAV2_BRIEF sec.2). One tty has one baud. Re-opening it at 115200
              under the running agent reconfigures the line for BOTH readers and
              breaks the XRCE framing; leaving it at 921600 means the board is
              listening for XRCE, not for us.
  BYTES.      Two processes reading one tty split the incoming stream
              non-deterministically. The agent would eat our 0xF7 replies and we
              would eat its XRCE frames. Neither side gets a clean read, and the
              side that mis-parses is holding the motors.
  EXCLUSIVE.  pyserial opens with TIOCEXCL by default. Either our open fails
              with EBUSY, or the agent did not take the lock and we corrupt it.
              Both are answers, and neither is "coexists".
  FIRMWARE.   The board answers the framed protocol only in roughly the first
              5 s after reset (NAV2_BRIEF sec.2 / knowledge OPS-010). After that
              the UART belongs to micro-ROS. Even with the tty free, the board
              may simply not be listening.

CONSEQUENCE: the agent must be STOPPED before raw duty, and that costs the
documented 90-225 s board reconnect afterwards. That is a real cost and it is
the reason this is a deliberate mode switch, not a fallback the code can take on
its own. The two supported shapes are:

  A. DUTY MODE (this module):  stop micro-ros-agent -> reset the board -> open
     within the answer window -> drive -> stop -> restart the agent. Encoders
     and gyro must then come from THIS link (see the UNVERIFIED decoders below),
     because /odom_raw and /imu are gone with the agent.
  B. If A does not answer on hardware, the alternative is NOT to go back to
     cmd_vel. It is `firmware_v2/` — our own duty firmware — where the duty
     path is ours and micro-ROS stays the transport. cmd_vel on factory
     firmware has no slow speed and no amount of host-side cleverness adds one.

Verified by reading, not by running: nothing in this file has been executed
against the board. `--selftest` covers the pure logic only.
"""

from __future__ import annotations

import atexit
import math
import os
import struct
import sys
import threading
import time
import weakref

try:                                     # absent on the Windows dev machine;
    import serial                        # present on the Pi (fpms_rover_agent
except Exception:                        # already uses it for the LiDAR).
    serial = None


# ============================================================ B8B CONSTANTS
# Carried over BY NAME and BY VALUE from `_p5_navdrive`
# (golden_backup/phase6_latest.py:1309-1311). These are not defaults to tune;
# they are the measured settings that produced 0.6 % distance error and
# +/-1-4 deg turns on this chassis. Re-deriving them is how a session gets lost.

TKMM = math.pi * 70.0 / 1320.0   # mm per encoder tick: 70 mm wheel, 1320 ticks/rev.
                                 #
                                 # *** DISPUTED -- THIS IMPLIES 6.00 counts/mm ***
                                 # 1/TKMM = 6.002. SPEC.md:242 records 5.5 counts/mm
                                 # as MEASURED. If 5.5 is right, anything trusting this
                                 # constant overshoots by ~9 %.
                                 #
                                 # The two are not reconcilable by choosing a favourite:
                                 #   - B8B measured 0.6 % distance error using 1320, but
                                 #     it measured AT REST after every leg and corrected
                                 #     the residual, so a scale error partly hides in the
                                 #     correction rather than showing up in the total.
                                 #   - the 5.5 measurement was taken on a chassis whose
                                 #     front-left hub was working loose. The 2.7x
                                 #     correction it produced is robust; its third digit
                                 #     is not.
                                 # The factory firmware header says MOTOR_ENCODER_CIRCLE
                                 # 1040, a third value, agreeing with neither.
                                 #
                                 # DO NOT tune this to make one distance come out right:
                                 # a wrong scale here is indistinguishable on a plot from
                                 # a constant coast factor. Re-measure it directly -- push
                                 # a known distance by tape and read the raw counts -- and
                                 # record the result in SPEC.md before trusting either.
DRIVE_COUNTS_PER_MM_DISPUTED = True
"""Set False only when TKMM has been re-derived from a tape-measured push.

Consumers that care about absolute distance should check this and say so in their
output rather than reporting a confident number built on a disputed scale.
"""
DRV = 26                         # drive duty, of 100. A square wave: 0 -> 26 -> 0.
TRN = 50                         # turn duty, of 100.
COAST = 0.93                     # cut a turn at 93 % of target and let momentum
                                 # finish it, then MEASURE the actual angle. Driving
                                 # to the target overshoots by the same amount every
                                 # time, and a bias is the one error a retrace cannot
                                 # cancel -- it reappears identically on the way back.
HZ = 20                          # control rate. The executor's rate, not this
                                 # module's: it is here so the deadman can be sized
                                 # against it and stay in one place.
FSTOP = 120                      # front stop, mm, LiDAR-referenced. Checked by the
                                 # executor before EVERY duty command, 20x a second.
KPH = 10                         # heading P-gain, applied to the gyro integral...
KPH_CLAMP = 6                    # ...and hard-clamped to +/-6 duty counts, i.e. a
                                 # +/-23 % differential. The clamp is what keeps a
                                 # bad gyro from becoming a spin.


# ================================================================ SAFETY LIMITS

HARD_MAX_DUTY = 60
"""Last-line clamp on |duty|, applied at the wire and nowhere else.

Sits above both B8B constants (TRN=50 is the larger) with margin, and far below
anything that could be a runaway. No caller -- present or future -- gets past it.
"""

DEADMAN_S = 0.30
"""Stop if the caller has not refreshed the duty within this long.

Matches the 300 ms deadman ARCHITECTURE_V2.md specifies for the board itself, so
host-side and board-side agree on what "the controller went away" means. Six
control ticks at 20 Hz: long enough that a scheduling hiccup is not a stop, short
enough that at DRV=26 the rover coasts a few mm rather than a few hundred.
"""

MAX_CONTINUOUS_MOTION_S = 15.0
"""Absolute cap on uninterrupted commanded motion, sensor-independent.

B8B's per-move timeout was 12 s, so a healthy executor never reaches this. It
fires only when something upstream has stopped making decisions but is still
refreshing the deadman -- the one failure the deadman alone cannot see.
"""

STOP_REPEATS = 3
"""A zero-duty frame is the only frame always safe to send in any state, so it is
sent more than once and its exceptions are swallowed. A dropped stop is the worst
byte loss in the system; a duplicated stop costs nothing."""


# ================================================================= THE PORT
#
# *** RESOLVED: drive = 1.3, LiDAR = 1.2. Golden is stale. ***
#
# golden_backup/phase6_latest.py:117-119 says the opposite:
#       YAHBOOM_PORT (drive) = ...usb-0:1.2:1.0-port0
#       D500_PORT    (LiDAR) = ...usb-0:1.3:1.0-port0
#
# Nine independent files disagree with it, including three that settle it:
#   micro-ros-agent.service:16        the unit driving the board today
#   fpms-os/.../config.env:92,96      what the shipped image binds
#   firmware/README.md:285            flashes the BOARD on 1.3 -- and you
#                                     cannot flash a LiDAR
# The cables were swapped between the golden session and now. 1.3 is drive.
#
# DO NOT re-resolve this by trying both: the wrong one puts motor frames into
# the LiDAR. The pre-open `udevadm info -q path -n <dev>` check below stays --
# it costs nothing and it is the only thing that would catch the cables being
# swapped back.
#
# Both CP2102 adapters report an IDENTICAL ID_SERIAL. Only USB topology tells
# them apart, which is why binding by /dev/ttyUSBn is refused below rather than
# discouraged -- enumeration order is not guaranteed.

DRIVE_PORT = "/dev/serial/by-path/platform-fc880000.usb-usb-0:1.3:1.0-port0"
LIDAR_PORT = "/dev/serial/by-path/platform-fc880000.usb-usb-0:1.2:1.0-port0"

DUTY_BAUD = 115200
"""The framed protocol's baud (CLAUDE.md, NAV2_BRIEF sec.2). NOT 921600 -- that is
micro-ROS's, and the fact that they differ is half of why the two cannot share
the line."""


# =========================================================== CHASSIS POLARITY
#
# UNMEASURED ON THE CURRENT WIRING. Handled the way fpms_missions.py handles
# TURN_WIRE_SIGN: named, defaulted, and flagged, so a wrong guess costs one
# twitch under an operator's eye instead of a mission.
#
#   golden phase6:      M1,M2 = LEFT   M3,M4 = RIGHT   ("set_motor(l,l,r,r)")
#   fpms_missions.py:   "M1/M2 are now the RIGHT side and M3/M4 the LEFT" after
#                       the 2026-08-01 rewiring.
#
# They cannot both be true. `set_motor()` itself takes four explicit motors and
# applies NO mapping, so it is unaffected. Only `set_sides()` needs this, and it
# refuses to run until the flag below is set by someone who watched the wheels.

MOTORS_LEFT = (1, 2)
MOTORS_RIGHT = (3, 4)
SIDE_MAP_MEASURED = False
"""Flip to True ONLY after an operator has watched one side turn on its own."""

FORWARD_SIGN = +1
"""phase6's header says "set_motor negative values drive forward on this chassis"
yet its own `_fwd` drives with a POSITIVE `pwr` and reverses with a negative one.
The code is what ran, so +1 follows the code -- and `set_sides` is gated on
SIDE_MAP_MEASURED anyway, so this is never applied unverified."""


# ============================================================= THE FRAME SPEC
#
# Rosmaster_Lib is NOT vendored in this repo, NOT in
# cloud/dashboard/backend/requirements.txt, and the one file that imports it
# (pi_encoder_test.py) is documented across CLAUDE.md, NAV2_BRIEF sec.2 and
# fpms_missions.py:255 as returning version=-1 and four zeros against this board.
# So the protocol is implemented here directly on pyserial. That is not a
# workaround -- it is better: we control the baud, we can log the exact bytes,
# and we can switch frame variants without a vendored library's opinion.
#
# TWO VARIANTS ARE DOCUMENTED FOR THIS HARDWARE AND THEY DISAGREE:
#
#   ROSMASTER_V3    head 0xFF, device 0xFC, replies 0xFB, checksum
#                   (sum + (257 - 0xFC)) % 256. This is byte-for-byte what
#                   `Rosmaster_Lib.set_motor()` -- the call the golden code made
#                   -- puts on the wire.
#   YAHBOOM_CONFIG  head 0xFF, tag 0xF8 out / 0xF7 back, checksum sum % 256.
#                   This is the variant CLAUDE.md and NAV2_BRIEF record as
#                   OBSERVED ANSWERING THIS BOARD, in the ~5 s window after reset.
#
# One corroboration worth its weight: CLAUDE.md names "0x51 = firmware version"
# for the config protocol, and 0x51 is also Rosmaster's FUNC_VERSION. The two
# variants appear to share a function-code table even where their framing
# differs, which is the main reason FUNC_MOTOR = 0x10 is worth trying at all.
# (They are not identical: CLAUDE.md's 0x06 = domain id is Rosmaster's
# FUNC_RGB_EFFECT. Do not assume any code beyond the two named here.)
#
# DEFAULT = ROSMASTER_V3, because reproducing the golden wire exactly is the
# whole point. Switch with FPMS_DUTY_FRAME=yahboom_config if a probe says so.


RX_LEN_OFFSET = 2
"""How the board's REPLY length byte relates to the frame length: this code
requires `frame[2] == len(frame) - RX_LEN_OFFSET`, i.e. the same convention the
transmit side uses (length excludes the head and the checksum).

AMBIGUOUS AND UNVERIFIED. Rosmaster_Lib's receive loop reads the length byte and
then that many further bytes INCLUDING the checksum, which implies 3, not 2. If
`--probe` shows `bad frames` climbing while the board is clearly answering, this
constant is the first thing to change -- one number, not a rewrite. It is
deliberately NOT auto-detected: a parser that tries both conventions until one
sums correctly will eventually accept a frame that is neither.
"""


class FrameSpec:
    """One framing variant. Everything uncertain about the protocol is here, so
    switching variants is a constant and not an audit."""

    __slots__ = ("name", "head", "tx_tag", "rx_tag", "complement")

    def __init__(self, name, head, tx_tag, rx_tag, complement):
        self.name = name
        self.head = head
        self.tx_tag = tx_tag
        self.rx_tag = rx_tag
        self.complement = complement

    def encode(self, func, payload=b""):
        """[head, tag, len, func, *payload, checksum].

        `len` is the frame length EXCLUDING the head and the checksum, which is
        how Rosmaster_Lib computes it (`cmd[2] = len(cmd) - 1`, before the
        checksum is appended). Getting this off by one is the single easiest way
        to make a board answer "Check sum error!".
        """
        body = bytearray((self.head, self.tx_tag, 0, func))
        body.extend(payload)
        body[2] = len(body) - 1
        body.append((sum(body) + self.complement) & 0xFF)
        return bytes(body)

    def checksum_ok(self, frame):
        """Validate a received frame: head, tag, declared length, sum.

        Rejects rather than repairs. A frame that fails here is either the other
        variant, the other protocol, or a collision with a second reader of this
        tty -- and every one of those means STOP ASKING, not guess harder.
        """
        if len(frame) < 5:
            return False
        if frame[0] != self.head or frame[1] != self.rx_tag:
            return False
        if frame[2] != len(frame) - RX_LEN_OFFSET:
            return False
        return ((sum(frame[:-1]) + self.complement) & 0xFF) == frame[-1]


FRAME_ROSMASTER_V3 = FrameSpec("rosmaster_v3", 0xFF, 0xFC, 0xFB, 257 - 0xFC)
FRAME_YAHBOOM_CONFIG = FrameSpec("yahboom_config", 0xFF, 0xF8, 0xF7, 0)

FRAMES = {f.name: f for f in (FRAME_ROSMASTER_V3, FRAME_YAHBOOM_CONFIG)}


def default_frame():
    name = os.environ.get("FPMS_DUTY_FRAME", FRAME_ROSMASTER_V3.name).strip()
    if name not in FRAMES:
        raise ValueError(
            f"FPMS_DUTY_FRAME={name!r} is not one of {sorted(FRAMES)}")
    return FRAMES[name]


# Function codes. Only the first two are used by any code path that can move the
# rover; the rest are read-only and marked with the confidence they deserve.
FUNC_MOTOR = 0x10          # inferred from Rosmaster_Lib. THE ONE THAT MOVES.
FUNC_VERSION = 0x51        # corroborated independently by CLAUDE.md. Read-only.
FUNC_AUTO_REPORT = 0x01    # inferred. enable/disable the board's own reporting.
FUNC_REPORT_ENCODER = 0x0D  # inferred. UNVERIFIED payload layout, see below.
FUNC_REPORT_IMU_RAW = 0x0B  # inferred. UNVERIFIED payload layout, see below.

GYRO_RAW_TO_RADPS = 500.0 / 32768.0 * math.pi / 180.0
"""UNVERIFIED. Rosmaster's usual +/-500 deg/s full scale on a 16-bit signed word.
The executor's turn measurement is the integral of this, so if turns come out
proportionally wrong by a constant factor, suspect this number before the code.
"""


# ================================================================= EXCEPTIONS

class DutyDriverError(Exception):
    pass


class UnsafePort(DutyDriverError):
    """The requested device is one we refuse to send motor frames to."""


class PortBusy(DutyDriverError):
    """Another process holds the port -- almost always micro-ros-agent."""


class NotOpen(DutyDriverError):
    pass


# ============================================================ PORT VALIDATION

def validate_port(path):
    """Refuse, loudly, anything that is not provably the drive board's by-path.

    This is a refusal and not a warning because the two CP2102 adapters are
    indistinguishable by ID_SERIAL. There is no recovery from sending a duty
    frame to the LiDAR that is cheaper than not sending it.
    """
    if not path:
        raise UnsafePort("no port given")
    if LIDAR_PORT in path or ":1.2:" in path:
        raise UnsafePort(
            f"{path} is the LiDAR (USB path 1.2), not the drive board. "
            "Motor frames must never go here.")
    if "/dev/ttyUSB" in path or "/dev/ttyACM" in path:
        raise UnsafePort(
            f"{path} binds by enumeration order. Both CP2102 adapters report an "
            "identical ID_SERIAL, so ttyUSBn can silently be the LiDAR. Use the "
            f"stable by-path device: {DRIVE_PORT}")
    if not path.startswith("/dev/serial/by-path/"):
        raise UnsafePort(
            f"{path} is not a /dev/serial/by-path/ device. Only USB topology "
            "distinguishes the drive board from the LiDAR on this rover.")
    return path


def port_holders(path):
    """Every (pid, cmdline) currently holding this device open.

    Returns None -- NOT an empty list -- when it cannot tell (no /proc, i.e. not
    the Pi). "I could not check" and "nothing is holding it" must not be the same
    value, because one of them is a reason to refuse to open a motor link.
    """
    if not os.path.isdir("/proc"):
        return None
    try:
        target = os.path.realpath(path)
    except OSError:
        return None
    found = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        fd_dir = f"/proc/{pid}/fd"
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue                     # gone, or not ours to look at
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) != target:
                    continue
                with open(f"/proc/{pid}/cmdline", "rb") as fh:
                    cmd = fh.read().replace(b"\0", b" ").decode(
                        "utf-8", "replace").strip()
                found.append((int(pid), cmd or "?"))
            except OSError:
                continue
            break
    return found


# ==================================================================== DRIVER

_OPEN_DRIVERS = weakref.WeakSet()


def _stop_all_open_drivers():
    """Last line of defence at interpreter exit. Best effort, never raises: an
    exception here would be an exception on the way out of a process that may
    still have duty on the wire."""
    for drv in list(_OPEN_DRIVERS):
        try:
            drv.stop()
        except Exception:
            pass


atexit.register(_stop_all_open_drivers)


class DutyDriver:
    """Raw duty to the drive board. Duty out, counts in, nothing else.

    Use it as a context manager. `__exit__` stops the motors before it closes the
    port, on the normal path and on every exception path, and that is the only
    reason a bare `open()`/`close()` pair is still exposed at all (the executor's
    process supervisor owns the port for its whole life).

        with DutyDriver() as bot:
            bot.set_motor(DRV, DRV, DRV, DRV)   # refresh me faster than DEADMAN_S
            ...
        # motors are stopped here, however we got here

    THREADING: `set_motor` and the deadman thread both write to the port, so
    every write takes `_lock`. The deadman only ever writes zeros.
    """

    def __init__(self, port=None, baud=DUTY_BAUD, frame=None, transport=None,
                 deadman_s=DEADMAN_S, allow_unchecked_port=False):
        self.port = validate_port(port or os.environ.get(
            "FPMS_DUTY_PORT") or DRIVE_PORT)
        self.baud = int(baud)
        self.frame = frame or default_frame()
        self.deadman_s = float(deadman_s)
        self.allow_unchecked_port = bool(allow_unchecked_port)

        self._transport = transport      # injectable, so --selftest needs no board
        self._injected = transport is not None
        self._lock = threading.RLock()
        self._closing = threading.Event()
        self._wd = None

        # Deadman + motion-budget bookkeeping. `_moving` is what makes the
        # watchdog a stop-only device: with nothing commanded there is nothing to
        # stop, so it stays silent instead of spamming zero frames at the board.
        self._last_cmd_t = 0.0
        self._motion_since = None
        self._moving = False

        self.stops_forced = 0            # deadman fires -- an executor health signal
        self.frames_sent = 0
        self.frames_bad = 0
        self._rx = bytearray()
        self._enc = None                 # None until a frame says otherwise.
        self._enc_t = 0.0                # NEVER zeros: on this project zeros have
        self._gyro_z = None              # meant "wrong protocol" more often than
        self._gyro_t = 0.0               # they have meant "not moving".

    # ------------------------------------------------------------ lifecycle
    def open(self):
        if self._transport is not None and not self._injected:
            return self
        if self._transport is None:
            holders = port_holders(self.port)
            if holders is None and not self.allow_unchecked_port:
                raise PortBusy(
                    f"cannot verify who holds {self.port} (no /proc -- this is "
                    "not the Pi). Refusing to open a motor link on trust. Run "
                    "this on the rover, or pass allow_unchecked_port=True and "
                    "own the consequence.")
            if holders:
                who = "; ".join(f"pid {p}: {c}" for p, c in holders)
                raise PortBusy(
                    f"{self.port} is held by [{who}]. Raw duty CANNOT share this "
                    "UART -- different baud (115200 vs 921600), interleaved "
                    "reads, and pyserial's TIOCEXCL. Stop micro-ros-agent first "
                    "and accept the 90-225 s reconnect afterwards. See "
                    "COEXISTENCE in this file's docstring.")
            if serial is None:
                raise DutyDriverError(
                    "pyserial is not importable here. It is installed on the Pi "
                    "(fpms_rover_agent.py uses it for the LiDAR).")
            self._transport = serial.Serial(
                self.port, self.baud, timeout=0.05, write_timeout=0.5)

        self._closing.clear()
        # STOP IS THE FIRST FRAME ON THE WIRE, always. B8B did this too
        # (open_bot -> stop_bot, before a single plan step). If the board came up
        # holding a duty from a previous session, this is what clears it, and if
        # the frame variant is wrong we find out with a frame that means "0".
        self.stop()
        self._wd = threading.Thread(
            target=self._watchdog, name="duty-deadman", daemon=True)
        self._wd.start()
        _OPEN_DRIVERS.add(self)
        return self

    def close(self):
        """Stop, then close. In that order, unconditionally."""
        try:
            self.stop()
        finally:
            self._closing.set()
            wd, self._wd = self._wd, None
            if wd is not None and wd is not threading.current_thread():
                wd.join(timeout=1.0)
            _OPEN_DRIVERS.discard(self)
            if not self._injected and self._transport is not None:
                try:
                    self._transport.close()
                except Exception:
                    pass
                self._transport = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False                     # never swallow: a caller's exception is
                                         # a reason to stop, not a reason to hide

    # ---------------------------------------------------------------- output
    @staticmethod
    def _clamp_duty(v):
        try:
            v = int(round(float(v)))
        except (TypeError, ValueError):
            # A duty we cannot parse is a duty we do not send. Zero is the only
            # honest interpretation of a bad number aimed at a motor.
            return 0
        return max(-HARD_MAX_DUTY, min(HARD_MAX_DUTY, v))

    def motor_frame(self, m1, m2, m3, m4):
        """The exact bytes `set_motor` would put on the wire. Pure; no I/O.

        Exposed so the frame can be inspected, diffed and unit-tested without a
        board attached -- which is the only way any of this got checked at all.
        """
        vals = [self._clamp_duty(v) for v in (m1, m2, m3, m4)]
        return self.frame.encode(FUNC_MOTOR, struct.pack("4b", *vals)), vals

    def set_motor(self, m1, m2, m3, m4):
        """Command raw duty on all four motors, -100..100 (clamped to
        +/-HARD_MAX_DUTY). Returns the clamped values actually sent.

        Refreshing this call is what feeds the deadman. Calling it once and
        walking away stops the rover after DEADMAN_S, by design.
        """
        frame, vals = self.motor_frame(m1, m2, m3, m4)
        moving = any(vals)
        now = time.monotonic()
        with self._lock:
            if moving and self._motion_since is None:
                self._motion_since = now
            elif not moving:
                self._motion_since = None
            self._last_cmd_t = now
            self._moving = moving
            self._write(frame)
        return vals

    def set_sides(self, left, right):
        """Convenience for a skid-steer executor. GATED, deliberately.

        The motor-to-side map is contested in this repo (see CHASSIS POLARITY),
        and a wrong map turns a drive into a spin. `set_motor` takes four explicit
        motors and needs no map, so nothing is blocked by this refusal.
        """
        if not SIDE_MAP_MEASURED:
            raise DutyDriverError(
                "set_sides() is disabled until SIDE_MAP_MEASURED is True. The "
                "golden code says M1/M2 are LEFT; fpms_missions.py says the "
                "2026-08-01 rewiring made them RIGHT. Have the operator watch "
                "ONE side turn, set the constant, then use this. Until then "
                "call set_motor(m1,m2,m3,m4) explicitly.")
        duty = {}
        for m in MOTORS_LEFT:
            duty[m] = FORWARD_SIGN * left
        for m in MOTORS_RIGHT:
            duty[m] = FORWARD_SIGN * right
        return self.set_motor(duty[1], duty[2], duty[3], duty[4])

    def stop(self):
        """Zero duty, sent STOP_REPEATS times. Never raises.

        Every path that can command motion ends here -- normal return, exception,
        deadman, context-manager exit, interpreter exit. It swallows exceptions
        because a stop that gives up on an I/O error is worse than useless.
        """
        frame = self.frame.encode(FUNC_MOTOR, struct.pack("4b", 0, 0, 0, 0))
        with self._lock:
            self._moving = False
            self._motion_since = None
            self._last_cmd_t = time.monotonic()
            for _ in range(max(1, STOP_REPEATS)):
                try:
                    self._write(frame)
                except Exception:
                    pass
                time.sleep(0.01)

    def _write(self, frame):
        tr = self._transport
        if tr is None:
            raise NotOpen("driver is not open")
        tr.write(frame)
        try:
            tr.flush()
        except Exception:
            pass
        self.frames_sent += 1

    # -------------------------------------------------------------- deadman
    def _watchdog(self):
        """Stop-only. It NEVER re-sends a duty.

        A watchdog that repeated the last command would keep the rover moving
        after its controller died, which is precisely the loop-outliving-its-
        caller hazard it exists to prevent. Two triggers:

          1. the caller stopped refreshing (DEADMAN_S)
          2. the caller kept refreshing but stopped deciding
             (MAX_CONTINUOUS_MOTION_S) -- sensor-independent, so it still fires
             when everything upstream is lying confidently.
        """
        tick = min(0.05, self.deadman_s / 3.0)
        while not self._closing.wait(tick):
            try:
                now = time.monotonic()
                with self._lock:
                    if not self._moving:
                        continue
                    stale = (now - self._last_cmd_t) > self.deadman_s
                    since = self._motion_since
                    over = since is not None and (
                        now - since) > MAX_CONTINUOUS_MOTION_S
                if stale or over:
                    self.stops_forced += 1
                    self.stop()
            except Exception:
                # Never let the watchdog die: it is the last thing standing
                # between a wedged executor and a moving rover.
                try:
                    self.stop()
                except Exception:
                    pass

    # ---------------------------------------------------------------- input
    #
    # EVERYTHING BELOW IS UNVERIFIED AGAINST THIS BOARD.
    #
    # The payload layouts are Rosmaster's, inferred, not read out of the factory
    # binary. They are here because the executor's `_fwd` cannot exist without
    # encoder counts and `_spin` cannot exist without gyro Z, and because with
    # micro-ros-agent stopped there is no /odom_raw and no /imu to get them from.
    #
    # They fail HONESTLY: an unparseable or absent reply leaves the reading None
    # and stale, and `encoders()`/`gyro_z()` return None. They never synthesise a
    # zero. On this project a returned zero has meant "wrong protocol" more often
    # than it has meant "not moving", and a zero delta reads to a control loop as
    # "no progress" -- which is how you get a rover pushing harder into a wall.

    def poll(self):
        """Drain the port and decode whatever whole frames arrived. Non-blocking.

        Call it from the executor's tick. Returns the number of valid frames
        decoded, so a tick that gets 0 for several ticks running can treat the
        link as stale rather than as a stationary rover.
        """
        tr = self._transport
        if tr is None:
            raise NotOpen("driver is not open")
        try:
            waiting = getattr(tr, "in_waiting", 0) or 0
            if waiting:
                self._rx.extend(tr.read(waiting))
        except Exception:
            return 0
        return self._consume()

    def _consume(self):
        good = 0
        head, tag = self.frame.head, self.frame.rx_tag
        while True:
            i = self._rx.find(bytes((head, tag)))
            if i < 0:
                # Keep one byte in case a header straddles two reads.
                if len(self._rx) > 1:
                    del self._rx[:-1]
                return good
            if i:
                del self._rx[:i]
            if len(self._rx) < 3:
                return good
            total = self._rx[2] + 2      # head + declared length + checksum
            if total < 5 or total > 64:
                del self._rx[:2]         # nonsense length: not our frame
                self.frames_bad += 1
                continue
            if len(self._rx) < total:
                return good
            frame = bytes(self._rx[:total])
            del self._rx[:total]
            if not self.frame.checksum_ok(frame):
                self.frames_bad += 1
                continue
            good += 1
            self._decode(frame[3], frame[4:-1])

    def _decode(self, func, payload):
        now = time.monotonic()
        if func == FUNC_REPORT_ENCODER and len(payload) >= 16:
            # UNVERIFIED: four little-endian int32, M1..M4.
            self._enc = list(struct.unpack("<4i", payload[:16]))
            self._enc_t = now
        elif func == FUNC_REPORT_IMU_RAW and len(payload) >= 6:
            # UNVERIFIED: gx, gy, gz as int16 LE; we want gz only.
            gz = struct.unpack("<3h", payload[:6])[2]
            self._gyro_z = gz * GYRO_RAW_TO_RADPS
            self._gyro_t = now

    def encoders(self, max_age_s=0.5):
        """Four tick counters, or None if we have never had them or they are
        stale. `_fwd` must treat None as NO_ENC and stop -- not as no movement."""
        if self._enc is None or (time.monotonic() - self._enc_t) > max_age_s:
            return None
        return list(self._enc)

    def gyro_z(self, max_age_s=0.5):
        """Yaw rate in rad/s, or None if stale. `_spin` integrates this; a None
        must abort the spin, because B8B's `_spin` had no timeout and a dead gyro
        spun it forever (AGENT_DESIGN.md, 'the two guards B8B never had')."""
        if self._gyro_z is None or (time.monotonic() - self._gyro_t) > max_age_s:
            return None
        return self._gyro_z

    def set_auto_report(self, enable=True):
        """Ask the board to stream its sensors. UNVERIFIED function code.

        B8B called the same thing through Rosmaster_Lib
        (`set_auto_report_state(True, False)`) immediately after opening.
        """
        with self._lock:
            self._write(self.frame.encode(
                FUNC_AUTO_REPORT, bytes((1 if enable else 0, 0))))

    def request_version(self, timeout_s=1.0):
        """Ask for the firmware version. SENDS NO DUTY. Returns (major, minor) or
        None.

        This is THE first thing to run on hardware. 0x51 is the one function code
        corroborated by two independent sources, so a valid answer here is
        evidence the framing and the checksum are right, bought without moving a
        wheel. No answer means try the other FrameSpec -- it does NOT mean try a
        motor frame.
        """
        with self._lock:
            self._write(self.frame.encode(FUNC_VERSION))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            tr = self._transport
            if tr is None:
                return None
            try:
                chunk = tr.read(32)
            except Exception:
                return None
            if chunk:
                self._rx.extend(chunk)
                i = self._rx.find(bytes((self.frame.head, self.frame.rx_tag)))
                if i >= 0 and len(self._rx) >= i + 3:
                    total = self._rx[i + 2] + 2
                    if len(self._rx) >= i + total:
                        frame = bytes(self._rx[i:i + total])
                        del self._rx[:i + total]
                        if (self.frame.checksum_ok(frame)
                                and frame[3] == FUNC_VERSION
                                and len(frame) >= 7):
                            return (frame[4], frame[5])
            time.sleep(0.01)
        return None


def duty_session(**kwargs):
    """`with duty_session() as bot:` -- stop is guaranteed on exit and on any
    exception. Identical to `with DutyDriver(...)`; it exists so call sites read
    as a bounded session rather than as an object with a lifetime."""
    return DutyDriver(**kwargs)


# =================================================================== SELFTEST

def _selftest():
    """Pure-logic checks. No port is opened and nothing can move.

    Every expected byte string below was computed by hand from the frame rules
    and is asserted against the code, so this catches a drifted encoder -- it
    canNOT tell you the board agrees. Only hardware can do that.
    """
    ok = True

    def check(name, got, want):
        nonlocal ok
        good = got == want
        ok = ok and good
        print(f"  {'PASS' if good else 'FAIL'}  {name}")
        if not good:
            print(f"        got  {got!r}\n        want {want!r}")

    print("frame encoding (hand-computed):")
    f = FRAME_ROSMASTER_V3
    # [FF FC 07 10 1A 1A 1A 1A] sum=634, +complement 5 = 639, 639 & 0xFF = 0x7F
    check("set_motor(26,26,26,26)",
          f.encode(FUNC_MOTOR, struct.pack("4b", 26, 26, 26, 26)).hex(),
          "fffc07101a1a1a1a7f")
    # [FF FC 07 10 00 00 00 00] sum=530, +5 = 535, & 0xFF = 0x17
    check("set_motor(0,0,0,0)",
          f.encode(FUNC_MOTOR, struct.pack("4b", 0, 0, 0, 0)).hex(),
          "fffc07100000000017")
    check("negative duty is two's complement",
          f.encode(FUNC_MOTOR, struct.pack("4b", -26, -26, 26, 26))[4:8].hex(),
          "e6e61a1a")
    check("length byte excludes head and checksum",
          f.encode(FUNC_VERSION)[2], 3)

    print("checksum validation:")
    # 7 bytes total, so the length byte is 7 - RX_LEN_OFFSET = 5.
    good = bytes((0xFF, 0xFB, 7 - RX_LEN_OFFSET, FUNC_VERSION, 2, 0))
    good += bytes(((sum(good) + f.complement) & 0xFF,))
    check("accepts a well-formed reply", f.checksum_ok(good), True)
    check("rejects a corrupted checksum",
          f.checksum_ok(good[:-1] + bytes((good[-1] ^ 0xFF,))), False)
    check("rejects the wrong rx tag",
          f.checksum_ok(bytes((0xFF, 0xF7)) + good[2:]), False)
    bad_len = bytearray(good)
    bad_len[2] += 1
    bad_len[-1] = (bad_len[-1] + 1) & 0xFF          # keep the sum valid
    check("rejects a mis-declared length (RX_LEN_OFFSET guard)",
          f.checksum_ok(bytes(bad_len)), False)

    print("duty clamping:")
    check("clamps high", DutyDriver._clamp_duty(999), HARD_MAX_DUTY)
    check("clamps low", DutyDriver._clamp_duty(-999), -HARD_MAX_DUTY)
    check("TRN survives the clamp", DutyDriver._clamp_duty(TRN), TRN)
    check("garbage becomes 0", DutyDriver._clamp_duty("fast"), 0)

    print("port validation:")
    for bad, why in ((LIDAR_PORT, "the LiDAR"),
                     ("/dev/ttyUSB0", "ttyUSBn"),
                     ("/dev/serial/by-id/usb-x", "by-id")):
        try:
            validate_port(bad)
            check(f"refuses {why}", "accepted", "UnsafePort")
        except UnsafePort:
            check(f"refuses {why}", True, True)
    check("accepts the drive board", validate_port(DRIVE_PORT), DRIVE_PORT)

    print("deadman and stop guarantees (fake transport, nothing can move):")

    class FakeSerial:
        in_waiting = 0

        def __init__(self):
            self.writes = []

        def write(self, b):
            self.writes.append(bytes(b))

        def flush(self):
            pass

        def read(self, n=1):
            return b""

        def close(self):
            pass

    stop_frame = f.encode(FUNC_MOTOR, struct.pack("4b", 0, 0, 0, 0))

    fake = FakeSerial()
    with DutyDriver(transport=fake, deadman_s=0.15) as bot:
        bot.set_motor(DRV, DRV, DRV, DRV)
    check("context manager exit stops", fake.writes[-1], stop_frame)

    fake = FakeSerial()
    try:
        with DutyDriver(transport=fake, deadman_s=0.15) as bot:
            bot.set_motor(DRV, DRV, DRV, DRV)
            raise RuntimeError("simulated executor crash")
    except RuntimeError:
        pass
    check("exception path stops", fake.writes[-1], stop_frame)

    fake = FakeSerial()
    bot = DutyDriver(transport=fake, deadman_s=0.15).open()
    try:
        bot.set_motor(DRV, DRV, DRV, DRV)
        time.sleep(0.6)                  # caller "goes away"
        check("deadman fired", bot.stops_forced >= 1, True)
        check("deadman sent a stop", fake.writes[-1], stop_frame)
        n = len(fake.writes)
        time.sleep(0.3)
        check("deadman does not spam once stopped", len(fake.writes), n)
    finally:
        bot.close()

    fake = FakeSerial()
    bot = DutyDriver(transport=fake, deadman_s=0.15).open()
    try:
        bot.set_motor(0, 0, 0, 0)
        n = len(fake.writes)
        time.sleep(0.4)
        check("deadman is silent when nothing is commanded",
              len(fake.writes), n)
    finally:
        bot.close()

    print("rx decoding (UNVERIFIED layouts -- shape only):")
    bot = DutyDriver(transport=FakeSerial())
    check("encoders() is None before any frame", bot.encoders(), None)
    check("gyro_z() is None before any frame", bot.gyro_z(), None)
    enc = f.encode(FUNC_REPORT_ENCODER, struct.pack("<4i", 10, -20, 30, -40))
    enc = bytes((f.head, f.rx_tag)) + enc[2:-1]
    enc += bytes(((sum(enc) + f.complement) & 0xFF,))
    bot._rx.extend(enc)
    bot._consume()
    check("decodes four int32 counters", bot.encoders(), [10, -20, 30, -40])

    print("\nSELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def _show_frames():
    """Print the bytes every command would send. Opens nothing."""
    for f in FRAMES.values():
        print(f"--- {f.name} (head {f.head:#04x} tx {f.tx_tag:#04x} "
              f"rx {f.rx_tag:#04x} complement {f.complement}) @ {DUTY_BAUD} ---")
        for label, func, payload in (
                ("stop            ", FUNC_MOTOR, struct.pack("4b", 0, 0, 0, 0)),
                (f"drive DRV={DRV}    ", FUNC_MOTOR,
                 struct.pack("4b", DRV, DRV, DRV, DRV)),
                (f"turn  TRN={TRN}    ", FUNC_MOTOR,
                 struct.pack("4b", -TRN, -TRN, TRN, TRN)),
                ("version query   ", FUNC_VERSION, b""),
        ):
            print(f"  {label} {f.encode(func, payload).hex(' ')}")


def _probe(port, frame_name):
    """Read-only hardware probe. SENDS NO DUTY beyond the mandatory open-stop.

    Intended to be run by the operator, at the rover, with micro-ros-agent
    stopped and the board just reset.
    """
    frame = FRAMES[frame_name]
    print(f"probing {port} @ {DUTY_BAUD} with frame {frame.name}")
    holders = port_holders(port)
    if holders is None:
        print("  cannot check port holders (no /proc) -- run this on the Pi")
        return 2
    if holders:
        for pid, cmd in holders:
            print(f"  HELD by pid {pid}: {cmd}")
        print("  stop micro-ros-agent first; this link cannot be shared")
        return 2
    with DutyDriver(port=port, frame=frame) as bot:
        bot.poll()
        ver = bot.request_version()
        print(f"  version reply: {ver!r}")
        print(f"  frames sent {bot.frames_sent}  bad frames {bot.frames_bad}")
        if ver is None:
            print("  NO ANSWER. Do NOT send a duty frame. Try the other "
                  "FrameSpec, and re-check the ~5 s post-reset window.")
            return 1
    return 0


def main(argv):
    args = list(argv[1:])
    if not args or args[0] in ("-h", "--help"):
        print(__doc__)
        print("usage: fpms_duty_driver.py [--selftest | --show-frames | "
              "--probe [--port P] [--frame NAME]]")
        print("\nNo argument of this CLI commands motion. --probe is read-only.")
        return 0
    if args[0] == "--selftest":
        return _selftest()
    if args[0] == "--show-frames":
        _show_frames()
        return 0
    if args[0] == "--probe":
        port = DRIVE_PORT
        frame_name = default_frame().name
        for i, a in enumerate(args):
            if a == "--port" and i + 1 < len(args):
                port = args[i + 1]
            if a == "--frame" and i + 1 < len(args):
                frame_name = args[i + 1]
        return _probe(validate_port(port), frame_name)
    print(f"unknown option {args[0]!r}")
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
