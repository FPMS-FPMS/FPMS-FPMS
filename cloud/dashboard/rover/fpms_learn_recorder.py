#!/usr/bin/env python3
"""
FPMS RUN RECORDER -- the dataset this rover has been throwing away.

    ROS_DOMAIN_ID=20 python3 fpms_learn_recorder.py
    ROS_DOMAIN_ID=20 python3 fpms_learn_recorder.py --once      # 60s smoke test
    ROS_DOMAIN_ID=20 python3 fpms_learn_recorder.py --print     # echo to stdout

WHAT THIS IS
    A subscribe-only witness. It watches the rover drive -- under a mission,
    under teleop, under anything -- and writes one newline-delimited JSON
    record per MOTION EPISODE: what was commanded, what the wheels actually
    did, and every independent reference that was available at the time.
    `fpms_learn_fit.py` then reads those files offline and fits the drive
    constants that this project has been arguing about for weeks.

WHAT THIS IS NOT
    It is not a learning model and it does not run one. There is no model on
    this Pi and none can be built here -- only the RKNN *runtime* is installed
    and authoring a model needs rknn-toolkit2 on an x86 host, which this
    project does not have. What follows is instrumentation for PARAMETER
    ESTIMATION. See README_LEARN.md, which says so in the first paragraph.

WHY IT CANNOT DRIVE THE ROVER
    Three independent structural guards, not a promise in a comment:

      1. The node class OVERRIDES `create_publisher` to raise for every topic
         but one. There is no code path in rclpy that puts a message on a
         topic without it. The exception is /parameter_events, which rclpy's
         own Node.__init__ creates unconditionally and which carries ROS
         parameter declarations -- nothing in this stack acts on it and the
         motor board does not read it. It is allowed BY NAME, not by a
         "during construction" window, because a name cannot be re-opened by
         accident.
      2. `_audit()` runs once a second and kills the process if any publisher
         outside that one name is ever held. It inspects what the node
         ACTUALLY HAS rather than trusting the door things came through, so
         it still catches anything that bypassed (1).
      3. The MQTT client class overrides `publish` to raise, and no LWT is
         set. The only MQTT traffic this process generates is SUBSCRIBE.

    It also never imports a message type it could actuate with beyond the
    ones it must SUBSCRIBE to, and it holds no reference to any publisher.

THE MEASUREMENT DISCIPLINE, INHERITED FROM B8B AND fpms_drive.py
    Every number here is taken AT REST. An episode is not closed when the
    command goes to zero -- it is closed when the command has been zero for
    SETTLE_S *and* the wheel ticks have stopped changing for STILL_S. Coast
    is therefore INSIDE the measurement, which is what makes the measurement
    honest: the rover's total displacement for that burst is what is
    recorded, not the displacement at the moment the wire was cut.

WHEEL INDEX CONVENTION
    /wheel_ticks is Int32MultiArray[4]:

        index 0 = FRONT LEFT     index 1 = FRONT RIGHT
        index 2 = REAR  LEFT     index 3 = REAR  RIGHT

    Taken from fpms_drive.py's `tick_yaw` docstring, which did not assume it
    -- it checked the layout against a watched floor run where the rear-right
    wheel read +24% against its neighbours and the operator confirmed a small
    LEFT drift. That cross-check is the only reason this convention is
    trusted here.

WHAT IS DELIBERATELY RECORDED RAW, AND WHY
    * Tick deltas are stored as RAW COUNTS, per wheel, signed. They are never
      divided by counts/mm on the way in, because counts/mm is the thing
      being fitted and baking a disputed constant into the dataset would make
      the dataset unable to settle the dispute. (5.5 hand-push vs 6.0 derived
      vs 14.8 tape -- the whole point.)
    * Tick-differential heading is stored as a COUNT DIFFERENCE, not degrees,
      for the same reason: degrees would require the effective track width,
      which has four conflicting values live in this codebase (105 mm in
      fpms_rtos_follower, 170 mm in fpms_odom_tf, 255 mm in fpms_drive, and
      271 mm in the older notes) and is exactly what needs fitting.
    * /imu angular_velocity.z is recorded even though it is CURRENTLY ALWAYS
      0.0 on this build (measured 2026-08-06: ~230 samples all zero, through
      a confirmed hand rotation; chip, accel, register config and register
      base all verified good -- the fault is the gyro register read itself).
      It is recorded anyway so that the dataset becomes turn-capable the
      instant the firmware deadband is fixed, with no change here and no
      re-run of the drives already banked.

GROUND TRUTH -- THE ONE THING THIS FILE CANNOT PRODUCE
    Encoders cannot calibrate encoders. Two references in this dataset are
    independent of the encoders and one is not:

      INDEPENDENT  lidar_front_mm before/after a straight run at a flat wall
      INDEPENDENT  an operator's tape measure, appended by hand (see below)
      CIRCULAR     /odom and /odom_raw pose delta -- the board integrates
                   those from these same encoders with its own COUNTS_PER_REV,
                   so fitting ticks against them recovers the FIRMWARE'S
                   ASSUMPTION, not the truth. Recorded for completeness and
                   refused as a headline reference by the fitter.

    To add a tape measurement, append a line to today's file by hand:

      {"rec":"truth","ts":1754500000.0,"seq":41,"true_mm":802.0,
       "note":"tape, start mark to stop mark, 2 people"}

    `seq` is the episode number printed in the segment record. The fitter
    joins on it. Nothing in this process ever writes a `truth` record.

LEARNING FROM THE GRID (schema 4)
    The drive constants above are half the ground truth this rover throws
    away. The other half is spatial: every run measures where the arena's
    obstacles are, what route was planned through them, which route was
    actually driven, and how far each segment missed by -- and all four are
    discarded when the mission ends.

    Each episode record therefore also carries:

      `grid`       the cells the LIVE rule believed at the start and end of
                   the episode, mirrored from /scan_lidar and the arena pose
                   exactly as fpms_missions.OccupancyGrid.integrate does it,
                   plus that node's OWN `occupancy` stats off MQTT as a
                   cross-check. When the two disagree, the mirror is wrong and
                   the record says so rather than hiding it.
      `route`      the planned polyline in force (telemetry/mission_plan) and
                   the arena pose actually reached, so planned-vs-driven is a
                   subtraction rather than an archaeology exercise.
      `obstacles`  every events/replan that fired during the episode.

    And across runs, `fpms_grid_learn.GridPrior` accumulates those cells into
    /var/lib/fpms/learned_grid.json -- a decayed hit/miss count per cell that a
    planner may read as a COST PRIOR. It is purely additive: absent or
    malformed, the planner ignores it and behaves as it does today. Nothing in
    this process is allowed to influence a live stop decision.

RESIDUALS COME FROM events/mission_done, NOT telemetry/residual
    `telemetry/residual` is documented in STACK.md and subscribed by two files,
    and NOTHING PUBLISHES IT -- the deployed fpms_missions.py has no
    `_publish_residual` (the repo mirror does; the two copies have diverged).
    The subscription is kept because it costs nothing and would start working
    the day that is fixed, but the commanded-vs-measured numbers are taken from
    `events/mission_done` -> `measured[]`, which this rover really does emit.
"""
import os
import sys
import json
import math
import time
import errno
import signal
import socket
import threading
import statistics
from datetime import datetime, timezone

SCHEMA = 4          # bump on any breaking field change; the fitter checks it
                    # 4 = added `grid`, `route`, `obstacles` to segment records
                    #     and the learned grid prior. Every schema-3 field kept
                    #     its name and its meaning, so a schema-3 dataset still
                    #     fits and a schema-4 one still fits with an old fitter.
NODE_NAME = "fpms_learn_recorder"

# ---------------------------------------------------------------- config
THING = os.environ.get("FPMS_THING_NAME", "rover2")
BROKER = os.environ.get("FPMS_MQTT_HOST", "127.0.0.1")
PORT = int(os.environ.get("FPMS_MQTT_PORT", "1883"))
MQTT_USER = os.environ.get("FPMS_MQTT_USER", "fpms")
MQTT_PASS = os.environ.get("FPMS_MQTT_PASS", "fpms1234")

# Survives a reboot, unlike /tmp. /var/lib/fpms is where this stack already
# keeps machine state; the unit chowns the `learn` subdirectory to the user it
# runs as rather than running this as root to write into a root-owned tree.
OUT_DIR = os.environ.get("FPMS_LEARN_DIR", "/var/lib/fpms/learn")

# --- disk bounds ------------------------------------------------------------
# A recorder that fills the disk takes the mission executor down with it, which
# is a far worse outcome than a gap in a dataset. Two independent bounds, both
# enforced on the way in: one file cannot grow past MAX_FILE_MB (it rolls to
# a .1, .2 ... suffix), and the directory cannot hold more than MAX_DIR_MB
# (the OLDEST files are deleted first). The second is the one that actually
# saves the rover, because the first only bounds a single day.
MAX_FILE_MB = float(os.environ.get("FPMS_LEARN_MAX_FILE_MB", "48"))
MAX_DIR_MB = float(os.environ.get("FPMS_LEARN_MAX_DIR_MB", "384"))
# Never write when the filesystem is this close to full, whatever the caps say.
MIN_FREE_MB = float(os.environ.get("FPMS_LEARN_MIN_FREE_MB", "256"))

# --- episode boundaries -----------------------------------------------------
# A command is "live" only while it is fresh. /cmd_vel is published at 20 Hz
# during a mission and /cmd_duty at 20 Hz by the duty driver, so 0.5 s is ten
# missed messages -- long enough to ride out a hiccup, short enough that a
# crashed commander closes the episode rather than leaving it open forever.
CMD_FRESH_S = float(os.environ.get("FPMS_LEARN_CMD_FRESH_S", "0.5"))
# How long the wire must be at zero before the episode is allowed to close.
# fpms_missions' own STOP_SETTLE_S is the model; this is deliberately a little
# longer so the measurement lands after the executor has taken its own.
SETTLE_S = float(os.environ.get("FPMS_LEARN_SETTLE_S", "0.9"))
# ...and the ticks must ALSO have stopped. This is what puts coast inside the
# measurement instead of outside it.
STILL_S = float(os.environ.get("FPMS_LEARN_STILL_S", "0.5"))
STILL_COUNTS = int(os.environ.get("FPMS_LEARN_STILL_COUNTS", "2"))
# An episode longer than this is a runaway or a stuck commander, not a burst.
# It is closed and flagged rather than dropped -- a runaway is data too.
MAX_EPISODE_S = float(os.environ.get("FPMS_LEARN_MAX_EPISODE_S", "60.0"))
# An MQTT telemetry/residual landing this soon after an episode closes is
# taken to describe that episode. fpms_missions publishes it at the end of
# `_run_one`, immediately after the same settle this recorder is waiting on.
JOIN_WINDOW_S = float(os.environ.get("FPMS_LEARN_JOIN_S", "4.0"))

# Front sector for the LiDAR wall reference, in degrees either side of the
# scanner's zero. Narrow, because a wide sector on an angled wall reports the
# nearest corner of the sector rather than the distance straight ahead.
FRONT_HALF_DEG = 10.0

HEARTBEAT_S = 60.0

# Track values quoted ONLY so the fitter and any reader can see, in the data
# file itself, which candidates were live when it was recorded. Nothing in
# this file divides by any of them.
TRACK_CANDIDATES_MM = {"fpms_rtos_follower.py": 105.0,
                       "fpms_odom_tf.py (geometric)": 170.0,
                       "fpms_drive.py (effective, 1.5x assumed)": 255.0,
                       "older session notes": 271.0}
COUNTS_PER_MM_CANDIDATES = {"hand push, 1000mm tape, motors off": 5.5,
                            "derived from TICKS_PER_REV 1320": 6.0,
                            "tape over 800mm, 2026-08-04, unconfirmed": 14.8,
                            "in the deployed code today (0.16657 mm/tick), "
                            "validated to 2.5% against odom on the M2 run "
                            "2026-08-14": 6.00}

# --- the grid half ----------------------------------------------------------
# How often the prior is written out. Not every episode: an atomic replace is
# three syscalls and an fsync, and the point of this file is that turning it on
# changes nothing about how the rover drives.
PRIOR_WRITE_S = float(os.environ.get("FPMS_LEARN_PRIOR_WRITE_S", "30.0"))
# Ray-casting the free space is the only expensive arithmetic in this process
# (360 bearings x ~39 cells). The LiDAR arrives at ~9.5 Hz; folding every scan
# in would burn a core the mission executor is using. One a second is plenty
# for a belief with a 14 day half-life.
PRIOR_SCAN_HZ = float(os.environ.get("FPMS_LEARN_PRIOR_SCAN_HZ", "1.0"))
# The pose is only good enough to place an obstacle when it is FRESH. A grid
# built on a two-second-old pose during a 200 mm/s crawl is out by 400 mm,
# which is eight cells -- worse than no prior at all.
POSE_FRESH_S = float(os.environ.get("FPMS_LEARN_POSE_FRESH_S", "1.5"))
# How many cells to write into an episode record. The arena is 30 x 24 cells,
# so this is a guard against a bug rather than an expected limit.
MAX_CELLS_PER_EPISODE = 2000

# ------------------------------------------------------------------ imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Int32MultiArray, Bool
    from sensor_msgs.msg import Imu, LaserScan, BatteryState
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    HAVE_ROS, ROS_ERR = True, None
except Exception as e:                                   # pragma: no cover
    HAVE_ROS, ROS_ERR = False, e
    Node = object

try:
    import paho.mqtt.client as mqtt
    HAVE_MQTT, MQTT_ERR = True, None
except Exception as e:                                   # pragma: no cover
    HAVE_MQTT, MQTT_ERR = False, e

# The grid half. Imported SOFT: if it is missing, the drive-constant dataset --
# which is the half that has been wanted for a fortnight -- must still record.
# A recorder that refuses to start because an additive feature is absent is a
# recorder that produces nothing on the day somebody deletes one file.
try:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import fpms_grid_learn as GL
    HAVE_GRID, GRID_ERR = True, None
except Exception as e:                                   # pragma: no cover
    HAVE_GRID, GRID_ERR = False, e
    GL = None


def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] learn_recorder: {msg}",
          flush=True)


def jnum(v, nd=None):
    """A number, or None -- never a NaN and never an inf on the wire.

    Same contract as fpms_missions.jnum, deliberately: a consumer that has
    learned `null means the rover did not know` must not have to learn a
    second convention for this file."""
    try:
        f = float(v)
    except Exception:
        return None
    if not math.isfinite(f):
        return None
    return round(f, nd) if nd is not None else f


def quat_yaw(q):
    """Yaw in radians from a quaternion. Z-axis only; this chassis is planar."""
    try:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)
    except Exception:
        return None


def wrap180(deg):
    return (float(deg) + 180.0) % 360.0 - 180.0


def signed_median4(vals):
    """The SIGNED median of four wheels: mean of the two middle values.

    Not the mean (one slipping wheel drags it) and not the max (fpms_drive
    measured that truncating an 800 mm run, because the rear-right races
    ahead in counts under power). Not abs-then-median either, which is what
    fpms_drive uses -- that one cannot represent a reversal and would report
    a wheel spinning backwards as forward travel. Both are recorded; this is
    the one the fitter uses."""
    s = sorted(float(v) for v in vals)
    return (s[1] + s[2]) / 2.0


# ====================================================================== SINK
class Sink:
    """One NDJSON file per UTC day, flushed after every record.

    Flushed, not buffered: this rover loses power by having its battery
    unplugged, and a dataset that survives only a clean shutdown is a dataset
    that will be empty on the day it matters."""

    def __init__(self, out_dir, echo=False):
        self.dir = out_dir
        self.echo = echo
        self.day = None
        self.part = 0
        self.path = None
        self.fh = None
        self.n = 0
        self.dropped = 0
        self.lock = threading.Lock()
        os.makedirs(self.dir, exist_ok=True)

    # -- disk bounds ------------------------------------------------------
    def _files(self):
        try:
            names = [f for f in os.listdir(self.dir) if f.endswith(".ndjson")]
        except OSError:
            return []
        out = []
        for f in names:
            p = os.path.join(self.dir, f)
            try:
                st = os.stat(p)
            except OSError:
                continue
            out.append((st.st_mtime, st.st_size, p))
        return sorted(out)

    def _free_mb(self):
        try:
            st = os.statvfs(self.dir)
            return st.f_bavail * st.f_frsize / 1e6
        except Exception:
            return float("inf")

    def _enforce_dir_cap(self):
        """Delete the OLDEST files until the directory is under the cap.

        Oldest-first, never newest-first: the run happening right now is the
        one somebody is waiting on, and a cap that ate the live file to make
        room for history would be exactly backwards. The current file is
        excluded outright."""
        files = self._files()
        total = sum(f[1] for f in files)
        cap = MAX_DIR_MB * 1e6
        for mtime, size, p in files:
            if total <= cap:
                break
            if p == self.path:
                continue
            try:
                os.unlink(p)
                total -= size
                log(f"size cap: deleted {os.path.basename(p)} "
                    f"({size/1e6:.1f} MB); dir cap is {MAX_DIR_MB:.0f} MB")
            except OSError:
                break

    def _rotate(self):
        """Open the right file: one per UTC day, split at MAX_FILE_MB.

        The part suffix is chosen by probing rather than remembered, so a
        restart mid-day appends to the part it left off at instead of
        truncating it or starting a third."""
        day = datetime.now(timezone.utc).strftime("%Y%m%d")
        roll = False
        if day != self.day or self.fh is None:
            self.day, self.part, roll = day, 0, True
        elif self.path:
            try:
                if os.path.getsize(self.path) >= MAX_FILE_MB * 1e6:
                    self.part += 1
                    roll = True
            except OSError:
                pass
        if not roll:
            return
        if self.fh:
            try:
                self.fh.close()
            except Exception:
                pass
            self.fh = None
        while True:
            name = (f"runs-{self.day}.ndjson" if self.part == 0
                    else f"runs-{self.day}.{self.part}.ndjson")
            p = os.path.join(self.dir, name)
            try:
                if os.path.getsize(p) >= MAX_FILE_MB * 1e6:
                    self.part += 1
                    continue
            except OSError:
                pass
            break
        self.path = p
        self.fh = open(p, "a", encoding="utf-8")
        log(f"writing {p}")
        self._enforce_dir_cap()

    def write(self, rec):
        line = json.dumps(rec, separators=(",", ":"), sort_keys=False)
        with self.lock:
            try:
                self._rotate()
                # The last line of defence, and it is checked BEFORE the write
                # rather than caught after it: ENOSPC on a filesystem the
                # mission executor is also writing its journal to is a rover
                # problem, not a dataset problem.
                if self._free_mb() < MIN_FREE_MB:
                    self.dropped += 1
                    if self.dropped % 200 == 1:
                        log(f"only {self._free_mb():.0f} MB free (< "
                            f"{MIN_FREE_MB:.0f}) -- dropping records to leave "
                            "the disk to the rover")
                    self._enforce_dir_cap()
                else:
                    self.fh.write(line + "\n")
                    self.fh.flush()
                    os.fsync(self.fh.fileno())
                    self.n += 1
            except OSError as e:
                if e.errno == errno.ENOSPC:
                    log("DISK FULL -- recording stopped, rover unaffected")
                else:
                    log(f"write failed: {e}")
            except Exception as e:
                log(f"write failed: {e}")
        if self.echo:
            print(line, flush=True)

    def close(self):
        with self.lock:
            if self.fh:
                try:
                    self.fh.flush()
                    self.fh.close()
                except Exception:
                    pass
                self.fh = None


# ============================================================ READ-ONLY MQTT
class ReadOnlyMqtt(mqtt.Client if HAVE_MQTT else object):
    """A paho client that CANNOT publish.

    `publish` is the only method paho exposes that puts application data on
    the broker, and it is overridden to raise. There is no LWT either, so
    this process cannot even speak on the topic tree when it dies."""

    def publish(self, *a, **k):
        raise RuntimeError(
            "fpms_learn_recorder is subscribe-only: it must never publish. "
            "If you need a node that commands the rover, write a different "
            "file -- do not remove this guard.")

    def will_set(self, *a, **k):
        raise RuntimeError("fpms_learn_recorder sets no last-will")


class Bus:
    """Subscribe-only MQTT. Delivers mission telemetry to the recorder."""

    def __init__(self, on_msg):
        self.on_msg = on_msg
        self.connected = False
        self.client = None
        if not HAVE_MQTT:
            return
        cid = f"fpms-learn-{socket.gethostname()}-{os.getpid()}"
        try:
            self.client = ReadOnlyMqtt(mqtt.CallbackAPIVersion.VERSION1,
                                       client_id=cid, clean_session=True)
        except (AttributeError, TypeError):
            self.client = ReadOnlyMqtt(client_id=cid, clean_session=True)
        if MQTT_USER:
            self.client.username_pw_set(MQTT_USER, MQTT_PASS)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

    # `telemetry/residual` is kept even though NOTHING PUBLISHES IT on this
    # rover -- the deployed fpms_missions.py has no `_publish_residual` at all.
    # A subscription costs one SUBSCRIBE frame and starts working by itself the
    # day that gap is closed; the residuals actually used come from
    # `events/mission_done` -> `measured[]`.
    #
    # `telemetry/mission_plan` is the PLANNED route and `events/replan` is the
    # obstacle event. Those two plus the pose in `telemetry/mission` are what
    # make planned-vs-driven a subtraction.
    TOPICS = ("events/mission_done", "telemetry/residual",
              "telemetry/mission", "telemetry/mission_plan",
              "events/replan", "events/mission_ack",
              "events/mission_nack", "events/ack", "events/nack",
              "telemetry/status")

    def start(self):
        if not self.client:
            log(f"paho-mqtt missing ({MQTT_ERR}); ROS-only dataset -- the "
                "`asked` values from the mission executor will be absent")
            return
        def run():
            while True:
                try:
                    self.client.connect(BROKER, PORT, keepalive=30)
                    self.client.loop_forever(retry_first_connection=True)
                except Exception as e:
                    self.connected = False
                    log(f"mqtt down ({e}); retrying in 5s")
                    time.sleep(5.0)
        threading.Thread(target=run, daemon=True, name="learn-mqtt").start()

    def _on_connect(self, c, u, f, rc, props=None):
        self.connected = (rc == 0)
        if self.connected:
            for t in self.TOPICS:
                c.subscribe(f"fpms/{THING}/{t}", qos=0)
            log(f"mqtt connected; subscribed to {len(self.TOPICS)} topics")

    def _on_disconnect(self, c, u, rc, props=None, reason=None):
        self.connected = False

    def _on_message(self, c, u, msg):
        try:
            suffix = msg.topic.split(f"fpms/{THING}/", 1)[-1]
            payload = json.loads(msg.payload.decode("utf-8", "replace"))
            self.on_msg(suffix, payload)
        except Exception as e:
            log(f"mqtt decode error on {msg.topic}: {e}")


# ================================================================ THE NODE
class Recorder(Node):
    """Subscribe-only. See the module docstring for the three guards."""

    # ---- GUARD 1 -----------------------------------------------------------
    # THE ONE ALLOWED PUBLISHER, and it is not ours. rclpy's own Node.__init__
    # creates /parameter_events unconditionally (humble, node.py:199) and the
    # constructor cannot be talked out of it. It carries ROS parameter
    # declarations and nothing else -- there is no message on it that any node
    # in this stack acts on, and certainly nothing the motor board reads. It is
    # named explicitly rather than allowed by a "during __init__" window,
    # because a window is a thing that can be re-opened by accident and a name
    # is not.
    ALLOWED_PUBLISHERS = ("/parameter_events", "parameter_events")

    def create_publisher(self, msg_type, topic, *a, **k):
        if topic in self.ALLOWED_PUBLISHERS:
            return super().create_publisher(msg_type, topic, *a, **k)
        raise RuntimeError(
            f"fpms_learn_recorder must never publish ({topic!r} refused). This "
            "node is a witness; giving it an output is how a recorder becomes "
            "a second commander racing the mission executor on /cmd_vel.")

    def __init__(self, sink, echo=False):
        super().__init__(NODE_NAME)
        self.sink = sink
        self.echo = echo
        self.lock = threading.RLock()

        # ---- live sensor state -------------------------------------------
        self.ticks = None            # list[4] raw counts
        self.ticks_t = 0.0
        self.ticks_prev = None
        self.ticks_last_change = 0.0
        self.duty = None             # /wheel_duty, what the board applied
        self.cmd_duty = None         # /cmd_duty, what was asked of it
        self.cmd_duty_t = 0.0
        self.cmd_vel = (0.0, 0.0)
        self.cmd_vel_t = 0.0
        self.odom = None             # (x_m, y_m, yaw_rad, ts)
        self.odom_raw = None
        self.gz = 0.0                # rad/s, raw -- no bias removal here
        self.gz_t = 0.0
        self.batt_v = None
        self.health = None
        self.front_mm = None         # from /scan_lidar, own computation
        self.front_mm_mqtt = None    # from telemetry/mission, the proven one
        self.mission_snap = {}

        # ---- the grid half -------------------------------------------------
        # `arena_pose` is (x_mm, y_mm, heading_deg, wall_ts) off
        # telemetry/mission. It is the SAME pose fpms_missions builds its own
        # occupancy grid on -- there is deliberately no second derivation in
        # this process, because a grid built on different geometry from the one
        # the rover drives puts obstacles where the rover never goes.
        self.arena_pose = None
        self.occ_stats = None        # the executor's OWN occupancy counters
        self.plan = None             # last telemetry/mission_plan
        self.replans = []            # events/replan, newest last
        self.phase = "idle"
        self.run_started = None      # wall clock the phase last left idle
        self.run_episodes = []       # seqs closed during the current run
        self.witness = None          # GL.ScanWitness, or None
        self.prior = None            # GL.GridPrior, or None
        self._prior_scan_t = 0.0
        self._prior_write_t = 0.0
        self.grid_hits_epoch = set()
        self.grid_free_epoch = set()
        if HAVE_GRID:
            try:
                self.witness = GL.ScanWitness()
                self.prior = GL.GridPrior()
                log(f"grid: arena {self.witness.w_mm:.0f}x"
                    f"{self.witness.h_mm:.0f}mm from {self.witness.arena_src}, "
                    f"cell {self.witness.cell_mm:.0f}mm, prior "
                    f"{self.prior.path} ({len(self.prior.cells)} cells known)")
            except Exception as e:
                log(f"grid learning unavailable ({e}); drive dataset only")
                self.witness = self.prior = None
        else:
            log(f"fpms_grid_learn not importable ({GRID_ERR}); "
                "drive dataset only, no learned grid prior")

        # ---- episode state -----------------------------------------------
        self.ep = None
        self.seq = 0
        self.pending = []            # closed episodes awaiting a residual join

        # ---- topic health -------------------------------------------------
        self.seen = {}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        # Every one of these is a SUBSCRIPTION. There is not a publisher in
        # this file, and create_publisher above makes that structural.
        self.create_subscription(Int32MultiArray, "/wheel_ticks", self._on_ticks, qos)
        self.create_subscription(Int32MultiArray, "/wheel_duty", self._on_duty, qos)
        self.create_subscription(Int32MultiArray, "/cmd_duty", self._on_cmd_duty, qos)
        self.create_subscription(Int32MultiArray, "/fpms_health", self._on_health, qos)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, qos)
        self.create_subscription(Imu, "/imu", self._on_imu, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.create_subscription(Odometry, "/odom_raw", self._on_odom_raw, qos)
        self.create_subscription(BatteryState, "/battery", self._on_batt, qos)
        self.create_subscription(LaserScan, "/scan_lidar", self._on_scan, qos)

        self.create_timer(0.05, self._tick)          # 20 Hz episode machine
        self.create_timer(1.0, self._audit)          # GUARD 2
        self.create_timer(HEARTBEAT_S, self._heartbeat)
        self.create_timer(PRIOR_WRITE_S, self._prior_tick)
        self.t_start = time.time()
        self._write_header()
        log("up: 10 subscriptions, 0 publishers, MQTT subscribe-only")

    # ---- GUARD 2 -----------------------------------------------------------
    def _audit(self):
        """A recorder that has grown a publisher is a recorder that can drive
        the rover. Rather than log and continue, this kills the process: a
        loud absence of telemetry is safer than a quiet second commander.

        Independent of GUARD 1 on purpose. This one inspects what the node
        ACTUALLY HOLDS rather than trusting the door it came through, so a
        publisher created by some future rclpy internal that never calls
        `create_publisher` is still caught."""
        names = [getattr(p, "topic_name", "?")
                 for p in list(getattr(self, "publishers", []))]
        names = [n for n in names if n not in self.ALLOWED_PUBLISHERS]
        if names:
            log(f"FATAL: publishers exist on a subscribe-only node: {names}")
            self.sink.write({"rec": "fatal", "ts": time.time(),
                             "why": "publisher_created", "topics": names})
            os._exit(3)

    # -------------------------------------------------------------- callbacks
    def _mark(self, key):
        self.seen[key] = self.seen.get(key, 0) + 1

    def _on_ticks(self, m):
        with self.lock:
            now = time.monotonic()
            v = [int(x) for x in m.data][:4]
            if len(v) < 4:
                return
            if self.ticks is None or max(abs(a - b) for a, b
                                         in zip(v, self.ticks)) >= STILL_COUNTS:
                self.ticks_last_change = now
            self.ticks, self.ticks_t = v, now
            self._mark("/wheel_ticks")

    def _on_duty(self, m):
        with self.lock:
            self.duty = [int(x) for x in m.data][:4]
            self._mark("/wheel_duty")
            if self.ep is not None:
                self.ep["duty_samples"].append(list(self.duty))

    def _on_cmd_duty(self, m):
        with self.lock:
            self.cmd_duty = [int(x) for x in m.data][:4]
            self.cmd_duty_t = time.monotonic()
            self._mark("/cmd_duty")
            if self.ep is not None:
                self.ep["cmd_duty_samples"].append(list(self.cmd_duty))

    def _on_cmd_vel(self, m):
        with self.lock:
            self.cmd_vel = (float(m.linear.x), float(m.angular.z))
            self.cmd_vel_t = time.monotonic()
            self._mark("/cmd_vel")
            if self.ep is not None:
                self.ep["cmd_vel_samples"].append(
                    [round(self.cmd_vel[0], 4), round(self.cmd_vel[1], 4)])

    def _on_health(self, m):
        with self.lock:
            self.health = [int(x) for x in m.data]
            self._mark("/fpms_health")

    def _on_imu(self, m):
        with self.lock:
            now = time.monotonic()
            gz = float(m.angular_velocity.z)
            # Integrate RAW, with no bias subtraction. Bias removal belongs
            # in the fitter, where it can be estimated from the at-rest
            # stretches this file also records -- doing it here would bake a
            # calibration into the raw data and make it unrecoverable.
            if self.ep is not None and self.gz_t:
                dt = now - self.gz_t
                if 0.0 < dt < 0.5:
                    self.ep["gyro_yaw_rad"] += gz * dt
                    self.ep["gyro_dt_s"] += dt
                self.ep["gz_n"] += 1
                if gz != 0.0:
                    self.ep["gz_nonzero"] += 1
                self.ep["gz_absmax"] = max(self.ep["gz_absmax"], abs(gz))
            self.gz, self.gz_t = gz, now
            self._mark("/imu")

    def _take_odom(self, msg):
        p = msg.pose.pose
        return (float(p.position.x), float(p.position.y),
                quat_yaw(p.orientation), time.monotonic())

    def _on_odom(self, m):
        with self.lock:
            self.odom = self._take_odom(m)
            self._mark("/odom")

    def _on_odom_raw(self, m):
        with self.lock:
            self.odom_raw = self._take_odom(m)
            self._mark("/odom_raw")

    def _on_batt(self, m):
        with self.lock:
            v = float(m.voltage)
            self.batt_v = v if math.isfinite(v) and v > 0.5 else None
            self._mark("/battery")

    def _on_scan(self, m):
        """Median range in a narrow forward sector, in mm.

        This is the ONLY encoder-independent length reference available
        without an operator holding a tape, so it is worth the arithmetic.
        It is a MEDIAN over the sector, not a min: a min reports whatever
        speck of dust the scanner caught, and one such reading in a
        before/after pair invents hundreds of millimetres of travel."""
        try:
            n = len(m.ranges)
            if not n:
                return
            half = math.radians(FRONT_HALF_DEG)
            vals = []
            for i, r in enumerate(m.ranges):
                a = m.angle_min + i * m.angle_increment
                if abs(wrap180(math.degrees(a))) > FRONT_HALF_DEG * 1.0:
                    continue
                if not math.isfinite(r):
                    continue
                if r < m.range_min or r > m.range_max or r <= 0.0:
                    continue
                vals.append(r * 1000.0)
            with self.lock:
                self.front_mm = statistics.median(vals) if len(vals) >= 3 else None
                self._mark("/scan_lidar")
            _ = half
            self._grid_scan(m)
        except Exception:
            pass

    def _grid_scan(self, m):
        """Fold one scan into the grid witness and the cross-run prior.

        THROTTLED, and gated on a FRESH POSE. Two independent reasons, both of
        which produce a wrong map rather than a slow one if ignored:

          * the LiDAR arrives at ~9.5 Hz and ray-casting 360 bearings across
            ~39 cells each is the only real arithmetic in this process. This
            node shares a core with the thing it is recording, and the whole
            claim is that turning it on changes nothing.
          * `telemetry/mission` carries the pose at 2 Hz. A scan folded in
            against a two-second-old pose during a 220 mm/s crawl lands ~440 mm
            -- nearly nine cells -- from where it belongs, and a prior that
            learns smeared obstacles is worse than an empty one.
        """
        if self.witness is None:
            return
        now = time.monotonic()
        if PRIOR_SCAN_HZ > 0 and (now - self._prior_scan_t) < (1.0 / PRIOR_SCAN_HZ):
            return
        with self.lock:
            pose = self.arena_pose
        if not pose or (time.time() - pose[3]) > POSE_FRESH_S:
            return
        self._prior_scan_t = now
        try:
            rmax = float(getattr(m, "range_max", 6.0)) or 6.0
            hits, frees = self.witness.integrate(
                list(m.ranges), (pose[0], pose[1], pose[2]),
                range_max_m=rmax, now=now)
        except Exception as e:
            log(f"grid integrate failed: {e}")
            return
        with self.lock:
            self.grid_hits_epoch |= hits
            self.grid_free_epoch |= frees
            self._mark("grid_scans")
            if self.ep is not None:
                self.ep["grid_hits"] |= hits
                self.ep["grid_free"] |= frees
        if self.prior is not None:
            try:
                self.prior.observe(hits, frees)
            except Exception as e:
                log(f"grid prior observe failed: {e}")

    # ------------------------------------------------------------------ MQTT
    def on_mqtt(self, suffix, payload):
        deferred = None
        with self.lock:
            now = time.time()
            if suffix == "telemetry/mission":
                self.mission_snap = payload
                f = payload.get("front_mm")
                self.front_mm_mqtt = jnum(f) if f is not None else None
                self.occ_stats = payload.get("occupancy")
                x, y, h = (payload.get("x_mm"), payload.get("y_mm"),
                           payload.get("heading_deg"))
                # OMITTED, not zero-filled, is fpms_missions' own convention
                # when it has no odometry -- so a missing key here means "the
                # rover did not know where it was", and folding a scan in at
                # (0,0) would write the whole room into the corner cell.
                if x is not None and y is not None and h is not None:
                    self.arena_pose = (float(x), float(y), float(h), now)
                ph = payload.get("phase")
                if ph and ph != self.phase:
                    was = self.phase
                    self.phase = ph
                    if was == "idle" and ph != "idle":
                        self.run_started = now
                        self.run_episodes = []
                        if self.prior is not None:
                            self.prior.runs += 1
                return
            if suffix == "telemetry/mission_plan":
                self.plan = payload
                self.sink.write({"rec": "plan", "ts": round(now, 3),
                                 "mission": payload.get("mission"),
                                 "committed": bool(payload.get("committed")),
                                 "payload": payload})
                return
            if suffix == "events/replan":
                # THE OBSTACLE EVENT. This is the only message on the bus that
                # says the rover met something real: `reason` is the guard that
                # tripped, `occupancy` is what the grid believed at that moment,
                # and `waypoints` is the detour it chose. All three are needed
                # to tell a real obstacle from the failure mode the session doc
                # warns about -- a reroute firing with `cells: 0`, which means
                # the cone saw something the grid never confirmed.
                self.replans.append({"ts": now, "payload": payload})
                self.replans = self.replans[-64:]
                if self.ep is not None:
                    self.ep["replans"].append({"ts": now, "payload": payload})
                self.sink.write({"rec": "replan", "ts": round(now, 3),
                                 "seq_open": (self.ep or {}).get("seq"),
                                 "payload": payload})
                return
            if suffix == "telemetry/residual":
                self._join_residual(payload, now)
                self.sink.write({"rec": "residual", "ts": round(now, 3),
                                 "payload": payload})
                return
            if suffix == "events/mission_done":
                self.sink.write({"rec": "mission_done", "ts": round(now, 3),
                                 "payload": payload})
                deferred = payload
            else:
                self.sink.write({"rec": "event", "ts": round(now, 3),
                                 "topic": suffix, "payload": payload})
        if deferred is not None:
            self._join_mission_done(deferred, time.time())

    def _join_mission_done(self, payload, now):
        """Attach `measured[]` to the episodes of the run that just ended.

        THIS IS WHERE THE COMMANDED-VS-MEASURED RESIDUAL ACTUALLY COMES FROM.
        `telemetry/residual` is documented, is advertised on six ROS topics, is
        subscribed by two files, and is published by NOTHING -- the deployed
        fpms_missions.py has no `_publish_residual`. `events/mission_done`
        carries the same numbers for the whole run in one message, and this
        rover really does send it.

        THE JOIN IS BY ORDER, AND ORDER IS A WEAKER KEY THAN A TIMESTAMP, so
        it is stated rather than assumed. `measured[]` is the executed segments
        in the order they ran; `run_episodes` is the episodes this recorder
        closed since the phase left `idle`, in the order they closed. When the
        two counts agree the pairing is exact and `confident` is true. When they
        do not -- an episode too small to trip the detector, or a recorder
        started mid-mission -- NOTHING IS PAIRED. A wrong residual attached to
        the wrong segment is worse than no residual: it would land in the
        fitter as a real measurement and there would be no way to tell.
        """
        with self.lock:
            eps = list(self.run_episodes)
            started = self.run_started
        measured = payload.get("measured") or []
        exact = (len(eps) == len(measured)) and bool(eps)
        rec = {"rec": "residual_join", "ts": round(now, 3),
               "source": "events/mission_done.measured[]",
               "mission": payload.get("name"),
               "outcome": payload.get("outcome"),
               "run_started": round(started, 3) if started else None,
               "episodes_in_run": len(eps), "segments_reported": len(measured),
               "confident": exact,
               "join": "positional; the k-th episode closed during this run is "
                       "the k-th executed segment",
               "pairs": []}
        if exact:
            for seq, mrec in zip(eps, measured):
                asked = jnum(mrec.get("asked"), 2)
                meas = jnum(mrec.get("measured"), 2)
                rec["pairs"].append({
                    "seq": seq, "kind": mrec.get("kind"),
                    "asked": asked, "executor_measured": meas,
                    "executor_residual": (None if (asked is None or meas is None)
                                          else jnum(meas - asked, 2)),
                    "retrace": mrec.get("retrace"), "dock": mrec.get("dock"),
                    "reason": mrec.get("reason"),
                    "elapsed_s": jnum(mrec.get("elapsed_s"), 2)})
        else:
            rec["why_unmatched"] = (
                f"{len(eps)} episode(s) closed during the run but the executor "
                f"reported {len(measured)} executed segment(s); a positional "
                "join would be a guess, so nothing was paired. The usual cause "
                "is a segment shorter than the episode detector's threshold, "
                "or this recorder having started mid-mission.")
        self.sink.write(rec)
        if self.prior is not None:
            try:
                self.prior.commit()
                self.prior.save()
                log(f"learned grid prior: {len(self.prior.cells)} cell(s) "
                    f"after {self.prior.runs} run(s) -> {self.prior.path}")
            except Exception as e:
                log(f"grid prior save failed: {e}")

    def _join_residual(self, payload, now):
        """Attach the executor's `asked` to the episode it describes.

        fpms_missions publishes telemetry/residual at the end of `_run_one`,
        immediately after the same full stop and settle this recorder waits
        on, so the newest closed episode inside JOIN_WINDOW_S is the right
        one. The join is written as its OWN record rather than by rewriting
        the episode line, because an append-only log that is never rewritten
        cannot be corrupted by a power cut mid-update."""
        best = None
        for ep in reversed(self.pending):
            if now - ep["closed_wall"] <= JOIN_WINDOW_S:
                best = ep
                break
        rec = {"rec": "join", "ts": round(now, 3),
               "seq": best["seq"] if best else None,
               "matched": best is not None,
               "dt_s": round(now - best["closed_wall"], 3) if best else None,
               "kind": payload.get("kind"),
               "asked": jnum(payload.get("target"), 2),
               "executor_measured": jnum(payload.get("measured"), 2),
               "executor_residual": jnum(payload.get("residual"), 2),
               "lateral_mm": jnum(payload.get("lateral_mm"), 2),
               "dock": payload.get("dock"), "retrace": payload.get("retrace"),
               "reason": payload.get("reason")}
        if best is None:
            rec["why_unmatched"] = (
                "no episode closed within the join window -- either the "
                "recorder started mid-mission, or the motion was too small "
                "to trip the episode detector")
        self.sink.write(rec)

    # ------------------------------------------------- the episode machine
    def _commanded(self, now):
        """Is a non-zero motion command live on the wire right now?

        Both command surfaces are checked. /cmd_vel is what fpms_missions
        writes; /cmd_duty is what fpms_drive and the duty driver write, and a
        rover driven by the second while only the first is watched would
        produce an empty dataset with no error anywhere."""
        vx, wz = self.cmd_vel
        if (now - self.cmd_vel_t) <= CMD_FRESH_S and (vx != 0.0 or wz != 0.0):
            return True
        if (self.cmd_duty and (now - self.cmd_duty_t) <= CMD_FRESH_S
                and any(d != 0 for d in self.cmd_duty)):
            return True
        return False

    def _snapshot(self):
        now = time.time()
        p = self.arena_pose
        # The ARENA pose, and its age. Age is recorded rather than the pose
        # being dropped when stale, because "the rover was at (1322,178) but
        # that reading was 4 s old" is a usable fact and a silent None is not.
        pose = None
        if p:
            pose = {"x_mm": jnum(p[0], 1), "y_mm": jnum(p[1], 1),
                    "heading_deg": jnum(p[2], 1),
                    "age_s": jnum(now - p[3], 2)}
        return {
            "ticks": list(self.ticks) if self.ticks else None,
            "odom": list(self.odom[:3]) if self.odom else None,
            "odom_raw": list(self.odom_raw[:3]) if self.odom_raw else None,
            "front_mm": jnum(self.front_mm, 1),
            "front_mm_mqtt": jnum(self.front_mm_mqtt, 1),
            "batt_v": jnum(self.batt_v, 2),
            "arena_pose": pose,
            "occupancy": self.occ_stats,
            "grid_believed": self._believed_cells(),
            "wall": round(now, 3),
        }

    def _believed_cells(self):
        """The mirror's currently-believed obstacle cells, as [x_mm, y_mm].

        CENTRES IN MILLIMETRES, not indices. An index is only meaningful
        alongside the cell size that produced it, and this dataset outlives the
        config that set it -- the arena was resized by 200 mm on the day this
        was written."""
        if self.witness is None:
            return None
        try:
            cells = sorted(self.witness.believed())[:MAX_CELLS_PER_EPISODE]
            return [[int(round(v)) for v in self.witness.centre_of(*c)]
                    for c in cells]
        except Exception:
            return None

    def _tick(self):
        with self.lock:
            now = time.monotonic()
            cmd = self._commanded(now)

            if self.ep is None:
                if cmd and self.ticks is not None:
                    self._open(now)
                return

            ep = self.ep
            if cmd:
                ep["last_cmd"] = now
            ep["dur"] = now - ep["t0"]

            if ep["dur"] >= MAX_EPISODE_S:
                self._close(now, "max_episode_s")
                return
            quiet = now - ep["last_cmd"]
            still = now - self.ticks_last_change
            if quiet >= SETTLE_S and still >= STILL_S:
                self._close(now, "settled")

    def _open(self, now):
        self.seq += 1
        snap = self._snapshot()
        self.ep = {
            "seq": self.seq, "t0": now, "last_cmd": now, "dur": 0.0,
            "start": snap,
            "cmd_vel_samples": [], "cmd_duty_samples": [], "duty_samples": [],
            "gyro_yaw_rad": 0.0, "gyro_dt_s": 0.0,
            "gz_n": 0, "gz_nonzero": 0, "gz_absmax": 0.0,
            # Copied at open, because by the time the episode closes the
            # executor has moved on to the next segment and the indices would
            # name the wrong one.
            "mission": self.mission_snap.get("mission")
                       or self.mission_snap.get("name"),
            "phase": self.mission_snap.get("phase"),
            "leg_i": self.mission_snap.get("leg_i"),
            "segment_i": self.mission_snap.get("segment_i"),
            "segment_kind": self.mission_snap.get("segment_kind"),
            "health0": list(self.health) if self.health else None,
            # Grid evidence gathered WHILE THIS EPISODE WAS RUNNING, as opposed
            # to the before/after snapshots. This is what the offline rebuild
            # replays, so the dataset -- not this process's memory -- is the
            # thing of record for the learned prior.
            "grid_hits": set(),
            "grid_free": set(),
            "replans": [],
            # The plan in force when the wheels started. Copied at OPEN because
            # a replan mid-episode replaces it, and the question this answers is
            # "what was it trying to do", not "what did it decide afterwards".
            "plan_at_open": self._plan_digest(),
        }

    def _plan_digest(self):
        """The planned route, small enough to carry on every episode.

        The full telemetry/mission_plan is written once as its own `plan`
        record; repeating a 40-waypoint polyline on every episode would triple
        the dataset to say the same thing. What is kept here is the identity of
        the plan and the geometry needed for planned-vs-driven."""
        p = self.plan
        if not p:
            return None
        wps = p.get("waypoints") or []
        return {"mission": p.get("mission"),
                "committed": bool(p.get("committed")),
                "distance_mm": jnum(p.get("distance_mm"), 1),
                "legs_n": p.get("legs_n"),
                "segments_n": len(p.get("segments") or []),
                "waypoints_n": len(wps),
                "target": p.get("target"),
                "from": p.get("from"),
                "planner_note": p.get("note") or p.get("planner_note"),
                # The polyline, thinned to the vertices: those ARE the plan.
                "waypoints": [[jnum(w.get("x_mm"), 1), jnum(w.get("y_mm"), 1),
                               w.get("kind")] for w in wps[:64]]}

    def _close(self, now, why):
        ep = self.ep
        self.ep = None
        end = self._snapshot()
        rec = self._build(ep, end, now, why)
        self.sink.write(rec)
        ep["closed_wall"] = time.time()
        self.pending.append(ep)
        # Bounded: the join only ever looks back JOIN_WINDOW_S, so anything
        # older is unreachable and holding it is a slow leak on a node that
        # is meant to run for weeks.
        cut = time.time() - max(JOIN_WINDOW_S * 4, 30.0)
        self.pending = [e for e in self.pending if e["closed_wall"] >= cut][-64:]
        # The run's episode list, which is the key events/mission_done joins on.
        # Bounded: a mission that somehow never ends must not grow this without
        # limit, and 512 segments is already an order of magnitude past the
        # 6-12 a real mission produces.
        self.run_episodes.append(ep["seq"])
        self.run_episodes = self.run_episodes[-512:]
        if self.prior is not None:
            # Commit at the episode boundary as well as on the epoch timer: an
            # episode IS a look, and closing one is the natural moment to bank
            # what it saw.
            try:
                self.prior.episodes += 1
                self.prior.commit()
            except Exception as e:
                log(f"grid prior commit failed: {e}")
        d = rec["measured"]["dist_ticks_median_mm_at_5p5"]
        log(f"seq {ep['seq']}: {rec['commanded']['duty_peak']} duty, "
            f"{rec['duration_s']:.2f}s -> {rec['measured']['ticks_median']:+.0f} "
            f"counts ({d if d is None else round(d)}mm @5.5) [{why}]")

    def _build(self, ep, end, now, why):
        t0, t1 = ep["start"]["ticks"], end["ticks"]
        per_wheel = None
        med = None
        med_abs = None
        diff_front = diff_all = None
        if t0 and t1 and len(t0) == 4 and len(t1) == 4:
            per_wheel = [t1[i] - t0[i] for i in range(4)]
            med = signed_median4(per_wheel)
            s = sorted(abs(v) for v in per_wheel)
            med_abs = (s[1] + s[2]) / 2.0
            # RIGHT minus LEFT, in raw counts. Positive = right side ran
            # further = the chassis turned LEFT (CCW). Front pair and all four
            # are both kept: fpms_drive found the rear pair injects phantom
            # rotation under power (a floor run where all four gave 54 deg of
            # "drift" and the front pair gave the 7 deg the operator actually
            # saw), and which pair to trust is itself a thing to be fitted.
            diff_front = per_wheel[1] - per_wheel[0]
            diff_all = ((per_wheel[1] + per_wheel[3])
                        - (per_wheel[0] + per_wheel[2])) / 2.0

        def pose_delta(a, b):
            if not a or not b:
                return None
            dx_mm = (b[0] - a[0]) * 1000.0
            dy_mm = (b[1] - a[1]) * 1000.0
            out = {"dx_mm": jnum(dx_mm, 2), "dy_mm": jnum(dy_mm, 2),
                   "dist_mm": jnum(math.hypot(dx_mm, dy_mm), 2)}
            if a[2] is not None and b[2] is not None:
                out["dyaw_deg"] = jnum(
                    wrap180(math.degrees(b[2] - a[2])), 3)
            return out

        cds = ep["cmd_duty_samples"] or ep["duty_samples"]
        duty_peak = None
        duty_signed = None
        if cds:
            flat = [abs(v) for s_ in cds for v in s_]
            duty_peak = max(flat) if flat else None
            # The per-wheel duty most often commanded, sign included. On this
            # firmware POSITIVE DUTY DRIVES ALL FOUR WHEELS BACKWARD until
            # MOTORn_INV is reflashed (measured on v3), so the sign here is
            # load-bearing and must not be discarded.
            try:
                duty_signed = [round(statistics.median(
                    [s_[i] for s_ in cds if len(s_) > i]), 1) for i in range(4)]
            except Exception:
                duty_signed = None
        vxs = [s_[0] for s_ in ep["cmd_vel_samples"]]
        wzs = [s_[1] for s_ in ep["cmd_vel_samples"]]

        front0 = ep["start"].get("front_mm")
        front1 = end.get("front_mm")
        wall_closure = None
        if front0 is not None and front1 is not None:
            # Positive = the rover got CLOSER to whatever is ahead, i.e. moved
            # forward. Only meaningful when the thing ahead is a flat wall the
            # rover drove straight at; the fitter gates on that with the
            # heading change and refuses the pair otherwise.
            wall_closure = jnum(front0 - front1, 1)

        return {
            "rec": "segment",
            "schema": SCHEMA,
            "seq": ep["seq"],
            "ts": ep["start"]["wall"],
            "ts_end": end["wall"],
            "duration_s": jnum(now - ep["t0"], 3),
            "closed_by": why,
            "context": {"mission": ep["mission"], "phase": ep["phase"],
                        "leg_i": ep["leg_i"], "segment_i": ep["segment_i"],
                        "segment_kind": ep["segment_kind"]},
            "commanded": {
                "duty_peak": duty_peak,
                "duty_per_wheel_median": duty_signed,
                "duty_n": len(cds),
                "cmd_vel_vx_median": jnum(statistics.median(vxs), 4) if vxs else None,
                "cmd_vel_wz_median": jnum(statistics.median(wzs), 4) if wzs else None,
                "cmd_vel_n": len(vxs),
                # The commanded pulse length, which is what MIN_PULSE_S is
                # about. Measured from first non-zero command to last, NOT
                # including the settle -- so a burst too short to break
                # stiction shows up here as a real duration with zero counts.
                "pulse_s": jnum(ep["last_cmd"] - ep["t0"], 3),
            },
            "measured": {
                "ticks_start": t0, "ticks_end": t1,
                "ticks_per_wheel": per_wheel,
                "ticks_median": jnum(med, 1),
                "ticks_median_abs": jnum(med_abs, 1),
                "ticks_mean": jnum(sum(per_wheel) / 4.0, 1) if per_wheel else None,
                "ticks_spread": (max(per_wheel) - min(per_wheel)) if per_wheel else None,
                "tick_diff_front_counts": jnum(diff_front, 1),
                "tick_diff_all_counts": jnum(diff_all, 1),
                # CONVENIENCE ONLY, and named so nobody mistakes it for a
                # measurement: the fitter never reads this field.
                "dist_ticks_median_mm_at_5p5": jnum(med / 5.5, 1) if med is not None else None,
            },
            "gyro": {
                # Currently 0.0 on every sample -- see the module docstring.
                # Recorded so the dataset becomes turn-capable the day the
                # firmware deadband is fixed.
                "yaw_deg": jnum(math.degrees(ep["gyro_yaw_rad"]), 4),
                "integrated_s": jnum(ep["gyro_dt_s"], 3),
                "samples": ep["gz_n"],
                "nonzero_samples": ep["gz_nonzero"],
                "absmax_radps": jnum(ep["gz_absmax"], 5),
                "alive": bool(ep["gz_nonzero"] > 0),
            },
            "odom": {
                "start": ep["start"]["odom"], "end": end["odom"],
                "delta": pose_delta(ep["start"]["odom"], end["odom"]),
                "start_raw": ep["start"]["odom_raw"], "end_raw": end["odom_raw"],
                "delta_raw": pose_delta(ep["start"]["odom_raw"], end["odom_raw"]),
                "note": "CIRCULAR reference -- integrated by the board from "
                        "these same encoders; not usable to fit counts/mm",
            },
            "lidar": {
                "front_mm_start": front0, "front_mm_end": front1,
                "front_closure_mm": wall_closure,
                "front_mm_mqtt_start": ep["start"].get("front_mm_mqtt"),
                "front_mm_mqtt_end": end.get("front_mm_mqtt"),
                "note": "INDEPENDENT of the encoders; valid as a length "
                        "reference only for a straight run at a flat wall",
            },
            "battery": {"v_start": ep["start"].get("batt_v"),
                        "v_end": end.get("batt_v")},
            "health": {"start": ep["health0"],
                       "end": list(self.health) if self.health else None},
            "grid": self._grid_block(ep, end),
            "route": self._route_block(ep, end),
            # Every reroute that fired inside this episode. Empty on a clean
            # run, which is the answer to "did the rover meet anything".
            "obstacles": [{"ts": round(r["ts"], 3),
                           "reason": (r["payload"] or {}).get("reason"),
                           "replan_i": (r["payload"] or {}).get("replan_i"),
                           "occupancy": (r["payload"] or {}).get("occupancy"),
                           "waypoints": [[jnum(w.get("x_mm"), 1),
                                          jnum(w.get("y_mm"), 1)]
                                         for w in ((r["payload"] or {})
                                                   .get("waypoints") or [])],
                           "relaxed": (r["payload"] or {}).get("relaxed"),
                           "plan_cost_mm": (r["payload"] or {}).get("plan_cost_mm")}
                          for r in ep["replans"]],
        }

    def _grid_block(self, ep, end):
        """What the arena looked like, and whether the mirror can be believed.

        `hit_cells`/`free_cells` are the evidence gathered DURING the episode,
        as cell indices, and they are what `fpms_grid_learn --rebuild` replays.
        `believed_start`/`believed_end` are millimetre centres, for a human or
        a plot.

        `agreement` is the part that matters. This process re-derives the grid
        from /scan_lidar and the pose off telemetry/mission; fpms_missions
        derives its own from the same two inputs and publishes the COUNT in
        `occupancy`. Those counts should track. When they do not, this mirror is
        wrong -- a bearing convention, a stale pose, a different max range --
        and every cell in this record is suspect. Recording the disagreement is
        the only way anybody finds out; a mirror that never checks itself is
        just a second opinion nobody asked for."""
        if self.witness is None:
            return None
        mine = end.get("grid_believed")
        theirs = (end.get("occupancy") or {})
        their_obs = theirs.get("obstacle_cells")
        their_all = theirs.get("cells")
        agree = None
        if mine is not None and their_all is not None:
            # Compared against `cells` (wall + obstacle), because this mirror
            # does not make the wall/obstacle split -- WALL_BAND_MM belongs to
            # the executor's `points()` and duplicating it here would be a
            # third place to get it wrong.
            agree = {"mirror_cells": len(mine), "executor_cells": their_all,
                     "executor_obstacle_cells": their_obs,
                     "delta": len(mine) - int(their_all)}
        return {
            "cell_mm": int(round(self.witness.cell_mm)),
            "arena_mm": [int(round(self.witness.w_mm)),
                         int(round(self.witness.h_mm))],
            "arena_src": self.witness.arena_src,
            "believed_start": ep["start"].get("grid_believed"),
            "believed_end": mine,
            "hit_cells": sorted([list(c) for c in ep["grid_hits"]])[:MAX_CELLS_PER_EPISODE],
            "free_cells": sorted([list(c) for c in ep["grid_free"]])[:MAX_CELLS_PER_EPISODE],
            "scans_folded": self.witness.scans,
            "executor_occupancy_start": ep["start"].get("occupancy"),
            "executor_occupancy_end": end.get("occupancy"),
            "agreement": agree,
            "note": "hit_cells/free_cells are (ix,iy) on cell_mm; "
                    "believed_* are cell CENTRES in arena mm. This is a MIRROR "
                    "of fpms_missions.OccupancyGrid, not a reading of it -- "
                    "check `agreement` before trusting it.",
        }

    def _route_block(self, ep, end):
        """Planned versus driven, as a subtraction.

        The planned leg is the next waypoint of the plan in force when the
        wheels started; the driven leg is the arena pose actually reached. The
        pose comes from telemetry/mission at 2 Hz, so `age_s` on either end is
        part of the answer and is carried rather than smoothed away."""
        a = (ep["start"].get("arena_pose") or None)
        b = (end.get("arena_pose") or None)
        driven = None
        if a and b and a.get("x_mm") is not None and b.get("x_mm") is not None:
            dx = b["x_mm"] - a["x_mm"]
            dy = b["y_mm"] - a["y_mm"]
            dh = None
            if a.get("heading_deg") is not None and b.get("heading_deg") is not None:
                dh = jnum(wrap180(b["heading_deg"] - a["heading_deg"]), 2)
            driven = {"dx_mm": jnum(dx, 1), "dy_mm": jnum(dy, 1),
                      "dist_mm": jnum(math.hypot(dx, dy), 1),
                      "dyaw_deg": dh,
                      "start_age_s": a.get("age_s"), "end_age_s": b.get("age_s")}
        return {"planned": ep.get("plan_at_open"),
                "pose_start": a, "pose_end": b, "driven": driven,
                "note": "planned is the route in force when this episode "
                        "opened; driven is the arena pose delta actually "
                        "reached. Both are dead reckoning unless "
                        "telemetry/mission said pose_source=slam_anchored."}

    # -------------------------------------------------------------- bookkeeping
    def _write_header(self):
        self.sink.write({
            "rec": "header", "schema": SCHEMA, "ts": round(time.time(), 3),
            "node": NODE_NAME, "thing": THING,
            "host": socket.gethostname(), "pid": os.getpid(),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "rmw": os.environ.get("RMW_IMPLEMENTATION"),
            # THE ONE SETTING THAT DECIDES WHETHER THIS DATASET EXISTS. Fast
            # DDS shared memory is dead across process eras on this Pi:
            # discovery succeeds, every topic is advertised, and NO DATA FLOWS.
            # It is what made /scan_lidar "not deliver" on 2026-08-07 while its
            # producer logged 9.5 Hz with zero drops. A null here means this
            # recorder is probably about to write a file full of nothing.
            "fastdds_profile": os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"),
            "out_dir": OUT_DIR,
            "size_caps": {"max_file_mb": MAX_FILE_MB, "max_dir_mb": MAX_DIR_MB,
                          "min_free_mb": MIN_FREE_MB},
            "grid": ({"enabled": True,
                      "arena_mm": [self.witness.w_mm, self.witness.h_mm],
                      "arena_src": self.witness.arena_src,
                      "cell_mm": self.witness.cell_mm,
                      "prior_path": self.prior.path if self.prior else None,
                      "prior_cells_at_start": (len(self.prior.cells)
                                               if self.prior else None),
                      "occ_min_hits": GL.OCC_MIN_HITS,
                      "occ_ttl_s": GL.OCC_TTL_S,
                      "lidar_rotation_sign": GL.LIDAR_ROTATION_SIGN,
                      "lidar_zero_offset_deg": GL.LIDAR_ZERO_OFFSET_DEG}
                     if self.witness else
                     {"enabled": False, "why": str(GRID_ERR)}),
            "episode_rules": {"cmd_fresh_s": CMD_FRESH_S,
                              "settle_s": SETTLE_S, "still_s": STILL_S,
                              "still_counts": STILL_COUNTS,
                              "max_episode_s": MAX_EPISODE_S,
                              "join_window_s": JOIN_WINDOW_S},
            "wheel_index": {"0": "front_left", "1": "front_right",
                            "2": "rear_left", "3": "rear_right"},
            "disputed_constants": {
                "counts_per_mm": COUNTS_PER_MM_CANDIDATES,
                "effective_track_mm": TRACK_CANDIDATES_MM,
                "note": "recorded for provenance; NOTHING in this file "
                        "divides by any of them",
            },
            "known_faults": [
                "/imu angular_velocity is identically 0.0 on this firmware "
                "build -- every turn in this dataset measures 0 degrees and "
                "turn-related parameters are UNIDENTIFIABLE until it is fixed",
                "positive duty drives all four wheels BACKWARD on v3 until "
                "MOTORn_INV is reflashed",
                "the rear-right wheel slips ~24% under power against its "
                "neighbours (fpms_drive.py, floor run 2026-08-06)",
            ],
        })

    def _prior_tick(self):
        """Persist the cross-run prior. Cheap, bounded, and never on the path
        of anything the rover is doing.

        Written even when nothing changed, because `updated` is what the decay
        is measured from and a file whose timestamp stops moving would keep
        re-applying the same forgetting on every load."""
        if self.prior is None:
            return
        try:
            self.prior.commit()
            self.prior.save()
        except Exception as e:
            log(f"grid prior write failed: {e}")

    def _heartbeat(self):
        with self.lock:
            snap = dict(self.seen)
            self.seen = {}
        self.sink.write({
            "rec": "health", "ts": round(time.time(), 3),
            "uptime_s": round(time.time() - self.t_start, 1),
            "records_written": self.sink.n,
            "episodes": self.seq,
            "episode_open": self.ep is not None,
            "mqtt": bool(getattr(self, "_bus_ok", False)),
            "records_dropped_low_disk": self.sink.dropped,
            "phase": self.phase,
            "arena_pose_age_s": (None if not self.arena_pose else
                                 round(time.time() - self.arena_pose[3], 1)),
            "grid_prior_cells": (len(self.prior.cells) if self.prior else None),
            "grid_believed_now": (len(self.witness.believed())
                                  if self.witness else None),
            "executor_occupancy": self.occ_stats,
            # Counts over the heartbeat interval, so a topic that died is
            # visible IN THE DATASET rather than only in a journal nobody
            # kept. A run with `/wheel_ticks: 0` is a run the fitter drops.
            "topic_counts": snap,
            "hz": {k: round(v / HEARTBEAT_S, 2) for k, v in snap.items()},
        })


# ====================================================================== MAIN
def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    echo = "--print" in argv
    once = "--once" in argv
    if "--help" in argv or "-h" in argv:
        print(__doc__)
        return 0

    if not HAVE_ROS:
        print(f"FATAL: rclpy is required ({ROS_ERR}). "
              "source /opt/ros/humble/setup.bash and set ROS_DOMAIN_ID=20.",
              file=sys.stderr)
        return 2
    dom = os.environ.get("ROS_DOMAIN_ID")
    if dom != "20":
        log(f"WARNING: ROS_DOMAIN_ID={dom!r}, expected '20' -- this board "
            "publishes on domain 20 only, and on any other domain this "
            "recorder will sit silent and write an empty dataset")
    prof = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")
    if not prof or not os.path.exists(prof):
        # Not a theory. On this Pi, Fast DDS shared memory is dead across
        # process eras: discovery succeeds, topics are advertised, subscriber
        # counts are right, and ZERO messages arrive. Without the UDPv4-only
        # profile this recorder produces a file of headers and heartbeats that
        # is indistinguishable from a rover that never moved.
        log(f"WARNING: FASTRTPS_DEFAULT_PROFILES_FILE={prof!r} -- shared "
            "memory transport is DEAD on this Pi and discovery will still "
            "succeed. Expect every ROS topic to be silent. Set it to "
            "/etc/fpms/fastdds_udp_only.xml.")

    sink = Sink(OUT_DIR, echo=echo)
    rclpy.init()
    node = Recorder(sink, echo=echo)
    bus = Bus(node.on_mqtt)
    bus.start()
    node._bus_ok = bus.connected

    stop = threading.Event()

    def _sig(_s, _f):
        log("signal: closing the dataset")
        stop.set()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    t_end = time.time() + 60.0 if once else None
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
            node._bus_ok = bus.connected
            if t_end and time.time() >= t_end:
                break
    except KeyboardInterrupt:
        pass
    finally:
        # The prior is banked BEFORE the footer, so a clean stop never loses a
        # run's worth of grid evidence. It is also written every PRIOR_WRITE_S
        # while running, because this rover is stopped by unplugging it and a
        # save that only happens on SIGTERM is a save that never happens.
        if node.prior is not None:
            try:
                node.prior.commit()
                node.prior.save()
                log(f"learned grid prior: {len(node.prior.cells)} cell(s) "
                    f"-> {node.prior.path}")
            except Exception as e:
                log(f"grid prior final save failed: {e}")
        sink.write({"rec": "footer", "ts": round(time.time(), 3),
                    "episodes": node.seq, "records": sink.n,
                    "records_dropped_low_disk": sink.dropped,
                    "grid_prior_cells": (len(node.prior.cells)
                                         if node.prior else None)})
        sink.close()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    log(f"done: {node.seq} episodes, {sink.n} records in {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
