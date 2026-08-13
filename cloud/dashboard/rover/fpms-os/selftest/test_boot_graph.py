#!/usr/bin/env python3
"""Offline validator for the FPMS-OS systemd BOOT GRAPH.

    python3 selftest/test_boot_graph.py
    python3 selftest/test_boot_graph.py --self-check     # prove the checks bite
    python3 selftest/test_boot_graph.py /path/to/units   # validate another tree

WHAT THIS FILE IS FOR, AND WHY IT IS NOT test_image_offline.py
==============================================================
test_image_offline.py checks units ONE AT A TIME: does it parse, does its
ExecStart exist, does it carry the env block. Every one of those questions can
be answered by looking at a single file, and every one of them can be answered
YES while the boot SEQUENCE is wrong.

This file checks the GRAPH -- the thing no single unit file can be wrong about
on its own:

  * cored is ordered before the two things that can move a rover;
  * consumers are ordered after publishers;
  * nothing waits on the 90-225 second serial link;
  * exactly one process owns map->odom;
  * the enable/mask/not-enable lists agree with the units on disk.

Those properties are the encoding of this project's hard-won failures. A
well-meant `After=` added six months from now breaks one of them silently:
systemd does not warn, `systemctl status` stays green, and the symptom shows up
as "the dashboard is empty" or "the rover moved before I could stop it".

WHAT THIS CANNOT PROVE -- read this before trusting a green run
===============================================================
  1. IT READS FILES, NOT A RUNNING SYSTEM. `systemctl show` on the rover is the
     only authority on what systemd actually resolved. Drop-ins under
     /etc/systemd/system/<unit>.d/, unit files added by apt after the build,
     `systemctl edit`, and a mask applied at runtime are all invisible here.
  2. IMPLICIT DEPENDENCIES ARE NOT MODELLED. DefaultDependencies=yes silently
     adds After=basic.target / Before=shutdown.target and more. A cycle that
     only exists once those are added will NOT be found here. `systemd-analyze
     verify` on the built image is the check that would.
  3. ORDERING IS NOT TIMING. `Before=` on a Type=simple unit means "execve has
     returned", not "the node has created its publishers and DDS has discovered
     them". Every race in this project's history lived in exactly that gap --
     the rosbridge/odom-tf failure is a race the barrier only NARROWS. A green
     run here says the declared order is right, not that the rover works.
  4. Only the ordering directives (After=/Before=) and the requirement
     directives (Wants=/Requires=/BindsTo=/PartOf=/Conflicts=) are read.
     Socket/path/timer activation and Upholds= are not.
"""

import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
DEFAULT_UNITS = os.path.join(ROOT, "overlay", "etc", "systemd", "system")
ENABLE_SCRIPT = os.path.join(ROOT, "scripts", "60-enable-units.sh")

UNITS_DIR = DEFAULT_UNITS

failures, warnings = [], []


def fail(check, msg):
    failures.append((check, msg))


def warn(check, msg):
    warnings.append((check, msg))


# ---------------------------------------------------------------------------
# THE PARSER
#
# There is no systemd library here, so this reimplements the parts of unit-file
# syntax the FPMS units actually use. Every branch below exists because getting
# it wrong has a named consequence:
#
#   repeated directives      six units declare WantedBy= twice. Keeping only
#                            the last one loses fpms-ros-publishers.target and
#                            the barrier stops waiting for that publisher.
#   several values per line  "Before=fpms-missions.service fpms-teleop.service"
#                            is ONE line and TWO edges. Reading it as one edge
#                            named "fpms-missions.service fpms-teleop.service"
#                            finds no edge at all and passes silently.
#   `\` line continuation    60-enable-units.sh mis-parsed exactly this and
#                            created a directory literally named "\" while
#                            dropping the real target on the continuation line.
#                            ExecStart= in fpms-tf, fpms-map-anchor and
#                            fpms-dashboard is spread over many lines this way.
#   comments                 these units are mostly prose, and the prose DISCUSSES
#                            directives the unit deliberately omits ("NO
#                            ROS_DOMAIN_ID here, deliberately"). Matching raw
#                            text turns every explanation into a false failure.
#   empty assignment         "After=" with no value RESETS the list in systemd.
#                            Treating it as an edge to "" would be a phantom node.
# ---------------------------------------------------------------------------

LIST_KEYS = {
    "after", "before", "wants", "requires", "requisite", "bindsto", "partof",
    "conflicts", "wantedby", "requiredby", "also", "upholds", "propagatesreloadto",
}

UNIT_SUFFIXES = (".service", ".target", ".socket", ".timer", ".path", ".mount",
                 ".device", ".slice", ".scope", ".automount", ".swap")


class Unit(object):
    def __init__(self, name, path):
        self.name = name
        self.path = path
        # [(section, key, value, lineno)] in file order.
        self.directives = []

    # -- accessors ---------------------------------------------------------
    def entries(self, key, section=None):
        k = key.lower()
        return [d for d in self.directives
                if d[1].lower() == k and (section is None or d[0] == section)]

    def values(self, key, section=None):
        """Merged list value, honouring systemd's reset-on-empty semantics."""
        out = []
        for _sec, _key, val, _ln in self.entries(key, section):
            if val.strip() == "":
                out = []          # an empty assignment clears the list
                continue
            out.extend(val.split())
        return out

    def last(self, key, section=None):
        e = self.entries(key, section)
        return e[-1][2].strip() if e else None

    def has(self, key, section=None):
        return bool(self.entries(key, section))

    def deps(self, key):
        """Unit names named by a dependency directive, normalised."""
        return [normalise(v) for v in self.values(key, "Unit")]


def normalise(name):
    """`fpms-cored` and `fpms-cored.service` must not be two different nodes."""
    if name.endswith(UNIT_SUFFIXES):
        return name
    return name + ".service"


def logical_lines(text):
    """Yield (lineno, text) with continuations joined and comments dropped.

    A comment line that ends in a backslash CONTINUES in systemd >= v237, which
    means a stray trailing backslash in the prose swallows the directive below
    it and nothing anywhere says so. That is worth a warning of its own, so it
    is handled here rather than being quietly ignored.
    """
    lines = text.split("\n")
    swallowed = []
    i = 0
    while i < len(lines):
        start = i
        cur = lines[i].rstrip("\r")
        stripped = cur.strip()
        if stripped == "" or stripped.startswith("#") or stripped.startswith(";"):
            while cur.rstrip().endswith("\\") and i + 1 < len(lines):
                i += 1
                cur = lines[i].rstrip("\r")
                swallowed.append(start + 1)
            i += 1
            continue
        acc = cur
        while acc.rstrip().endswith("\\") and i + 1 < len(lines):
            acc = acc.rstrip()[:-1]
            i += 1
            acc = acc + " " + lines[i].rstrip("\r").strip()
        yield (start + 1, acc, swallowed)
        i += 1


def parse_unit(path):
    name = os.path.basename(path)
    unit = Unit(name, path)
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    section = None
    for lineno, line, swallowed in logical_lines(text):
        s = line.strip()
        if s.startswith("[") and s.endswith("]"):
            section = s[1:-1].strip()
            continue
        if "=" not in s:
            warn("parse", "%s:%d: not a section header and not key=value: %r"
                 % (name, lineno, s[:60]))
            continue
        key, val = s.split("=", 1)
        key = key.strip()
        # systemd allows a leading "-" / "@" / ":" only on Exec*= values, and
        # those are kept verbatim; everything else is stripped of whitespace.
        unit.directives.append((section, key, val.strip(), lineno))
        for ln in swallowed:
            warn("parse", "%s:%d: a COMMENT line ends in a backslash. systemd "
                          "continues it, so the line below is swallowed and "
                          "never applied." % (name, ln))
    return unit


# ---------------------------------------------------------------------------
# THE UNIT INVENTORY
#
# Every shipped unit is classified, and the classification is CHECKED FOR
# COMPLETENESS below. A new unit dropped into the overlay that nobody has
# classified fails this file, which is the point: the failure this project
# keeps paying for is a thing that was added and never decided about.
# ---------------------------------------------------------------------------

# Units that create ROS publications the consumers need to see. These must be
# ordered Before= the barrier.
ROS_PUBLISHERS = {
    "fpms-tf.service",            # static TF below base_footprint
    "fpms-odom-tf.service",       # /odom + odom->base_footprint
    "fpms-map-anchor.service",    # static map->odom
    "fpms-lidar-ros.service",     # /scan_lidar
    "fpms-teleop.service",        # /cmd_vel
    "fpms-missions.service",      # /cmd_vel and mission topics
    # MQTT -> ROS mirror for the camera and NPU health that nothing else
    # publishes: the agent sends frames straight to MQTT and fpms-npud raises
    # its faults there, so neither had a ROS source and the dashboard rendered
    # both tiles as NO ROS SOURCE rather than guess. A publisher, so it belongs
    # before the barrier -- a consumer attaching first is accepted, subscribed
    # and silent forever, which is exactly how those tiles would come to look
    # healthy on a rover that is detecting nothing.
    "fpms-telemetry-ros.service",
}

# micro-ros-agent IS a publisher -- every topic the ESP32 produces arrives
# through it -- and it is nevertheless DELIBERATELY EXCLUDED from the set
# above, because ordering the barrier after it would order every consumer
# after a link that costs 90-225 s. docs/ARCHITECTURE.md draws it OUTSIDE the
# publishers block for exactly this reason, and SPEC.md section 3 is blunt:
# "Do not add After=micro-ros-agent to anything." The consumers detect their
# own missing input and say so instead. See test_no_transitive_uros_dependency.
UROS_AGENT = "micro-ros-agent.service"

# Units that only consume ROS topics through rosbridge. These must be ordered
# After= the barrier.
ROS_CONSUMERS = {
    "fpms-rosbridge.service",
    "fpms-console.service",
    "fpms-dashboard.service",
}

# Everything else, with the reason it is neither. Being on this list is a
# decision, not an omission.
NOT_IN_THE_PUBLISH_GRAPH = {
    "fpms-cored.service": "MQTT stop authority; publishes no ROS topic",
    "fpms-rover-agent.service": "MQTT only -- deliberately not a ROS node",
    "fpms-npud.service": "MQTT only -- deliberately not a ROS node",
    "fpms-npu-tune.service": "report-only monitor, no ROS at all",
    "fpms-model-provision.service": "oneshot provisioning, MQTT only",
    "fpms-firstboot.service": "runs before anything ROS exists",
    "fpms-wifi-powersave-hold.service": "driver workaround, no ROS",
    "fpms-uros-supervisor.service": "watchdog; subscribes, never publishes",
    "fpms-selftest.service": "report-only; publishes to no ROS topic",
    "fpms-hwcheck.service": "report-only; the report FAILS if it publishes",
    "fpms-ros-settle.service": "fallback, not enabled; restarts publishers",
    "fpms-nav2.service": "operator-initiated, not enabled",
    "fpms-slam-mapping.service": "operator-initiated, not enabled",
    "fpms-slam-localization.service": "operator-initiated, not enabled",
    "fpms-ros-tunnel.service": "MASKED -- second /cmd_vel writer",
    "fpms-rtos-follower.service": "MASKED -- second /cmd_vel writer",
    "fpms-ros-publishers.target": "the barrier itself",
    UROS_AGENT: "publisher, but see UROS_AGENT above -- 90-225 s link",
}

BARRIER = "fpms-ros-publishers.target"
CORED = "fpms-cored.service"
MOVERS = ("fpms-missions.service", "fpms-teleop.service")
MAP_ODOM_OWNERS = {"fpms-map-anchor.service",
                   "fpms-slam-mapping.service",
                   "fpms-slam-localization.service"}

# Requirement directives that make a failure PROPAGATE. Wants= is deliberately
# absent: it is the one that does not.
HARD_DEP_KEYS = ("Requires", "Requisite", "BindsTo", "PartOf")


_cache = {}


def all_units():
    if UNITS_DIR not in _cache:
        if not os.path.isdir(UNITS_DIR):
            _cache[UNITS_DIR] = {}
        else:
            names = sorted(f for f in os.listdir(UNITS_DIR)
                           if f.endswith(UNIT_SUFFIXES))
            _cache[UNITS_DIR] = dict(
                (n, parse_unit(os.path.join(UNITS_DIR, n))) for n in names)
    return _cache[UNITS_DIR]


def ordering_edges():
    """The ordering digraph. Edge (a, b) means: a is ordered BEFORE b.

    `After=X` in unit U yields (X, U). `Before=Y` in U yields (U, Y). Units
    named but not shipped (mosquitto.service, network-online.target, the .device
    unit) become nodes with no outgoing edges of their own, which is honest: we
    do not know what they declare, so we assume nothing.
    """
    edges = set()
    for name, u in all_units().items():
        for dep in u.deps("After"):
            edges.add((dep, name))
        for dep in u.deps("Before"):
            edges.add((name, dep))
    return edges


def successors():
    succ = {}
    for a, b in ordering_edges():
        succ.setdefault(a, set()).add(b)
        succ.setdefault(b, set())
    return succ


def reaches(start, succ):
    """Every node that must start AFTER `start`."""
    seen, stack = set(), [start]
    while stack:
        n = stack.pop()
        for m in succ.get(n, ()):
            if m not in seen:
                seen.add(m)
                stack.append(m)
    return seen


# ---------------------------------------------------------------------------
# 1. NO ORDERING CYCLE
# ---------------------------------------------------------------------------
def test_no_ordering_cycle():
    """An ordering cycle does not fail loudly -- systemd DELETES AN EDGE.

    When systemd finds a cycle it breaks it by dropping one ordering edge, and
    which edge it drops is not something the unit files control. It logs the
    fact and carries on. So a cycle anywhere in this graph silently forfeits
    one ordering guarantee somewhere in it -- and the one it forfeits could be
    `fpms-cored before fpms-missions`, which is a safety property, or
    `publishers before the barrier`, which is the whole reason the barrier
    exists. There is no such thing as a harmless cycle here.
    """
    succ = successors()
    colour = {}                       # 0 = on stack, 1 = done
    path = []

    def visit(n):
        colour[n] = 0
        path.append(n)
        for m in sorted(succ.get(n, ())):
            if colour.get(m) == 0:
                cyc = path[path.index(m):] + [m]
                fail("no_cycle",
                     "ordering cycle: %s -- systemd will delete one of these "
                     "edges at random and log it. Whichever it picks, an "
                     "ordering guarantee this image depends on is gone."
                     % " -> ".join(cyc))
                continue
            if m not in colour:
                visit(m)
        path.pop()
        colour[n] = 1

    for n in sorted(succ):
        if n not in colour:
            visit(n)


# ---------------------------------------------------------------------------
# 2. THE SAFETY EDGE
# ---------------------------------------------------------------------------
def test_cored_before_movers():
    """THE ordering edge that is a safety property, and its opposite half.

    Half one: there must be no window at boot in which something that can move
    the rover accepts a command while the stop authority is not yet listening.
    fpms-cored must therefore be Before= both /cmd_vel writers.

    Half two: if cored FAILS, missions and teleop must still start. A rover you
    cannot drive is not safer than one you can stop, and cored has never run on
    the Pi -- its own header says so. So the pull-in must be Wants=, never
    Requires=/BindsTo=/Requisite=/PartOf=, in EITHER direction.

    These two pull against each other. Asserting only the first produces a
    stack that will not boot when cored is broken; asserting only the second
    produces a rover that can move before anything can stop it. Both, or
    neither is worth anything.
    """
    us = all_units()
    if CORED not in us:
        fail("safety_edge", "%s is not shipped at all" % CORED)
        return

    succ = successors()
    for mover in MOVERS:
        if mover not in us:
            fail("safety_edge", "%s is not shipped" % mover)
            continue
        # Declared directly by cored, or by the mover's own After=. Both are
        # the same ordering edge to systemd; accept either, demand one.
        direct = mover in us[CORED].deps("Before") or CORED in us[mover].deps("After")
        if not direct:
            fail("safety_edge",
                 "%s is NOT ordered before %s. At boot that unit can accept a "
                 "command while the STOP authority is not yet subscribed. This "
                 "is the one ordering edge SPEC.md section 3 calls a safety "
                 "property." % (CORED, mover))
        elif mover not in reaches(CORED, succ):
            # Belt and braces: the edge is declared, but confirm the graph
            # agrees -- a cycle elsewhere could have been the thing that
            # deleted it.
            fail("safety_edge",
                 "%s declares the edge to %s but the ordering graph does not "
                 "contain it" % (CORED, mover))

    # The far side must not be able to take the rover out of service.
    for name, u in sorted(us.items()):
        for key in HARD_DEP_KEYS:
            if CORED in u.deps(key):
                fail("safety_edge",
                     "%s declares %s=%s. That makes cored's failure propagate: "
                     "if the stop authority is down, this unit refuses to "
                     "start too. Wants=, never %s -- a rover you cannot drive "
                     "is not safer than one you can stop."
                     % (name, key, CORED, key))
        if name == CORED:
            for key in HARD_DEP_KEYS:
                for mover in MOVERS:
                    if mover in u.deps(key):
                        fail("safety_edge",
                             "%s declares %s=%s. cored must ORDER the movers, "
                             "not require them." % (CORED, key, mover))

    # And the pull-in that does exist must be the soft one.
    puller = [n for n, u in us.items() if CORED in u.deps("Wants")]
    if not puller and CORED not in us[MOVERS[0]].deps("Wants"):
        warn("safety_edge",
             "nothing declares Wants=%s. The Before= edge only orders units "
             "that are ALREADY being started; if a future change stops pulling "
             "cored into the boot transaction, the edge becomes a no-op and "
             "the movers start with no stop authority at all." % CORED)


# ---------------------------------------------------------------------------
# 3. NOTHING TRANSITIVELY WAITS ON THE SERIAL LINK
# ---------------------------------------------------------------------------
def test_no_transitive_uros_dependency():
    """The 90-225 second rule, applied to the TRANSITIVE closure.

    The ESP32-S3 re-establishes its XRCE session unaided in 90-225 seconds
    after any agent restart. An ordering edge that reaches micro-ros-agent
    therefore turns a trivial restart of a bridge into a multi-minute outage of
    every topic on the robot. SPEC.md section 3: "Do not add
    After=micro-ros-agent to anything." docs/ARCHITECTURE.md says the same and
    says TRANSITIVELY.

    Transitively is the word that matters, and it is why a per-unit check
    cannot find this. No unit file has to name micro-ros-agent to end up
    behind it: one edge onto a shared barrier is enough, and the unit that
    pays is three files away from the unit that declared it.

    fpms-uros-supervisor is the single allowed exception. Its entire job is
    watching that link, so being ordered behind it is not a coupling, it is the
    definition of the unit.
    """
    us = all_units()
    if UROS_AGENT not in us:
        fail("uros_isolation", "%s is not shipped" % UROS_AGENT)
        return

    succ = successors()
    downstream = reaches(UROS_AGENT, succ)
    offenders = sorted(n for n in downstream
                       if n != "fpms-uros-supervisor.service")
    if not offenders:
        return

    # Name the EDGE, not just the victims: the whole difficulty of a transitive
    # defect is that the file that pays is not the file that is wrong.
    culprits = []
    for name, u in sorted(us.items()):
        if UROS_AGENT in u.deps("After"):
            culprits.append("%s declares After=%s" % (name, UROS_AGENT))
    for tgt in us[UROS_AGENT].deps("Before"):
        culprits.append("%s declares Before=%s" % (UROS_AGENT, tgt))

    fail("uros_isolation",
         "%d unit(s) are transitively ordered after %s: %s. Responsible "
         "edge(s): %s. Every one of those units now waits on a link that costs "
         "90-225 s, and each of them was written on the assumption that it does "
         "not."
         % (len(offenders), UROS_AGENT, ", ".join(offenders),
            "; ".join(c for c in culprits
                      if "fpms-uros-supervisor" not in c) or "(none found)"))


# ---------------------------------------------------------------------------
# 4. PUBLISHERS BEFORE CONSUMERS
# ---------------------------------------------------------------------------
def test_publishers_before_consumers():
    """rosbridge only ever delivers topics whose publisher pre-existed it.

    Measured and reproduced three times: rosbridge started at 22:26:33,
    fpms-odom-tf at 22:45:45, and /odom delivered ZERO messages while
    publishing healthily at 6.6 Hz. The client subscription is accepted,
    subscribed, and silent forever -- with no error anywhere.

    So every ROS publisher declares Before=fpms-ros-publishers.target and every
    consumer declares After= it. A target rather than direct edges, so the
    consumers are ORDERED without being COUPLED: a publisher restarting later
    must not take the operator's whole view of the robot with it.

    The WantedBy= half matters as much as the Before= half. Before= only orders
    units that are already in the same start job; if the barrier does not also
    PULL IN the publisher, the barrier can be reached before that publisher has
    started and the ordering buys nothing.
    """
    us = all_units()
    if BARRIER not in us:
        fail("publish_barrier", "%s is not shipped" % BARRIER)
        return

    for p in sorted(ROS_PUBLISHERS):
        if p not in us:
            fail("publish_barrier", "%s is classified a publisher but is not "
                                    "shipped" % p)
            continue
        u = us[p]
        if BARRIER not in u.deps("Before"):
            fail("publish_barrier",
                 "%s publishes ROS topics but does not declare Before=%s. A "
                 "consumer can start first, subscribe, and stay silent forever "
                 "with no error anywhere." % (p, BARRIER))
        if BARRIER not in [normalise(v) for v in u.values("WantedBy", "Install")]:
            fail("publish_barrier",
                 "%s is not WantedBy=%s. Before= only orders units already in "
                 "the start job, so the barrier could be reached without ever "
                 "waiting for this publisher." % (p, BARRIER))

    for c in sorted(ROS_CONSUMERS):
        if c not in us:
            fail("publish_barrier", "%s is classified a consumer but is not "
                                    "shipped" % c)
            continue
        u = us[c]
        if BARRIER not in u.deps("After"):
            fail("publish_barrier",
                 "%s consumes ROS topics but does not declare After=%s. It can "
                 "attach before a publisher exists, which is accepted and "
                 "silent -- the worst-shaped failure this project has."
                 % (c, BARRIER))
        if BARRIER in u.deps("Before"):
            fail("publish_barrier",
                 "%s declares Before=%s. It is a consumer; that inverts the "
                 "barrier." % (c, BARRIER))
        # Direct edges onto a publisher are exactly what the barrier exists to
        # avoid: they couple a consumer to one publisher's restart.
        for dep in u.deps("After"):
            if dep in ROS_PUBLISHERS or dep == UROS_AGENT:
                fail("publish_barrier",
                     "%s declares After=%s directly. Use the barrier: a direct "
                     "edge re-couples the consumer to one publisher's restart, "
                     "which is the coupling %s was created to remove."
                     % (c, dep, BARRIER))

    # Completeness: nobody may add a unit and leave it unclassified.
    classified = (set(ROS_PUBLISHERS) | set(ROS_CONSUMERS)
                  | set(NOT_IN_THE_PUBLISH_GRAPH))
    for name in sorted(us):
        if name not in classified:
            fail("publish_barrier",
                 "%s is shipped but this test does not classify it as a "
                 "publisher, a consumer, or neither. Decide, in this file, "
                 "with the reason written down -- an unclassified unit is how "
                 "an unordered publisher gets into an image." % name)


# ---------------------------------------------------------------------------
# 5. ONE OWNER FOR map->odom
# ---------------------------------------------------------------------------
def test_map_odom_single_owner():
    """TF_TREE.md: "One publisher per edge, no exceptions."

    fpms-map-anchor publishes a STATIC map->odom at the arena start pose.
    slam_toolbox, in either mode, publishes a DYNAMIC one from its scan
    matcher. Two publishers on one TF edge do not error. They produce a pose
    that FLICKERS between two self-consistent answers, which is the hardest
    class of bug to see on an arena map -- every individual reading looks
    right.

    Conflicts= makes the exclusion mechanical: starting one STOPS the other. A
    comment cannot do that, and this used to be a comment.

    Conflicts= is checked in BOTH directions on purpose. systemd does treat it
    as implicitly symmetric at runtime, but a one-sided declaration means the
    guarantee lives in one file that a future edit can delete without the other
    file looking wrong.
    """
    us = all_units()
    for a in sorted(MAP_ODOM_OWNERS):
        if a not in us:
            fail("map_odom", "%s is not shipped" % a)
            continue
        declared = set(us[a].deps("Conflicts"))
        for b in sorted(MAP_ODOM_OWNERS):
            if a == b:
                continue
            if b not in declared:
                fail("map_odom",
                     "%s does not declare Conflicts=%s. Both own map->odom; "
                     "with both running the pose flickers between two "
                     "self-consistent answers and nothing errors."
                     % (a, b))
        # Ordering alone is not exclusion, and looks like it is.
        if not declared and (us[a].deps("After") or us[a].deps("Before")):
            warn("map_odom",
                 "%s declares ordering but no Conflicts=. Ordering does not "
                 "stop two publishers coexisting." % a)


# ---------------------------------------------------------------------------
# 6. THE THREE LISTS IN 60-enable-units.sh
# ---------------------------------------------------------------------------
def _enable_lists():
    """Read BOOT_UNITS / masked / installed-not-enabled out of stage 60.

    Parsed rather than duplicated, because a list copied into a test is a list
    that goes stale silently. The shell uses `\\` continuations in the
    not-enabled loop, so those are joined first -- the same syntax the stage's
    own awk parser once got wrong badly enough to create a directory named "\\".
    """
    if not os.path.exists(ENABLE_SCRIPT):
        return None
    with open(ENABLE_SCRIPT, encoding="utf-8") as fh:
        raw = fh.read()
    joined = re.sub(r"\\\n\s*", " ", raw)

    suffixes = "|".join(re.escape(s) for s in UNIT_SUFFIXES)
    unit_rx = re.compile(r"^[A-Za-z0-9@:._-]+(?:%s)$" % suffixes)

    def tokens(blob):
        out = []
        for line in blob.split("\n"):
            line = re.sub(r"#.*$", "", line)
            for tok in line.split():
                if unit_rx.match(tok):
                    out.append(tok)
        return out

    m = re.search(r"BOOT_UNITS=\((.*?)\n\)", joined, re.S)
    boot = tokens(m.group(1)) if m else []

    # The two `for u in <literal names>; do` loops. The BOOT_UNITS loops use
    # "${BOOT_UNITS[@]}" and are skipped by the ${ test.
    loops = [(m.start(), m.group(1))
             for m in re.finditer(r"for u in ([^;]*?); do", joined)
             if "${" not in m.group(1)]
    mask_hdr = joined.find("masked, permanently")
    notenb_hdr = joined.find("installed but deliberately NOT enabled")
    masked, notenabled = [], []
    for pos, blob in loops:
        if notenb_hdr != -1 and pos > notenb_hdr:
            notenabled = tokens(blob)
        elif mask_hdr != -1 and pos > mask_hdr:
            masked = tokens(blob)
    return boot, masked, notenabled


def test_enable_lists_disjoint_and_complete():
    """The enabled / masked / not-enabled lists must partition the units shipped.

    fpms-missions was once found DISABLED on the running Pi, so the rover came
    back from a power cycle with no mission executor and NOTHING SAID SO. Every
    boot unit is enabled at image build time for exactly that reason -- which
    only helps if the list is right.

    Four ways it can be wrong, all silent:
      * an enabled unit with no [Install]: `systemctl enable` prints "no
        installation config", changes nothing, and the unit sits in the list
        looking enabled forever;
      * a masked unit that is not shipped: masking an absent file is a weaker
        guarantee than masking a present one -- a restore from backup or a copy
        from the old Pi puts an unmasked /cmd_vel writer back on the robot;
      * a unit in two lists: whichever the script reaches last wins, and which
        that is depends on the order of two loops nobody reads;
      * a unit in NO list: shipped, never decided about. That is the
        fpms-missions failure with the sign flipped.
    """
    lists = _enable_lists()
    if lists is None:
        fail("enable_lists", "scripts/60-enable-units.sh is missing")
        return
    boot, masked, notenabled = lists
    if not boot or not masked or not notenabled:
        fail("enable_lists",
             "could not read all three lists out of 60-enable-units.sh "
             "(boot=%d masked=%d not-enabled=%d). The script's structure "
             "changed and this check is now reading nothing -- which would "
             "otherwise look like a pass."
             % (len(boot), len(masked), len(notenabled)))
        return

    us = all_units()
    named = {"enabled": boot, "masked": masked, "not-enabled": notenabled}

    for a in named:
        for b in named:
            if a < b:
                both = set(named[a]) & set(named[b])
                if both:
                    fail("enable_lists",
                         "in both the %s and %s lists: %s"
                         % (a, b, ", ".join(sorted(both))))

    for u in sorted(boot):
        if u not in us:
            fail("enable_lists",
                 "%s is in BOOT_UNITS but is not shipped in the overlay. The "
                 "image would boot without it and nothing at runtime would say "
                 "so." % u)
            continue
        if not us[u].values("WantedBy", "Install") and \
           not us[u].values("RequiredBy", "Install"):
            fail("enable_lists",
                 "%s is in BOOT_UNITS but has no [Install] target. `systemctl "
                 "enable` cannot enable it and reports success doing nothing."
                 % u)

    for u in sorted(masked):
        if u not in us:
            fail("enable_lists",
                 "%s is masked but not shipped. Masking a unit that does not "
                 "exist is not the same guarantee: a restored backup would put "
                 "an unmasked second /cmd_vel writer on the robot." % u)

    for u in sorted(notenabled):
        if u not in us:
            fail("enable_lists", "%s is listed not-enabled but is not shipped"
                 % u)

    covered = set(boot) | set(masked) | set(notenabled)
    for name in sorted(us):
        if name not in covered:
            fail("enable_lists",
                 "%s is shipped but appears in none of the three lists in "
                 "60-enable-units.sh. Nobody has decided whether it boots. "
                 "That is exactly how fpms-missions came back from a power "
                 "cycle absent." % name)

    # A unit with no [Install] cannot be enabled by accident, which is the
    # guarantee the not-enabled list leans on for nav2 and the SLAM pair.
    for u in sorted(notenabled):
        if u in us and us[u].values("WantedBy", "Install"):
            warn("enable_lists",
                 "%s is deliberately not enabled but HAS an [Install] section, "
                 "so it is one `systemctl enable` -- or one well-meant build "
                 "edit -- away from the boot path." % u)


# ---------------------------------------------------------------------------
# 7. THE ENV BLOCK, AND WHICH SIDE WINS
# ---------------------------------------------------------------------------
def test_ros_env_block_and_precedence():
    """The env block is a safety property in ROS units and a knob in fpms-npud.

    A unit missing FASTRTPS_DEFAULT_PROFILES_FILE falls back to Fast DDS's
    shared-memory transport, whose segments do not survive across process eras:
    discovery succeeds and NOT ONE BYTE is exchanged. A unit missing
    RMW_IMPLEMENTATION can land on a different RMW from the rest of the stack,
    discover nothing, and report no error -- fpms-teleop shipped exactly like
    that. A unit on the wrong ROS_DOMAIN_ID sees an empty topic list, which is
    indistinguishable from dead hardware.

    Then the ORDER decides who wins, because systemd applies Environment= and
    EnvironmentFile= in file order and a later setting overrides an earlier one:

      ROS units    EnvironmentFile= FIRST -> the UNIT wins. The domain and the
                   RMW are safety properties; a stale value in config.env must
                   not be able to move the rover onto an empty domain.

      fpms-npud    EnvironmentFile= LAST -> CONFIG.ENV wins. Every FPMS_NPU_*
                   value is a tuning knob and config.env is the documented
                   single configuration point. This unit shipped one revision
                   with the file first and a comment claiming the opposite, so
                   every knob an operator set was silently inert.
    """
    us = all_units()
    required = ("RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE")
    nice_to_have = ("ROS_LOCALHOST_ONLY", "PYTHONUNBUFFERED", "HOME")

    for name in sorted(us):
        u = us[name]
        env = dict()
        order_env, order_file = [], []
        for _sec, key, val, ln in u.directives:
            if key == "Environment":
                order_env.append(ln)
                for tok in val.split():
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        env[k] = v.strip('"')
            elif key == "EnvironmentFile":
                order_file.append(ln)
        if "ROS_DOMAIN_ID" not in env:
            continue                      # not a ROS unit; see the classifier
        if env.get("ROS_DOMAIN_ID") != "20":
            fail("ros_env",
                 "%s sets ROS_DOMAIN_ID=%s. The ESP32 declares domain 20 in its "
                 "CREATE_PARTICIPANT and publishes nowhere else, so any other "
                 "value gives an EMPTY topic list that reads as dead hardware."
                 % (name, env.get("ROS_DOMAIN_ID")))
        for key in required:
            if key not in env:
                fail("ros_env",
                     "%s is a ROS unit but does not set %s. Missing it does not "
                     "error -- it produces a node that runs, discovers, and "
                     "never exchanges a byte." % (name, key))
        for key in nice_to_have:
            if key not in env:
                warn("ros_env",
                     "%s is a ROS unit without %s (SPEC.md section 2 lists it "
                     "in the mandatory block)" % (name, key))
        if not order_file:
            warn("ros_env",
                 "%s is a ROS unit with no EnvironmentFile=-/etc/fpms/config.env. "
                 "Nothing set in the documented single configuration point "
                 "reaches it." % name)
        elif min(order_file) > max(order_env):
            fail("ros_env",
                 "%s is a ROS unit but EnvironmentFile= comes AFTER the last "
                 "Environment=, so config.env overrides ROS_DOMAIN_ID and "
                 "RMW_IMPLEMENTATION. A stale domain there reads as dead "
                 "hardware." % name)

    # The deliberate inversion. fpms-npud is the one unit where config.env must
    # win, and it is the one unit that has already shipped the wrong way round.
    npud = us.get("fpms-npud.service")
    if npud is not None:
        env_lines = [ln for _s, k, _v, ln in npud.directives
                     if k == "Environment"]
        file_lines = [ln for _s, k, _v, ln in npud.directives
                      if k == "EnvironmentFile"]
        if not file_lines:
            fail("ros_env", "fpms-npud.service has no EnvironmentFile= at all, "
                            "so no FPMS_NPU_* knob in config.env has any effect")
        elif env_lines and max(file_lines) < max(env_lines):
            fail("ros_env",
                 "fpms-npud.service has EnvironmentFile= before its "
                 "Environment= defaults, so the UNIT wins and every FPMS_NPU_* "
                 "value an operator sets in config.env -- and everything "
                 "fpms-npu-tune writes -- is silently inert. This unit has "
                 "already shipped that way once.")


# ---------------------------------------------------------------------------
# 8. Restart= SANITY
# ---------------------------------------------------------------------------
def _restart_sec(unit):
    v = unit.last("RestartSec", "Service")
    if v is None:
        return None
    m = re.match(r"^(\d+(?:\.\d+)?)\s*(s|sec|second|seconds)?$", v)
    if m:
        return float(m.group(1))
    m = re.match(r"^(\d+)\s*(ms|msec)$", v)
    if m:
        return float(m.group(1)) / 1000.0
    m = re.match(r"^(\d+)\s*(min|minute|minutes|m)$", v)
    if m:
        return float(m.group(1)) * 60.0
    return None


def test_restart_sanity():
    """A long-running unit with no Restart= stays dead after one crash.

    Silently: `systemctl status` reports inactive, nothing is retried, and the
    first anyone knows is a topic that stopped. Every FPMS long-running service
    is meant to be Restart=always.

    The mirror of that is a Type=oneshot unit WITH Restart=: fpms-hwcheck says
    it in its own header -- re-running a report against hardware that is not
    going to fix itself republishes a verdict forever, and the failure most
    likely to be seen there is a drive board still linking, which anything
    resembling a retry makes WORSE.

    RestartSec must be explicit wherever Restart= is active, because the
    default is 100 ms. A crash-looping unit at 100 ms is a unit hammering the
    thing it depends on and smearing the crash across the journal so that the
    original traceback cannot be found.

    And fpms-cored keeps the shortest RestartSec in the stack, because it is
    the stop path: every second it is down is a second in which a stop has no
    owner.
    """
    us = all_units()
    restart_secs = {}
    for name in sorted(us):
        if not name.endswith(".service"):
            continue
        u = us[name]
        typ = (u.last("Type", "Service") or "simple").lower()
        restart = (u.last("Restart", "Service") or "no").lower()

        if typ == "oneshot":
            if restart not in ("no", "on-failure", "on-abnormal"):
                fail("restart",
                     "%s is Type=oneshot with Restart=%s. A oneshot is a "
                     "report or a provisioning step, not a service; restarting "
                     "it re-runs a verdict against hardware that will not fix "
                     "itself." % (name, restart))
            if restart == "no" and u.has("RestartSec", "Service"):
                warn("restart", "%s is Type=oneshot but sets RestartSec=, "
                                "which does nothing" % name)
            continue

        if restart == "no":
            # Legitimate for an operator-initiated run you stop deliberately,
            # but never for anything in the boot set.
            lists = _enable_lists()
            boot = lists[0] if lists else []
            if name in boot:
                fail("restart",
                     "%s is a long-running boot unit with Restart=no. One "
                     "crash and it stays dead, silently, until somebody "
                     "notices a missing topic." % name)
            else:
                warn("restart", "%s is long-running with Restart=no (not in "
                                "the boot set, so this may be deliberate)" % name)
            continue

        if not u.has("Restart", "Service"):
            fail("restart",
                 "%s is a long-running service with no Restart=. It stays dead "
                 "after its first crash and nothing says so." % name)
            continue

        secs = _restart_sec(u)
        if secs is None:
            fail("restart",
                 "%s sets Restart=%s but no usable RestartSec=. The default is "
                 "100 ms: a crash loop there hammers whatever it depends on "
                 "and buries the original traceback." % (name, restart))
            continue
        restart_secs[name] = secs

    if CORED in restart_secs:
        cored_sec = restart_secs[CORED]
        ties = sorted(n for n, s in restart_secs.items()
                      if n != CORED and s <= cored_sec)
        if ties:
            fail("restart",
                 "fpms-cored has RestartSec=%g, which is not shorter than %s. "
                 "cored is the stop path: every second it is down is a second "
                 "in which a stop has no owner, so it restarts fastest in the "
                 "stack by design." % (cored_sec, ", ".join(ties)))
    elif CORED in us:
        fail("restart", "fpms-cored has no usable RestartSec=")


# ---------------------------------------------------------------------------
# 9. THE PARSER'S OWN ASSUMPTIONS
# ---------------------------------------------------------------------------
def test_parser_assumptions():
    """Confirm the graph was actually built, rather than built out of nothing.

    Every check above is a statement about a set of edges. If the parser
    silently produced no edges -- a renamed directory, a syntax this file does
    not understand -- every one of them would pass. A test that passes against
    an empty input is worse than no test, so the input is checked too.
    """
    us = all_units()
    if not us:
        fail("parser", "no unit files found under %s" % UNITS_DIR)
        return
    edges = ordering_edges()
    if len(edges) < 20:
        fail("parser",
             "only %d ordering edges parsed from %d units. The parser is "
             "probably not reading these files." % (len(edges), len(us)))
    multi = [n for n, u in us.items()
             if any(len(v.split()) > 1
                    for _s, k, v, _l in u.directives if k in ("After", "Before"))]
    if not multi:
        warn("parser",
             "no After=/Before= line carries more than one value. These units "
             "do use that shape, so the parser may be splitting wrongly.")


# ---------------------------------------------------------------------------
# SELF-CHECK: mutate a copy and confirm each check bites
# ---------------------------------------------------------------------------
def _mutate(path, old, new):
    """Replace the first DIRECTIVE occurrence of `old`, never a comment one.

    These units are mostly prose, and the prose quotes the directives verbatim
    -- fpms-rosbridge.service's header contains the literal string
    "After=fpms-ros-publishers.target" nine lines above the real one. A naive
    first-occurrence replace edits the comment, changes nothing, and the
    self-check then reports the mutation as "not caught" when in truth it was
    never applied. That is the same shape of mistake this whole file exists to
    catch, so it is worth getting right here too.
    """
    with open(path, encoding="utf-8") as fh:
        body = fh.read()
    at = -1
    search = 0
    while True:
        idx = body.find(old, search)
        if idx == -1:
            break
        line_start = body.rfind("\n", 0, idx) + 1
        if not body[line_start:idx].lstrip().startswith(("#", ";")):
            at = idx
            break
        search = idx + 1
    if at == -1:
        raise AssertionError("mutation anchor not found outside comments in "
                             "%s: %r" % (os.path.basename(path), old[:60]))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(body[:at] + new + body[at + len(old):])


GHOST = """[Unit]
Description=a unit somebody added and nobody classified

[Service]
Type=simple
ExecStart=/bin/true
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
"""

# (unit file, anchor, replacement, check that must fail, must_fail?)
#   anchor None  -> CREATE the file with `replacement` as its body
#   replacement None -> REMOVE the file (from the temp copy only)
MUTATIONS = [
    ("fpms-cored.service",
     "Before=fpms-missions.service fpms-teleop.service",
     "# Before= removed by the self-check",
     "safety_edge", True),
    ("fpms-missions.service",
     "Wants=fpms-cored.service",
     "Requires=fpms-cored.service",
     "safety_edge", True),
    ("fpms-lidar-ros.service",
     "After=network-online.target mosquitto.service",
     "After=network-online.target mosquitto.service micro-ros-agent.service",
     "uros_isolation", True),
    ("fpms-tf.service",
     "Before=fpms-ros-publishers.target",
     "Before=fpms-ros-publishers.target\nAfter=fpms-rosbridge.service",
     "no_cycle", True),
    ("fpms-map-anchor.service",
     "Conflicts=fpms-slam-mapping.service fpms-slam-localization.service",
     "# Conflicts= removed by the self-check",
     "map_odom", True),
    ("fpms-odom-tf.service",
     "Before=fpms-ros-publishers.target",
     "# Before= removed by the self-check",
     "publish_barrier", True),
    ("fpms-rosbridge.service",
     "After=fpms-ros-publishers.target",
     "# After= removed by the self-check",
     "publish_barrier", True),
    ("fpms-teleop.service",
     "Environment=FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fpms/fastdds_udp_only.xml",
     "# profile line removed by the self-check",
     "ros_env", True),
    ("fpms-cored.service",
     "RestartSec=2",
     "RestartSec=30",
     "restart", True),
    ("fpms-npud.service",
     "EnvironmentFile=-/etc/fpms/config.env\n\nExecStart=/usr/bin/python3 /usr/local/bin/fpms-npud",
     "ExecStart=/usr/bin/python3 /usr/local/bin/fpms-npud",
     "ros_env", True),
    # PARSER MUTATIONS. The first must still PASS -- it is the same edge
    # written with a `\` continuation, and a parser that mis-handles that would
    # report a false failure. The second drops the continuation line's value
    # and must FAIL, which is the bug 60-enable-units.sh once shipped.
    ("fpms-cored.service",
     "Before=fpms-missions.service fpms-teleop.service",
     "Before=fpms-missions.service \\\n        fpms-teleop.service",
     "safety_edge", False),
    ("fpms-cored.service",
     "Before=fpms-missions.service fpms-teleop.service",
     "Before=fpms-missions.service \\",
     "safety_edge", True),
    # The three lists in 60-enable-units.sh, cross-checked against the disk.
    ("fpms-ghost.service", None, GHOST, "enable_lists", True),
    ("fpms-dashboard.service",
     "[Install]\nWantedBy=multi-user.target",
     "# [Install] removed by the self-check",
     "enable_lists", True),
    ("fpms-rtos-follower.service", None, None, "enable_lists", True),
]


def self_check():
    """Run every mutation against a COPY. Never touches the repository."""
    global UNITS_DIR
    real = UNITS_DIR
    ok = True
    print("=" * 72)
    print(" self-check: does each check actually bite?")
    print("=" * 72)
    for unit, old, new, check, must_fail in MUTATIONS:
        tmp = tempfile.mkdtemp(prefix="fpms-bootgraph-")
        try:
            dst = os.path.join(tmp, "system")
            shutil.copytree(real, dst)
            target = os.path.join(dst, unit)
            if old is None and new is None:
                os.remove(target)               # the COPY, never the repository
            elif old is None:
                with open(target, "w", encoding="utf-8") as fh:
                    fh.write(new)
            else:
                _mutate(target, old, new)
            UNITS_DIR = dst
            _cache.clear()
            del failures[:]
            del warnings[:]
            for t in _tests():
                try:
                    t()
                except Exception as exc:            # noqa: BLE001
                    fail(t.__name__, "test itself raised: %r" % exc)
            hit = sorted(set(c for c, _ in failures))
            caught = check in hit
            good = caught if must_fail else not caught
            ok = ok and good
            print("  %-5s %-28s %-16s %s"
                  % ("PASS" if good else "BROKEN",
                     unit.replace(".service", ""),
                     check + ("" if must_fail else " (must NOT fire)"),
                     "caught by " + ",".join(hit) if hit else "nothing fired"))
            if not good:
                for c, m in failures:
                    print("          [%s] %s" % (c, m[:110]))
        finally:
            UNITS_DIR = real
            _cache.clear()
            shutil.rmtree(tmp, ignore_errors=True)
    del failures[:]
    del warnings[:]
    print("-" * 72)
    print("  %s" % ("every mutation behaved as expected"
                    if ok else "SOME MUTATIONS WERE NOT CAUGHT - do not trust "
                               "a green run of this file"))
    print("=" * 72)
    return 0 if ok else 1


# ---------------------------------------------------------------------------
def _tests():
    return [v for k, v in sorted(globals().items())
            if k.startswith("test_") and callable(v)]


def main(argv):
    global UNITS_DIR
    args = [a for a in argv[1:] if a != "--self-check"]
    if args:
        UNITS_DIR = os.path.abspath(args[0])
    if "--self-check" in argv[1:]:
        return self_check()

    tests = _tests()
    for t in tests:
        try:
            t()
        except Exception as exc:                    # noqa: BLE001
            fail(t.__name__, "test itself raised: %r" % exc)

    print("=" * 72)
    print(" FPMS-OS boot graph checks")
    print("=" * 72)
    print("  units: %s" % UNITS_DIR)
    print("  %d units, %d ordering edges, %d checks run"
          % (len(all_units()), len(ordering_edges()), len(tests)))
    print("")
    for check, msg in warnings:
        print("  [WARN] %s: %s" % (check, msg))
    if warnings:
        print("")
    for check, msg in failures:
        print("  [FAIL] %s: %s" % (check, msg))
    print("-" * 72)
    if failures:
        print("  %d FAILED, %d warnings" % (len(failures), len(warnings)))
    else:
        print("  all passed, %d warnings" % len(warnings))
    print("")
    print("  WHAT THIS RUN DID NOT PROVE: it read unit FILES. It cannot know")
    print("  what systemd resolved on the rover (drop-ins, runtime masks and")
    print("  apt-installed units are invisible here), it does not model the")
    print("  implicit dependencies DefaultDependencies= adds, and ordering is")
    print("  not timing -- Before= on a Type=simple unit means execve returned,")
    print("  not that its publishers exist. A race that only shows up under")
    print("  load is out of reach of any offline check.")
    print("=" * 72)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
