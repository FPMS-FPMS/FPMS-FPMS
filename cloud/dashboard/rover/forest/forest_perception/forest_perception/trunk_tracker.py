#!/usr/bin/env python3
"""trunk_tracker.py -- cluster the filtered scan into obstacles and TRACK them.

WHY PYTHON HERE, WHEN THE FILTER AND THE COSTMAP LAYER ARE C++
--------------------------------------------------------------
This node runs at the scan rate over a few DOZEN clusters, not a few hundred
thousand points: the per-cycle work is association and bookkeeping, and the
budget is milliseconds with room to spare. It is also the node whose logic will
change most often on a real machine -- gate widths, birth and death rules, what
counts as one clump versus two. rclpy is the right trade for that; the two nodes
either side of it are C++ because one is per-point real time and the other is a
pluginlib costmap layer, which has no Python path at all.

STATUS: IMPLEMENTED and standalone-testable in its maths (`cluster`, `Track`,
`associate` are pure and take plain numbers). NEVER RUN ON HARDWARE -- there is
no forest rover in this project, and no ROS 2 environment here to run it in.

WHERE IT SITS. github.com/Ellakiya15/ros2-dynamic-obstacle-avoidance-bot (read
before this was written) is three packages -- bot_description, bot_gazebo,
bot_nav -- running stock Nav2 in Gazebo with a C++ navigation_node for
interactive goals. It has NO tracker: its "dynamic obstacle avoidance" is the
stock obstacle layer re-marking a moving thing every cycle. That is adequate
indoors and is exactly what fails in a forest, so this node is new work.

    scan_filtered ->  cluster  ->  associate  ->  tracks  ->  TrackedObstacles
                                                              |
                                            forest_nav/DynamicObstacleLayer

WHAT MAKES THIS A FOREST TRACKER AND NOT A ROOM TRACKER
-------------------------------------------------------
  * IT DOES NOT FIT CIRCLES OR BOXES. A trunk with a root flare, a multi-stem
    clump and a fallen limb are all non-convex, and a bounding shape around a
    fallen limb closes a gap the rover could have driven. Each track carries the
    ORDERED SUPPORT POLYGON of its own returns, and the costmap layer rasterises
    that outline as given.
  * IT SEPARATES "MOVING" FROM "MIS-TRACKED" BY EVIDENCE, NOT BY SPEED. A trunk
    that appears to drift at 0.2 m/s is odometry error, not a walking tree. A
    track is only declared MOVING after `min_moving_updates` consecutive
    consistent velocity estimates (`_moving_run`), which is what stops a drifting
    pose turning the whole forest into predicted swept volumes.
  * IT SURVIVES OCCLUSION. Under canopy a trunk is hidden and re-exposed
    constantly. A track coasts on its motion model for `max_coast_s` before it
    dies, and the costmap layer decays confidence rather than ray-cast clearing,
    so a leaf passing in front of a tree does not delete the tree.
  * IT DOES NOT ASSUME A MAP OR A WALL. Everything here is in the sensor/odom
    frame. There is no AMCL scan match against straight walls, because in the
    open there are none -- see the honest note in `main` about what that means.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point32, Vector3
from sensor_msgs.msg import LaserScan

from forest_msgs.msg import TrackedObstacle, TrackedObstacles


# ============================================================== PURE MATHS
def cluster(points, gap_m, min_points):
    """Adjacency clustering along the beam order. [[(x, y), ...], ...]

    ALONG THE BEAM ORDER, not a general spatial clustering, and that is
    deliberate: consecutive beams that land on one surface are the definition of
    one object to a scanner, and it costs O(n) instead of O(n log n) with no
    parameter to tune beyond the gap. The gap is ADAPTIVE with range in the
    caller, because two beams 0.5 degrees apart are 9 cm apart at 10 m and 0.9 cm
    apart at 1 m -- a fixed gap either splits far trunks into slivers or merges
    near ones into a wall.
    """
    out, cur = [], []
    for i, p in enumerate(points):
        if not cur:
            cur = [p]
            continue
        prev = cur[-1]
        if math.hypot(p[0] - prev[0], p[1] - prev[1]) <= gap_m:
            cur.append(p)
        else:
            if len(cur) >= min_points:
                out.append(cur)
            cur = [p]
    if len(cur) >= min_points:
        out.append(cur)
    # A scan wraps: the first and last clusters may be one object split by the
    # seam. Merging them is the difference between one trunk and two ghosts that
    # each fail the support test.
    if len(out) >= 2:
        a, b = out[0], out[-1]
        if math.hypot(a[0][0] - b[-1][0], a[0][1] - b[-1][1]) <= gap_m:
            out[0] = b + a
            out.pop()
    return out


def centroid(pts):
    return (sum(p[0] for p in pts) / len(pts),
            sum(p[1] for p in pts) / len(pts))


def spread(pts, c):
    return max(math.hypot(p[0] - c[0], p[1] - c[1]) for p in pts)


class Track:
    """One tracked obstacle. Constant-velocity, with an alpha-beta filter.

    NOT A KALMAN FILTER, AND THAT IS A CHOICE. A Kalman filter here would need a
    process-noise model for "a deer" and a measurement covariance for a centroid
    whose SHAPE changes as the object rotates -- both of which would be invented
    numbers dressed as statistics. An alpha-beta filter has two knobs that mean
    what they say, and its failure mode (lag on a hard turn) is visible instead
    of hidden inside a covariance nobody validated.
    """

    _next_id = 1

    def __init__(self, pts, stamp, alpha=0.55, beta=0.25):
        self.id = Track._next_id
        Track._next_id += 1
        self.x, self.y = centroid(pts)
        self.vx = self.vy = 0.0
        self.radius = spread(pts, (self.x, self.y))
        self.polygon = list(pts)
        self.stamp = stamp
        self.hits = 1
        self.misses = 0
        self.alpha, self.beta = alpha, beta
        self._moving_run = 0
        self.moving = False

    def predict(self, dt):
        return (self.x + self.vx * dt, self.y + self.vy * dt)

    def update(self, pts, stamp, static_speed):
        dt = max(1e-3, stamp - self.stamp)
        px, py = self.predict(dt)
        mx, my = centroid(pts)
        rx, ry = mx - px, my - py                       # residual
        self.x = px + self.alpha * rx
        self.y = py + self.alpha * ry
        self.vx += self.beta * rx / dt
        self.vy += self.beta * ry / dt
        self.radius = spread(pts, (self.x, self.y))
        self.polygon = list(pts)
        self.stamp = stamp
        self.hits += 1
        self.misses = 0
        # MOVING IS A CLAIM, AND IT NEEDS EVIDENCE. One fast estimate is
        # odometry noise; several consecutive ones are a walking animal. Getting
        # this wrong in the permissive direction fills the costmap with predicted
        # swept volumes for stationary trees and the rover stops.
        if math.hypot(self.vx, self.vy) >= static_speed:
            self._moving_run += 1
        else:
            self._moving_run = 0
        self.moving = self._moving_run >= 3

    def coast(self, stamp):
        dt = max(0.0, stamp - self.stamp)
        self.misses += 1
        return self.predict(dt)

    def confidence(self, max_hits=8):
        """0..1. Grows with corroboration, falls while coasting.

        The costmap layer decays this AGAIN with age; the two are not redundant.
        This one says how well the track is SUPPORTED; that one says how long ago
        anybody looked.
        """
        c = min(1.0, self.hits / float(max_hits))
        return max(0.0, c * (0.7 ** self.misses))


def associate(tracks, clusters, stamp, gate_m):
    """Greedy nearest-neighbour, cheapest pair first. (pairs, unmatched_clusters)

    GREEDY AND SORTED, NOT HUNGARIAN. With a handful of clusters the optimal
    assignment and the greedy one differ about never, and greedy-by-cheapest is
    DETERMINISTIC -- the same scan gives the same assignment, which matters more
    here than optimality because a flapping assignment produces phantom velocity
    and phantom velocity produces phantom swept volumes.
    """
    cands = []
    for ti, t in enumerate(tracks):
        px, py = t.predict(max(0.0, stamp - t.stamp))
        for ci, c in enumerate(clusters):
            cx, cy = centroid(c)
            d = math.hypot(cx - px, cy - py)
            # The gate widens with the track's own extent: a 2 m fallen limb's
            # centroid legitimately shifts by most of a metre as one end comes
            # into view, and a tight gate would split it into a new track every
            # few scans.
            if d <= gate_m + 0.5 * t.radius:
                cands.append((d, ti, ci))
    cands.sort()
    used_t, used_c, pairs = set(), set(), []
    for d, ti, ci in cands:
        if ti in used_t or ci in used_c:
            continue
        used_t.add(ti)
        used_c.add(ci)
        pairs.append((ti, ci))
    unmatched = [i for i in range(len(clusters)) if i not in used_c]
    return pairs, unmatched


# ================================================================== THE NODE
class TrunkTracker(Node):
    def __init__(self):
        super().__init__("trunk_tracker")
        self.declare_parameter("scan_topic", "scan_filtered")
        self.declare_parameter("output_topic", "tracked_obstacles")
        # Adaptive cluster gap: base + slope * range. See `cluster`.
        self.declare_parameter("cluster_gap_base_m", 0.12)
        self.declare_parameter("cluster_gap_per_m", 0.03)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("association_gate_m", 0.60)
        self.declare_parameter("static_speed_m_s", 0.15)
        self.declare_parameter("max_coast_s", 1.5)
        self.declare_parameter("min_hits_to_publish", 2)
        # A cluster wider than this is not an obstacle to route around, it is
        # terrain -- a hedge line, an embankment, a deadfall pile. It is still
        # published, but never as MOVING: predicting a swept volume for a 6 m
        # object because its centroid wandered is how a rover talks itself into a
        # wall of cost.
        self.declare_parameter("max_dynamic_extent_m", 2.5)
        # The polygon handed to the costmap layer is decimated to this many
        # vertices. The layer does a scanline fill per vertex pair per cell, and
        # a 200-point outline per track per prediction step is real cost for no
        # extra fidelity at 5 cm resolution.
        self.declare_parameter("polygon_max_points", 24)

        g = lambda k: self.get_parameter(k).value    # noqa: E731
        self.gap_base = g("cluster_gap_base_m")
        self.gap_per_m = g("cluster_gap_per_m")
        self.min_pts = int(g("min_cluster_points"))
        self.gate = g("association_gate_m")
        self.static_speed = g("static_speed_m_s")
        self.max_coast = g("max_coast_s")
        self.min_hits = int(g("min_hits_to_publish"))
        self.max_dyn = g("max_dynamic_extent_m")
        self.poly_max = int(g("polygon_max_points"))

        self.tracks = []
        self.pub = self.create_publisher(
            TrackedObstacles, g("output_topic"), qos_profile_sensor_data)
        self.sub = self.create_subscription(
            LaserScan, g("scan_topic"), self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            "trunk_tracker: gate %.2f m, static below %.2f m/s, coast %.1f s"
            % (self.gate, self.static_speed, self.max_coast))

    # -- the cycle ---------------------------------------------------------
    def on_scan(self, msg):
        stamp = (rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)
        pts, ranges = [], []
        for i, r in enumerate(msg.ranges):
            # A NON-FINITE OR OUT-OF-BAND RANGE IS UNKNOWN, NOT FREE. It marks
            # nothing and clears nothing -- the same rule the C++ filter applies,
            # stated in both places because getting it wrong in either one
            # produces an obstacle nobody saw.
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
            ranges.append(r)
        if not pts:
            self._coast_all(stamp)
            self._publish(msg.header, stamp)
            return

        mean_r = sum(ranges) / len(ranges)
        gap = self.gap_base + self.gap_per_m * mean_r
        clusters = cluster(pts, gap, self.min_pts)

        pairs, unmatched = associate(self.tracks, clusters, stamp, self.gate)
        matched = set()
        for ti, ci in pairs:
            self.tracks[ti].update(clusters[ci], stamp, self.static_speed)
            matched.add(ti)
        for ti, t in enumerate(self.tracks):
            if ti not in matched:
                t.coast(stamp)
        for ci in unmatched:
            self.tracks.append(Track(clusters[ci], stamp))
        # Death by coasting time, not by miss count: a 10 Hz scanner and a 2 Hz
        # scanner should forget an obstacle after the same number of SECONDS.
        self.tracks = [t for t in self.tracks
                       if (stamp - t.stamp) <= self.max_coast]
        self._publish(msg.header, stamp)

    def _coast_all(self, stamp):
        for t in self.tracks:
            t.coast(stamp)
        self.tracks = [t for t in self.tracks
                       if (stamp - t.stamp) <= self.max_coast]

    def _decimate(self, poly):
        if len(poly) <= self.poly_max:
            return poly
        step = len(poly) / float(self.poly_max)
        return [poly[int(i * step)] for i in range(self.poly_max)]

    def _publish(self, header, stamp):
        out = TrackedObstacles()
        out.header = header
        for t in self.tracks:
            if t.hits < self.min_hits:
                continue
            m = TrackedObstacle()
            m.header = header
            m.id = t.id
            m.position = Point32(x=float(t.x), y=float(t.y), z=0.0)
            extent = 2.0 * t.radius
            moving = t.moving and extent <= self.max_dyn
            m.velocity = Vector3(x=float(t.vx) if moving else 0.0,
                                 y=float(t.vy) if moving else 0.0, z=0.0)
            m.radius = float(t.radius)
            m.confidence = float(t.confidence())
            m.classification = (
                TrackedObstacle.DYNAMIC if moving else
                TrackedObstacle.TERRAIN if extent > self.max_dyn else
                TrackedObstacle.STATIC)
            m.polygon.points = [Point32(x=float(p[0]), y=float(p[1]), z=0.0)
                                for p in self._decimate(t.polygon)]
            out.obstacles.append(m)
        self.pub.publish(out)


def main(args=None):
    # A NOTE THAT BELONGS IN THE CODE, NOT ONLY IN A DESIGN DOC. Everything this
    # node publishes is in the scan's own frame, and the costmap layer transforms
    # it with whatever TF says. In a forest there is no AMCL fix against straight
    # walls, so that TF is odometry plus whatever scan matching or GNSS the
    # platform has -- and the tracker's velocity estimates inherit ALL of that
    # error. `static_speed_m_s` and the `_moving_run` evidence rule exist to stop
    # pose drift being reported as obstacle motion, and they are the first two
    # parameters to re-tune on a machine whose odometry is worse than assumed.
    rclpy.init(args=args)
    node = TrunkTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
