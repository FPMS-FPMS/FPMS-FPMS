#!/usr/bin/env bash
# Stage 40 - the ROS configuration layer: DDS profile, nav2/slam params,
# the patched behaviour tree, and the arena map.
set -euo pipefail
echo "--- 40-ros-layer"

H="${FPMS_HOME}"
OVL=/opt/fpms-os/overlay

# --- PyYAML, before anything needs it ---------------------------------------
#
# make_arena_map.py imports yaml at module scope and the map comparison below
# parses two documents. Stage 20 installs python3-yaml and verifies the import,
# so by the time stage 40 runs it is there -- but `--stage 40` against a chroot
# somebody built by hand skips stage 20 entirely, and under `set -euo pipefail`
# the failure is a bare ModuleNotFoundError traceback attributed to the map
# generator rather than to the missing package.
python3 -c 'import yaml' 2>/dev/null || {
    echo "FATAL: PyYAML is not importable in the chroot." >&2
    echo "Stage 20 installs it (apt python3-yaml). Run the full build, or" >&2
    echo "--stage 20 before --stage 40." >&2
    exit 1
}

# --- the DDS profile --------------------------------------------------------
#
# VERIFY IT PARSES, AND FAIL THE BUILD IF IT DOES NOT.
#
# Fast DDS does not fail loudly on a profile it cannot read. It logs at a level
# nobody reads and falls back to defaults -- which silently reintroduces the
# exact bug this file exists to remove: shared-memory segments that do not
# survive across process eras, so discovery succeeds and NO DATA FLOWS.
#
# A malformed profile would produce an image that looks correct in every way
# and reproduces "connected, subscribed, silent - no error anywhere".
PROFILE="$OVL/etc/fpms/fastdds_udp_only.xml"
[ -f "$PROFILE" ] || { echo "FATAL: $PROFILE missing" >&2; exit 1; }

# Checked through the PARSED TREE, not by grepping for a literal string.
# A substring test for "<useBuiltinTransports>false</useBuiltinTransports>"
# passes only for one exact spelling: reformat the file, split the tag over two
# lines, or put a space inside it, and the assertion fires on a profile that is
# perfectly correct -- while `<useBuiltinTransports> false </useBuiltinTransports>`
# would fail a test that Fast DDS itself would accept. Both directions are
# wrong answers.
#
# Read as BYTES and let ElementTree honour the XML declaration's encoding.
# build.sh runs every stage under `env -i ... LC_ALL=C`, and with LC_ALL
# explicitly set, PEP 538's C-locale coercion is skipped, so Python's text-mode
# default encoding here is ASCII. One non-ASCII character anywhere in a comment
# would raise UnicodeDecodeError and the build would report "the profile does
# not parse" about a file that parses fine.
python3 - "$PROFILE" <<'PY' || { echo "FATAL: the Fast DDS profile check failed. Fast DDS would SILENTLY ignore a bad profile and fall back to shared memory." >&2; exit 1; }
import sys, xml.etree.ElementTree as ET

path = sys.argv[1]
raw = open(path, "rb").read()
try:
    root = ET.fromstring(raw)
except ET.ParseError as e:
    sys.exit("    XML PARSE ERROR in %s: %s" % (path, e))

# The document declares a default namespace, so every tag arrives as
# "{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}useBuiltinTransports".
# Match on the local name.
def local(tag):
    return tag.rsplit("}", 1)[-1]

def find(name):
    return [e for e in root.iter() if local(e.tag) == name]

def text(e):
    return (e.text or "").strip()

# useBuiltinTransports=false is the load-bearing line. Listing a UDPv4
# transport alone leaves the builtin set -- which includes SHM -- in place,
# and Fast DDS still prefers SHM for same-host peers. A profile without it
# looks right and fixes nothing.
ubt = find("useBuiltinTransports")
if not ubt:
    sys.exit("    useBuiltinTransports is absent -- SHM would still be used")
bad = [text(e) for e in ubt if text(e).lower() != "false"]
if bad:
    sys.exit("    useBuiltinTransports is %r, not false -- SHM would still be used" % bad)

types = [text(e) for e in find("type")]
if "UDPv4" not in types:
    sys.exit("    no UDPv4 transport declared (found %r)" % types)
shm = [t for t in types if t.upper().startswith("SHM")]
if shm:
    sys.exit("    a shared-memory transport is declared explicitly: %r" % shm)

print("    DDS profile parses, UDPv4 only, builtin transports disabled")
PY

# --- nav2 / slam params -----------------------------------------------------
#
# THE PARAMS ARE IN THE PAYLOAD, NOT THE OVERLAY.
#
# This used to read only from $OVL/etc/fpms/{nav2,slam}/ -- directories that do
# not exist in the repository. Unmatched globs are left unexpanded, `[ -f ]` is
# false for the literal pattern, and both loops became silent no-ops. The
# result: /etc/fpms/nav2/ and /etc/fpms/slam/ were created EMPTY, and
#
#   fpms-nav2.service           ExecStart ... params_file:=/etc/fpms/nav2/nav2_params.yaml
#   fpms-slam-localization      ... params_file:=/etc/fpms/slam/mapper_params_localization.yaml
#   fpms-slam-mapping           ... params_file:=/etc/fpms/slam/mapper_params_mapping.yaml
#
# all pointed at nothing. fpms_slam_localization.launch.py at least refuses out
# loud ("refuse: params_file %r does not exist"), which is how this was found.
#
# The real files are rover/nav2/*.yaml and rover/slam/*.yaml, installed to
# $H/nav2 and $H/slam by stage 30. Copy from there; the overlay is applied
# afterwards so it can still override.
install -d -m 0755 /etc/fpms/nav2 /etc/fpms/slam

# Named, not globbed, on the payload side. $H/nav2 also holds arena_map.yaml,
# whose `image:` is a BARE FILENAME resolved next to the yaml -- copying it here
# would leave a map descriptor in /etc/fpms/nav2 with no .pgm beside it, which
# is a live trap for the next person who points map_server at the copy that
# happens to be in the config directory.
for f in nav2_params.yaml nav2_params_slam.yaml; do
    [ -f "$H/nav2/$f" ] && install -m 0644 "$H/nav2/$f" /etc/fpms/nav2/
done
for f in mapper_params_localization.yaml mapper_params_mapping.yaml; do
    [ -f "$H/slam/$f" ] && install -m 0644 "$H/slam/$f" /etc/fpms/slam/
done

# The overlay wins if a build ever grows one, so adding
# overlay/etc/fpms/nav2/nav2_params.yaml later still overrides the payload --
# as SPEC.md's ownership table expects -- without this stage changing again.
for f in "$OVL"/etc/fpms/nav2/*.yaml "$OVL"/etc/fpms/nav2/*.xml; do
    [ -f "$f" ] && install -m 0644 "$f" /etc/fpms/nav2/
done
for f in "$OVL"/etc/fpms/slam/*.yaml; do
    [ -f "$f" ] && install -m 0644 "$f" /etc/fpms/slam/
done
true   # the loops above end on a false [ -f ] whenever the last glob misses

# Verify what the units actually name. An empty /etc/fpms/nav2 is invisible
# until the first time somebody starts Nav2 on a competition floor.
PARAMS_MISSING=0
for p in /etc/fpms/nav2/nav2_params.yaml \
         /etc/fpms/nav2/nav2_params_slam.yaml \
         /etc/fpms/slam/mapper_params_localization.yaml \
         /etc/fpms/slam/mapper_params_mapping.yaml; do
    if [ -f "$p" ]; then
        python3 -c 'import sys,yaml; yaml.safe_load(open(sys.argv[1]))' "$p" \
            || { echo "FATAL: $p is not valid YAML" >&2; exit 1; }
        echo "    ok  $p"
    else
        echo "    MISSING: $p" >&2
        PARAMS_MISSING=1
    fi
done
if [ "$PARAMS_MISSING" = 1 ]; then
    echo "FATAL: a params file a unit ExecStart=s by absolute path is absent." >&2
    echo "These come from the stage 30 payload (\$H/nav2, \$H/slam). If stage 30" >&2
    echo "reported MISSING lines, fix that first -- this is the same cause." >&2
    exit 1
fi

# --- the arena map ----------------------------------------------------------
#
# arena_map.pgm is NOT committed to the repository -- only the .yaml and the
# generator are. Generate it and verify the emitted yaml AGREES WITH the
# committed one, so a drift in make_arena_map.py's constants cannot silently
# produce a map whose origin disagrees with the dashboard's arena frame.
#
# "AGREES WITH", NOT "IS BYTE-IDENTICAL TO". The previous `diff -q` could never
# pass, for two independent reasons, and would have failed every build here:
#
#   1. The committed arena_map.yaml is ~90 lines: the seven values plus the
#      derivation of the origin arithmetic, the frame argument, and why the
#      border is in the picture. make_arena_map.py emits a bare seven-line
#      safe_dump. The committed file says so itself in its own header --
#      "numbers only; expect comments to differ, values must not". A text diff
#      is comparing a document against a machine dump.
#   2. The committed file is CRLF. fpms-os/.gitattributes forces LF but does
#      not reach two levels up into rover/nav2/, and the workstation has
#      core.autocrlf=true. Stage 30 now normalises the payload, but parsing
#      both sides makes this check immune to line endings for good rather than
#      dependent on another stage having run first.
#
# So parse both and compare the VALUES, which is the only comparison that
# actually tests the thing we care about.
[ -f "$H/nav2/make_arena_map.py" ] || {
    echo "FATAL: $H/nav2/make_arena_map.py is absent." >&2
    echo "The arena map cannot be generated, and arena_map.yaml (which IS" >&2
    echo "committed) would then name a .pgm that does not exist -- map_server" >&2
    echo "fails to configure and Nav2's amcl mode never comes up. Stage 30" >&2
    echo "installs this file; see its MISSING lines." >&2
    exit 1
}

echo "--- generating the arena map"
# Clear first: a stale /tmp/arena_map.yaml from a previous --stage 40 that died
# mid-run would otherwise be compared as if it were this run's output.
rm -f /tmp/arena_map.pgm /tmp/arena_map.yaml /tmp/arena_map_zones.json
( cd "$H/nav2" && python3 make_arena_map.py --out /tmp/arena_map ) | sed 's/^/    /'

for f in /tmp/arena_map.pgm /tmp/arena_map.yaml /tmp/arena_map_zones.json; do
    [ -f "$f" ] || { echo "FATAL: make_arena_map.py did not write $f" >&2; exit 1; }
done

if [ -f "$H/nav2/arena_map.yaml" ]; then
    # `if cmd <<'PY'` / body / PY / `then`, NOT `cmd <<'PY' || {` split over
    # several lines: bash starts collecting a here-document at the very next
    # newline after the << operator, so a multi-line `|| { ... }` written after
    # it would be swallowed as the script's first lines and the Python would be
    # handed to the shell. It parses, and it does something entirely different.
    if python3 - /tmp/arena_map.yaml "$H/nav2/arena_map.yaml" <<'PY'
import sys, yaml

gen_path, committed_path = sys.argv[1], sys.argv[2]
gen = yaml.safe_load(open(gen_path, "rb"))
com = yaml.safe_load(open(committed_path, "rb"))

# Only the keys map_server actually reads. `mode` is included because trinary
# vs scale changes what the thresholds mean.
KEYS = ("image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh", "mode")

def close(a, b):
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(close(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        # 1e-9, not equality: -(10 * 0.01) and the literal -0.10 are the same
        # double today, but a comparison that depends on that staying true is
        # a trap for whoever next changes BORDER_PX or RESOLUTION.
        return abs(float(a) - float(b)) <= 1e-9
    return a == b

bad = []
for k in KEYS:
    g, c = gen.get(k), com.get(k)
    if g is None and c is None:
        continue
    if not close(g, c):
        bad.append("    %-16s generated=%r  committed=%r" % (k, g, c))

if bad:
    print("VALUES DISAGREE:", file=sys.stderr)
    print("\n".join(bad), file=sys.stderr)
    sys.exit(1)

print("    generated yaml agrees with the committed one on every map_server value")
PY
    then
        :
    else
        echo "FATAL: the generated arena_map.yaml does not agree with the committed file." >&2
        echo "make_arena_map.py's constants have drifted from the checked-in map," >&2
        echo "which means the map origin and the dashboard's arena frame disagree." >&2
        echo "Every waypoint Nav2 plans would land offset from where arena.ts says" >&2
        echo "it should -- silently, because nothing errors; it just drives to the" >&2
        echo "wrong spot. Take the SCRIPT's numbers: the .pgm was rasterised from" >&2
        echo "them, so it is the committed yaml that is stale." >&2
        exit 1
    fi
else
    echo "    WARNING: $H/nav2/arena_map.yaml is absent; nothing to check the" >&2
    echo "    generated map's constants against." >&2
fi

# Only the RASTER is copied back. The committed yaml carries the derivation and
# stays; replacing it with the generator's seven-line dump would delete the
# reasoning and leave nothing to check the next build against.
install -m 0644 -o "${FPMS_USER}" -g "${FPMS_USER}" \
    /tmp/arena_map.pgm "$H/nav2/arena_map.pgm"
echo "    $H/nav2/arena_map.pgm"

# The zone sidecar was being generated and thrown away. It is derived from the
# same ARENA_MM/Z_FRAC/M_FRAC constants as the map and as frontend arena.ts, so
# installing it means waypoint code reads the zone centres and start pose
# instead of retyping that arithmetic a third time. This is the ~/nav2/
# arena_zones.json that PI_FILE_INVENTORY.md records as existing only on the
# old Pi -- unlike the RKNN model, it is pure arithmetic and reconstructs
# exactly. Installed under the name every document and fpms-tf.service's header
# already use.
install -m 0644 -o "${FPMS_USER}" -g "${FPMS_USER}" \
    /tmp/arena_map_zones.json "$H/nav2/arena_zones.json"
echo "    $H/nav2/arena_zones.json  (regenerated, not the old Pi's copy)"

# slam/maps stays EMPTY on purpose. fpms_room.posegraph/.data describe one
# physical room and cannot be baked into a shared image. The localisation unit
# refuses to start without them, which is correct.
install -d -o "${FPMS_USER}" -g "${FPMS_USER}" "$H/slam/maps"

# --- the patched behaviour tree ---------------------------------------------
#
# Stock Nav2's BackUp recovery uses backup_speed="0.025" m/s -- under a fifth
# of this rover's firmware velocity floor -- and NO PARAMETER CAN OVERRIDE IT,
# because the speed lives in the XML. Every BackUp recovery with the stock file
# is a guaranteed stall-then-lurch.
BT_DIR="/opt/ros/humble/share/nav2_bt_navigator/behavior_trees"
BT_SPEED="0.18"
BT_PATCHED=0

# Verified present in Humble's nav2_bt_navigator. Kept as the FIRST place to
# look rather than the only one: nav2_bringup ships copies of the same trees,
# and an upstream that moves the directory would otherwise take the whole
# recovery fix out silently -- the old code's guard was a bare
# `[ -f "$BT_SRC" ] && ...`, so a moved file was a skip with no output at all.
bt_locate() {  # bt_locate <filename> -> path on stdout, non-zero if not found
    local name="$1" hit
    if [ -f "$BT_DIR/$name" ]; then echo "$BT_DIR/$name"; return 0; fi
    hit="$(find /opt/ros/humble/share -maxdepth 3 -name "$name" -type f 2>/dev/null | head -1)"
    [ -n "$hit" ] || return 1
    echo "$hit"
}

patch_bt() {  # patch_bt <stock-filename> <dest>
    local src dst="$2" was
    src="$(bt_locate "$1")" || {
        echo "    NOT FOUND anywhere under /opt/ros/humble/share: $1" >&2
        return 1
    }
    [ "$src" = "$BT_DIR/$1" ] || echo "    NOTE: found $1 at $src, not $BT_DIR" >&2
    # Match ANY backup_speed value, not the literal 0.025.
    #
    # A sed for one hardcoded number is a check that silently stops working:
    # if upstream ever ships 0.05 (Iron did), the substitution matches nothing,
    # the file is copied through unpatched, and the build prints
    # "patched BT: 0.025 -> 0.18" about a file it did not touch. Then BackUp
    # stalls forever and the build log says it was fixed.
    was="$(grep -o 'backup_speed="[^"]*"' "$src" | head -1 || true)"
    sed -E 's/backup_speed="[^"]*"/backup_speed="'"$BT_SPEED"'"/g' "$src" > "$dst"
    chmod 0644 "$dst"

    # Prove it. Both directions: the new value is present, and no value below
    # the firmware floor survives anywhere in the file. On failure DELETE the
    # output -- an unpatched tree sitting at a path named fpms_bt_* is worse
    # than no file, because the next person to read the filename will believe
    # it, and nothing downstream would be able to tell.
    if ! grep -q "backup_speed=\"$BT_SPEED\"" "$dst"; then
        rm -f "$dst"
        echo "    WARNING: $1 declares no backup_speed at all -- upstream's" >&2
        echo "    recovery XML has changed shape. The BackUp speed is NOT patched." >&2
        echo "    Re-read $src before relying on recovery." >&2
        return 1
    fi
    if grep -o 'backup_speed="[^"]*"' "$dst" | grep -qv "\"$BT_SPEED\""; then
        echo "    WARNING: $dst still contains a backup_speed other than $BT_SPEED" >&2
        rm -f "$dst"
        return 1
    fi
    echo "    patched BT: ${was:-<none>} -> backup_speed=\"$BT_SPEED\" (the firmware floor)"
    echo "                $dst"
    return 0
}

# Regenerated every run, deliberately. The old guard was
# `[ ! -f "$dst" ]`, which meant a re-run with a corrected speed, or after a
# Nav2 package upgrade, kept whatever the first build happened to write. Clear
# the two we own first, by name, so a --stage 40 that can no longer find the
# stock tree cannot leave last run's copy behind looking current.
rm -f /etc/fpms/nav2/fpms_bt_navigate_to_pose.xml \
      /etc/fpms/nav2/fpms_bt_navigate_through_poses.xml

if patch_bt navigate_to_pose_w_replanning_and_recovery.xml \
        /etc/fpms/nav2/fpms_bt_navigate_to_pose.xml; then
    BT_PATCHED=1
fi
patch_bt navigate_through_poses_w_replanning_and_recovery.xml \
        /etc/fpms/nav2/fpms_bt_navigate_through_poses.xml || true

if [ "$BT_PATCHED" = 1 ]; then
    # A patched tree nothing points at is dead weight. nav2_params.yaml ships
    # with default_nav_to_pose_bt_xml COMMENTED OUT, aimed at a path that has
    # never existed (/home/ubuntu/fpms_nav2/...), with a header saying to
    # "uncomment and repoint these after making the BackUp fix described
    # above". This stage IS that fix, so it does the repointing too -- on the
    # INSTALLED copy under /etc/fpms/nav2 only. The repo's file keeps its
    # instructions, and the image ships a stack that actually uses the tree it
    # went to the trouble of patching.
    P=/etc/fpms/nav2/nav2_params.yaml
    sed -i -E \
        -e 's|^([[:space:]]*)#[[:space:]]*default_nav_to_pose_bt_xml:.*|\1default_nav_to_pose_bt_xml: "/etc/fpms/nav2/fpms_bt_navigate_to_pose.xml"|' \
        "$P"
    if [ -f /etc/fpms/nav2/fpms_bt_navigate_through_poses.xml ]; then
        sed -i -E \
            -e 's|^([[:space:]]*)#[[:space:]]*default_nav_through_poses_bt_xml:.*|\1default_nav_through_poses_bt_xml: "/etc/fpms/nav2/fpms_bt_navigate_through_poses.xml"|' \
            "$P"
    fi
    if grep -qE '^[[:space:]]*default_nav_to_pose_bt_xml:' "$P"; then
        echo "    $P now points bt_navigator at the patched tree"
    else
        echo "    WARNING: could not activate default_nav_to_pose_bt_xml in $P." >&2
        echo "    The patched tree exists at /etc/fpms/nav2/fpms_bt_navigate_to_pose.xml" >&2
        echo "    but bt_navigator will still load Nav2's stock file, so BackUp" >&2
        echo "    recovery WILL stall. Set the key by hand." >&2
    fi
    # A params file bt_navigator cannot parse is worse than an unpatched tree.
    python3 -c 'import sys,yaml; yaml.safe_load(open(sys.argv[1]))' "$P" \
        || { echo "FATAL: $P stopped being valid YAML after the BT rewrite" >&2; exit 1; }
else
    cat >&2 <<EOF

    ##################################################################
     THE BACKUP RECOVERY IS NOT PATCHED.

     $BT_DIR/navigate_to_pose_w_replanning_and_recovery.xml
     could not be read or carries no backup_speed.

     Nav2's stock tree drives BackUp at 0.025 m/s, under a fifth of
     this rover's firmware velocity floor, and NO PARAMETER CAN
     OVERRIDE IT -- the speed lives in the XML. Every BackUp recovery
     will stall, lurch, and time out having not backed up.

     Check that ros-humble-navigation2 / ros-humble-nav2-bt-navigator
     installed (stage 10), then re-run: sudo ./build.sh --stage 40
    ##################################################################

EOF
fi

# --- record what we actually got --------------------------------------------
set +u; source /opt/ros/humble/setup.bash; set -u
{
    echo "ros_distro=${ROS_DISTRO}"
    echo "nav2=$(dpkg-query -W -f='${Version}' ros-humble-navigation2 2>/dev/null || echo absent)"
    echo "slam_toolbox=$(dpkg-query -W -f='${Version}' ros-humble-slam-toolbox 2>/dev/null || echo absent)"
    echo "rosbridge=$(dpkg-query -W -f='${Version}' ros-humble-rosbridge-suite 2>/dev/null || echo absent)"
    echo "rmw_fastrtps=$(dpkg-query -W -f='${Version}' ros-humble-rmw-fastrtps-cpp 2>/dev/null || echo absent)"
    echo "bt_backup_speed=$([ "$BT_PATCHED" = 1 ] && echo "$BT_SPEED" || echo UNPATCHED)"
} | tee /etc/fpms/ros-versions.txt

echo "--- 40-ros-layer OK"
