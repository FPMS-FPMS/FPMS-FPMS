#!/usr/bin/env bash
# Stage 70 - update policy: make `apt upgrade` SAFE on this specific board.
#
# WHAT THIS STAGE OWNS
# ====================
#   /etc/apt/preferences.d/fpms-kernel            (shipped in the overlay)
#   /etc/apt/preferences.d/fpms-numpy-ceiling     (shipped in the overlay)
#   /etc/apt/apt.conf.d/70fpms-update-policy      (shipped in the overlay)
#   /etc/apt/apt.conf.d/71fpms-unattended-upgrades(shipped in the overlay)
#   /etc/apt/preferences.d/fpms-kernel-installed  GENERATED HERE - it names
#                                                 versions that only exist
#                                                 inside the image
#   the dpkg holds on the same package set
#   the proof that all of the above actually does what it claims
#
# It does NOT own /etc/apt/preferences.d/fpms-numpy (scripts/20-python-deps.sh
# writes that; this stage only re-reads it and reports), and it does not touch
# any systemd unit, /usr/local/bin/fpms-update, or scripts/60-enable-units.sh.
#
# THE DANGER THIS STAGE EXISTS FOR
# ================================
# The base image is Joshua Riek's ubuntu-rockchip Ubuntu 22.04 arm64 and its
# kernel is a Rockchip BSP kernel. rknpu (the NPU), the Mali stack and bcmdhd
# (the WiFi chip) exist ONLY there. A routine `apt upgrade` that pulls a
# generic linux-image-* gives you a rover with no NPU, no WiFi and possibly no
# boot - silently, on a machine with no monitor attached. The long form of the
# argument, and of every design decision below, is in the files themselves:
# read /etc/apt/preferences.d/fpms-kernel before changing anything here.
#
# WHERE IT SITS IN THE BUILD
# ==========================
# build.sh globs scripts/[0-9]*.sh, so 70 runs after 60-enable-units and
# before 90-finalise. That ordering is required in one direction: the overlay
# copy happens in stage 50, so by the time this stage runs the four static
# files are already on disk and this stage's job is to VERIFY them, complete
# them with the parts that can only be known from inside the image, and prove
# the result. Stage 90 runs `apt-get clean` and nothing else, so nothing after
# this point can be broken by a pin.
#
# TWO BUG CLASSES THIS BUILD HAS REPEATEDLY HIT, AVOIDED THROUGHOUT
# =================================================================
#   - Under `set -o pipefail`, `cmd | grep -q`, `cmd | awk ...exit` and
#     `cmd | head -N` return 141 from SIGPIPE, and a plain assignment ADOPTS
#     that status (a `local x="$(...)"` would hide it instead, which is
#     worse). There is not one such pipeline in this file: every capture is a
#     bare command substitution, and every test is done afterwards with `case`
#     against the captured text.
#   - An exit code is never trusted where an outcome can be checked. Every
#     file written here is read back; every pin is confirmed through
#     `apt-cache policy`; and the whole policy is exercised with
#     `apt-get -s dist-upgrade --ignore-hold`, which is the only test that
#     proves the PIN is doing the work rather than the hold.
set -euo pipefail
echo "--- 70-update-policy"

export DEBIAN_FRONTEND=noninteractive

fail() { echo ""; echo "FATAL: 70-update-policy: $*" >&2; echo ""; exit 1; }
note() { echo "    $*"; }
warn() { echo "    WARNING: $*" >&2; }

PREF_D=/etc/apt/preferences.d
CONF_D=/etc/apt/apt.conf.d
GEN="${PREF_D}/fpms-kernel-installed"

# ---------------------------------------------------------------------------
# 1. the four static files must have LANDED, with their content
# ---------------------------------------------------------------------------
#
# Existence is not enough. `cp` exiting 0 having written a truncated file, an
# overlay staged from a half-checked-out tree, or a file renamed at some point
# to something apt silently ignores, all produce a present-but-useless policy
# - and this is the one stage in the build whose entire product is a set of
# text files nobody looks at again until an operator types `apt upgrade` on a
# rover. Check for a string that could only come from the intended file.
check_landed() {
    local path="$1" sentinel="$2" body
    [ -f "$path" ] || fail \
"${path} is missing.

scripts/50-overlay.sh copies overlay/* onto /, so this file should have
arrived there. Either the overlay was staged from a tree that does not have
it, or stage 50 did not run in this image. Resume with:
    sudo ./build.sh --from 50"
    body="$(cat "$path" 2>/dev/null || true)"
    case "$body" in
        *"$sentinel"*) : ;;
        *) fail \
"${path} exists but does not contain the expected text
    '${sentinel}'
so it is not the file this stage was written against. Do not ship it: an apt
policy that is half of what it claims is worse than none, because it reads
like protection." ;;
    esac
    # A carriage return would be inherited from the Windows workstation these
    # files are authored on. scripts/50-overlay.sh sweeps CRLF out of
    # /etc/fpms, /etc/systemd, /etc/sudoers.d and the udev rules - /etc/apt is
    # NOT on that list, so this is the only place it gets checked. A trailing
    # \r on "Pin-Priority: -1" makes the value unparseable, and a broken
    # preferences file does not break one pin: it breaks EVERY apt operation
    # on the image. Detect, do not repair - .gitattributes says `* text
    # eol=lf`, so a CR here means something went wrong upstream of the build
    # and quietly fixing it would hide that.
    case "$body" in
        *$'\r'*) fail \
"${path} contains a carriage return.

Check .gitattributes (it says '* text eol=lf') and do not paste this file back
from a Windows editor. A CR in an apt preferences or apt.conf file breaks every
apt command on the image, with an error nobody connects back to this stage." ;;
    esac
    # Modes are NOT inherited from a Windows checkout. `cp -a` in stage 50
    # preserves the SOURCE mode, and every file on a drvfs/9p mount reads back
    # world-writable. A world-writable file in /etc/apt is not refused by apt
    # the way sudo refuses /etc/sudoers.d - it is simply obeyed. Set it here,
    # then confirm.
    chmod 0644 "$path"
    chown root:root "$path"
    local mode
    mode="$(stat -c '%a %U' "$path" 2>/dev/null || true)"
    [ "$mode" = "644 root" ] || warn "${path} is '${mode}', expected '644 root'"
    return 0
}

check_landed "$PREF_D/fpms-kernel"                  "FPMS-OS kernel/bootloader freeze"
check_landed "$PREF_D/fpms-numpy-ceiling"           "FPMS-OS numpy 2.x ceiling"
check_landed "$CONF_D/70fpms-update-policy"         "APT::Periodic::Unattended-Upgrade"
check_landed "$CONF_D/71fpms-unattended-upgrades"   "Unattended-Upgrade::Automatic-Reboot"
note "the four static policy files are present, 0644 root:root"

# stage 20's half of the numpy policy. Not ours to write, and its absence is
# not fatal here - but it IS the file this image's numpy protection is built
# on, so an image without it should say so in the build log rather than in a
# traceback from tf_transformations three weeks later.
if [ -f "$PREF_D/fpms-numpy" ]; then
    note "stage 20's ${PREF_D}/fpms-numpy is present (the 1.x floor)"
else
    warn "${PREF_D}/fpms-numpy is MISSING - scripts/20-python-deps.sh writes it."
    warn "The ceiling in fpms-numpy-ceiling still blocks 2.x and 3.x, but the"
    warn "1.x pin that keeps apt WANTING the 1.x is gone. Check stage 20."
fi

# ---------------------------------------------------------------------------
# 2. refresh the lists, so the simulations below mean something
# ---------------------------------------------------------------------------
#
# Not fatal. The pins do not need the network; only the proofs at the end do,
# and running them against lists that are a few days old still answers the
# question this stage asks ("would apt move the kernel?"). Say which case we
# are in so a clean run is distinguishable from a run that proved less.
LISTS_FRESH=0
if apt-get update -qq -o Acquire::Retries=3; then
    LISTS_FRESH=1
    note "apt lists refreshed"
else
    warn "apt-get update failed; the simulations below run on the lists already"
    warn "in the image. A PASS from them is still meaningful; a clean list is"
    warn "just a stronger test."
fi

# ---------------------------------------------------------------------------
# 3. which packages are actually here
# ---------------------------------------------------------------------------
#
# THE NAMES ARE DISCOVERED, NOT GUESSED. The base .img is not something this
# repository can read, so no file in it can honestly state that the kernel
# package is called `linux-image-5.10.160-rockchip` rather than
# `linux-image-rockchip` or anything else. dpkg knows. Ask it, print what it
# said into the build log - which is the only place this will ever be recorded
# - and pin exactly that.
#
# `${Status}` rather than `${db:Status-Abbrev}`: same information, and the
# long form has been in dpkg-query forever, so this cannot become the one line
# that fails on a base image with an older dpkg.
DPKG_LIST="$(dpkg-query -W -f='${Status}|${Package}|${Version}\n' 2>/dev/null || true)"
[ -n "$DPKG_LIST" ] || fail \
"dpkg-query listed no packages at all. This chroot's dpkg database is not
readable, and nothing this stage does afterwards would mean anything."

# The glob set, and why each entry is here:
#
#   linux-image-*      the kernel itself, BSP flavour AND every generic Ubuntu
#   linux-headers-*    kernel. This is the whole point of the stage.
#   linux-modules-*
#   linux-generic*     the METApackages. This is how a generic kernel arrives
#   linux-virtual*     as somebody's dependency rather than as a direct
#   linux-lowlatency*  install.
#   linux-rockchip*    Riek's flavour naming, both orders, because this repo
#   linux-*-rockchip*  cannot confirm which one the base image uses.
#   linux-firmware     may own the bcmdhd blobs. UNVERIFIED - see the note in
#                      /etc/apt/preferences.d/fpms-kernel.
#   u-boot*            postinsts write raw boot sectors.
#   flash-kernel*      rewrites boot config on kernel changes.
#   *rockchip*         rockchip-multimedia-config, librockchip-mpp, ...
#   libmali* mali-*    the vendor GPU blobs, matched to the BSP driver.
#   *bcmdhd*           speculative; a no-op if no such package exists.
#
# Deliberately NOT here: linux-libc-dev (userspace headers - freezing it
# blocks unrelated builds and it cannot break the board) and initramfs-tools
# (freezing it would be defensible, but it is a normal Ubuntu package whose
# upgrades are not the failure mode this stage is about; see docs/UPDATING.md).
#
# ONE definition of the set, used twice: to decide what to freeze here, and to
# decide what counts as a violation in the proofs at the end. Two copies of a
# pattern list is how a proof quietly stops covering the thing it names.
#
# It also has to be a name test rather than a lookup in the installed set,
# because the most dangerous package of all - a generic `linux-image-*` - is
# by definition NOT installed today. A proof that only watched the installed
# packages could not see it arriving.
is_frozen_name() {
    case "$1" in
        linux-image-*|linux-headers-*|linux-modules-*|\
        linux-generic*|linux-virtual*|linux-lowlatency*|\
        linux-rockchip*|linux-*-rockchip*|linux-firmware|\
        u-boot*|flash-kernel*|\
        *rockchip*|libmali*|mali-*|*bcmdhd*) return 0 ;;
    esac
    return 1
}

HOLD_NAMES=""
HOLD_LINES=""
n_hold=0
while IFS='|' read -r st pkg ver; do
    [ "$st" = "install ok installed" ] || continue
    [ -n "$pkg" ] || continue
    [ -n "$ver" ] || continue
    if ! is_frozen_name "$pkg"; then continue; fi
    HOLD_NAMES="${HOLD_NAMES}${pkg} "
    HOLD_LINES="${HOLD_LINES}${pkg}|${ver}"$'\n'
    n_hold=$(( n_hold + 1 ))
done <<EOF
$DPKG_LIST
EOF

# A heredoc, not a pipe: `... | while read` would run the loop in a SUBSHELL
# and every variable it set would be gone by the time this line executes. That
# is the same class of silent nothing-happened the rest of this file is
# written against.

[ "$n_hold" -gt 0 ] || fail \
"no kernel, bootloader or Rockchip BSP package is installed under any of the
names this stage knows about.

That is not a result this build can ship on. Either the base image is not the
ubuntu-rockchip image this whole project is built around, or its packages are
named something nobody here has seen. Look, then widen the glob list above:

    dpkg-query -W -f='\${Package}\\n' | grep -iE 'linux|u-boot|rockchip|mali'

Do NOT work around this by deleting the check. An image whose kernel is
'protected' by a pin that matches nothing is exactly the silent success this
project bans."

note "packages to freeze (${n_hold}):"
while IFS='|' read -r pkg ver; do
    [ -n "$pkg" ] || continue
    printf '        %-44s %s\n' "$pkg" "$ver"
done <<EOF
$HOLD_LINES
EOF

# ---------------------------------------------------------------------------
# 4. the specific-form pins
# ---------------------------------------------------------------------------
#
# /etc/apt/preferences.d/fpms-kernel already gives EVERY version of these
# packages priority -1 through a general-form (`Pin: release *`) record. This
# file gives the ONE version that is installed a positive priority through a
# specific-form (`Pin: version`) record, and apt_preferences(5) resolves
# specific before general regardless of filename order. Net effect:
#
#     the version this image shipped with   1000  <- the candidate
#     every other version                     -1  <- never installable
#
# WHY 1000 AND NOT 1001, WHICH IS WHAT STAGE 20 USES FOR NUMPY. Above 1000 is
# what permits a DOWNGRADE. Stage 20 wants that: if something has already
# dragged a numpy 2.x in, apt must be willing to go backwards. Here the
# opposite is true. If an operator has deliberately, knowingly upgraded the
# kernel (the procedure is in docs/UPDATING.md), a 1001 pin would make the
# very next `apt install anything` propose silently DOWNGRADING the running
# kernel back - a kernel downgrade nobody asked for, on a headless board, is
# the same catastrophe this stage exists to prevent, arriving through the
# door marked "safety". 1000 refuses to move forward without permitting a
# reverse. Deliberate difference; do not "make it consistent".
#
# Written through a .tmp first: a file in preferences.d whose name has an
# extension other than .pref is ignored by apt, so a half-written file is
# invisible to apt rather than a parse error that breaks every apt command in
# the image.
{
    echo "# FPMS-OS: the kernel/bootloader/BSP versions THIS IMAGE WAS BUILT WITH."
    echo "#"
    echo "# GENERATED by scripts/70-update-policy.sh at build time. Do not edit by"
    echo "# hand and do not copy between images: it names exact versions, and a"
    echo "# version string that is not installed here pins nothing at all."
    echo "#"
    echo "# Read with /etc/apt/preferences.d/fpms-kernel, which explains the whole"
    echo "# design. In short: that file is general-form and gives every version -1;"
    echo "# this file is specific-form and rescues exactly the installed one at"
    echo "# 1000, and apt resolves specific before general whatever the filenames."
    echo "#"
    echo "# AFTER A DELIBERATE KERNEL CHANGE this file is stale - it names a version"
    echo "# that is no longer installed, so the new kernel drops to -1 and shows"
    echo "# 'Candidate: (none)'. It stays installed and the rover still boots, but"
    echo "# regenerate it. docs/UPDATING.md has the one-liner."
    echo "#"
    echo "# Built: $(date -u '+%Y-%m-%dT%H:%M:%SZ')"
    echo ""
    while IFS='|' read -r pkg ver; do
        [ -n "$pkg" ] || continue
        echo "Explanation: FPMS-OS: frozen at the version this image shipped with."
        echo "Package: ${pkg}"
        echo "Pin: version ${ver}"
        echo "Pin-Priority: 1000"
        echo ""
    done <<EOF
$HOLD_LINES
EOF
} > "${GEN}.tmp"

mv -f "${GEN}.tmp" "$GEN"
chmod 0644 "$GEN"
chown root:root "$GEN"

# Read it back. `mv` and the redirect can both succeed on a full filesystem
# and leave a truncated file behind.
GEN_BODY="$(cat "$GEN" 2>/dev/null || true)"
case "$GEN_BODY" in
    *"Pin-Priority: 1000"*) note "wrote ${GEN} (${n_hold} pinned versions)" ;;
    *) fail "${GEN} was written but contains no 'Pin-Priority: 1000' record." ;;
esac

# ---------------------------------------------------------------------------
# 5. the dpkg holds
# ---------------------------------------------------------------------------
#
# Belt and braces, and NOT the mechanism - see the long note in
# /etc/apt/preferences.d/fpms-kernel. The hold is the half an operator can
# SEE (`apt-mark showhold` prints the protected set in one line, which a pin
# file does not), and it makes apt refuse rather than silently keep back. The
# pin is the half that survives --allow-change-held-packages and a reimaged
# rootfs.
#
# Unquoted on purpose: HOLD_NAMES is a space-separated list and word splitting
# is what turns it into argv here. The names come from dpkg, not from a human,
# so there is nothing in them to split badly.
# shellcheck disable=SC2086
if apt-mark hold $HOLD_NAMES >/dev/null 2>&1; then
    note "apt-mark hold applied"
else
    warn "apt-mark hold returned non-zero; checking what actually took"
fi

# Never trust that exit code: check the outcome. apt-mark can report success
# per-package and still skip one it could not parse.
HELD_NOW="$(apt-mark showhold 2>/dev/null || true)"
missing_hold=""
for p in $HOLD_NAMES; do
    found_hold=0
    while IFS= read -r h; do
        if [ "$h" = "$p" ]; then found_hold=1; fi
    done <<EOF
$HELD_NOW
EOF
    # An `if`, not `[ ... ] || missing_hold=...`. A test as the LAST statement
    # of a loop body hands the loop the test's status, and under `set -e` that
    # is fatal at a distance. It happens to be harmless with `||`; it is not
    # worth leaving a shape in this file that has to be re-reasoned about.
    if [ "$found_hold" != 1 ]; then
        missing_hold="${missing_hold} ${p}"
    fi
done
if [ -n "$missing_hold" ]; then
    warn "apt-mark showhold does not list:${missing_hold}"
    warn "The pin in ${PREF_D}/fpms-kernel still covers them - the proof below"
    warn "runs with --ignore-hold precisely so it does not depend on holds."
else
    note "all ${n_hold} packages are held at the dpkg level too"
fi

# ---------------------------------------------------------------------------
# 6. PROVE IT - the pin, per package
# ---------------------------------------------------------------------------
#
# The property that matters is not "a pin file exists", it is "apt has nothing
# it wants to install for this package". That is exactly `Installed:` ==
# `Candidate:` in apt-cache policy, so read those two lines back and compare
# them. A `Candidate: (none)` here would mean the general -1 landed on the
# installed version too - i.e. the specific/general ordering this design rests
# on is not behaving as apt_preferences(5) describes - and that must fail the
# build rather than ship.
policy_bad=0
for p in $HOLD_NAMES; do
    POL="$(apt-cache policy "$p" 2>&1 || true)"
    inst=""
    cand=""
    while IFS= read -r line; do
        case "$line" in
            *"Installed: "*) [ -n "$inst" ] || inst="${line##*Installed: }" ;;
            *"Candidate: "*) [ -n "$cand" ] || cand="${line##*Candidate: }" ;;
        esac
    done <<EOF
$POL
EOF
    if [ -z "$inst" ] || [ -z "$cand" ]; then
        warn "apt-cache policy ${p} reported no Installed/Candidate line"
        policy_bad=1
    elif [ "$inst" = "$cand" ]; then
        printf '        %-44s %s\n' "$p" "candidate == installed"
    else
        echo "    FAULT: ${p}: installed ${inst}, candidate ${cand}" >&2
        policy_bad=1
    fi
done

[ "$policy_bad" = 0 ] || fail \
"at least one frozen package still has a candidate that is not what is
installed, or apt could not report on it at all. The pin is not doing what
this stage claims. Look at:
    apt-cache policy <package>
    cat ${PREF_D}/fpms-kernel ${GEN}
An image shipped in this state would upgrade its own kernel on the first
\`apt upgrade\` an operator runs."

note "every frozen package: apt's candidate is the installed version"

# ---------------------------------------------------------------------------
# 7. PROVE IT - the whole policy, end to end
# ---------------------------------------------------------------------------
#
# `--ignore-hold` is the important flag and the reason this test is worth
# running at all. Without it, a PASS could come entirely from the dpkg holds
# and would say nothing about the pin - and the pin is the half that survives
# a reimaged rootfs. With it, apt is explicitly told to disregard every hold,
# so anything that still refuses to move is the preferences file doing the
# work.
#
# `-s` is a simulation: it resolves and prints, and changes nothing.
simulate() {
    local what="$1" out line verb rest name found=""
    shift
    out="$(apt-get -s "$@" 2>&1 || true)"
    if [ -z "$out" ]; then
        warn "apt-get -s ${what} produced no output at all; this proof is inconclusive"
        return 0
    fi
    while IFS= read -r line; do
        case "$line" in
            "Inst "*|"Remv "*) : ;;
            *) continue ;;
        esac
        verb="${line%% *}"
        rest="${line#* }"
        name="${rest%% *}"
        # is_frozen_name, not "is it in HOLD_NAMES": an `Inst
        # linux-image-generic` is the disaster this stage exists for and it is
        # not in the installed set, so a membership test would miss exactly
        # the case that matters most.
        if is_frozen_name "$name"; then
            found="${found}        ${verb} ${name}"$'\n'
        fi
    done <<EOF
$out
EOF
    if [ -n "$found" ]; then
        echo "    FAULT: apt-get -s ${what} would touch frozen packages:" >&2
        printf '%s' "$found" >&2
        SIM_BAD=1
    else
        note "apt-get -s ${what}: touches no frozen package"
    fi
    return 0
}

SIM_BAD=0
simulate "upgrade"                     upgrade
simulate "dist-upgrade"                dist-upgrade
simulate "dist-upgrade --ignore-hold"  dist-upgrade --ignore-hold

[ "$SIM_BAD" = 0 ] || fail \
"apt still wants to install or remove a frozen package.

If this fired on the --ignore-hold run only, the dpkg holds are working and
the PIN is not - which means the image is protected by state that a reimage or
an --allow-change-held-packages would drop. Either way, look at
${PREF_D}/fpms-kernel and ${GEN} before shipping. This is precisely the
silent kernel swap this stage exists to prevent."

# ---------------------------------------------------------------------------
# 8. PROVE IT - the apt.conf half
# ---------------------------------------------------------------------------
#
# `apt-config dump` is both the readback and the syntax check: a malformed
# file in apt.conf.d breaks EVERY apt operation on the image, with an error
# nobody connects back to this stage.
CFG="$(apt-config dump 2>&1 || true)"
[ -n "$CFG" ] || fail \
"apt-config dump produced nothing. apt cannot read its own configuration,
which means one of the files in ${CONF_D} does not parse. Every apt command on
this image is broken until it is fixed."

cfg_expect() {
    local key="$1" want="$2"
    case "$CFG" in
        *"${key} \"${want}\";"*) note "${key} = ${want}" ; return 0 ;;
    esac
    warn "apt-config does not report ${key} = ${want}."
    warn "  Check ${CONF_D}/70fpms-update-policy and 71fpms-unattended-upgrades,"
    warn "  and remember apt.conf.d is read in alphanumeric order - a file"
    warn "  sorting after 71 can overwrite any scalar set there."
    CFG_BAD=1
    return 0
}

CFG_BAD=0
cfg_expect "APT::Periodic::Update-Package-Lists"          "0"
cfg_expect "APT::Periodic::Download-Upgradeable-Packages" "0"
cfg_expect "APT::Periodic::Unattended-Upgrade"            "0"
cfg_expect "Unattended-Upgrade::Automatic-Reboot"         "false"
cfg_expect "Unattended-Upgrade::InstallOnShutdown"        "false"
cfg_expect "Unattended-Upgrade::Remove-Unused-Kernel-Packages" "false"

# The allowed-origins list is the one place a mistake is INVISIBLE rather than
# loud: apt.conf lists APPEND across files, so a missing `#clear` leaves
# whatever 50unattended-upgrades set still in the list and the policy is wider
# than the file appears to say. Assert the property that actually matters -
# every entry is a security pocket - rather than assuming #clear worked.
origins_n=0
origins_bad=""
while IFS= read -r line; do
    case "$line" in
        "Unattended-Upgrade::Allowed-Origins::"*|"Unattended-Upgrade::Origins-Pattern::"*) : ;;
        *) continue ;;
    esac
    origins_n=$(( origins_n + 1 ))
    case "$line" in
        *security*) : ;;
        *) origins_bad="${origins_bad}      ${line}"$'\n' ;;
    esac
done <<EOF
$CFG
EOF

if [ "$origins_n" = 0 ]; then
    warn "unattended-upgrades has NO allowed origins in the merged config."
    warn "That is fail-closed (it would upgrade nothing), but it is not what"
    warn "${CONF_D}/71fpms-unattended-upgrades says. Check it parsed."
elif [ -n "$origins_bad" ]; then
    echo "    FAULT: non-security origin(s) allowed for unattended-upgrades:" >&2
    printf '%s' "$origins_bad" >&2
    echo "    A '#clear' directive did not take effect, or another file in" >&2
    echo "    ${CONF_D} adds to the list after 71." >&2
    CFG_BAD=1
else
    note "unattended-upgrades: ${origins_n} allowed origin(s), all security-only"
fi

[ "$CFG_BAD" = 0 ] || fail \
"the merged apt configuration is not the policy this stage installed. The
files are in ${CONF_D}; \`apt-config dump\` shows what apt actually believes.
Shipping now would give a rover an update policy nobody has read."

# ---------------------------------------------------------------------------
# 9. the master switch, checked rather than assumed
# ---------------------------------------------------------------------------
#
# APT::Periodic::Enable is documented, but it is honoured by a SHELL SCRIPT in
# this image, not by apt itself - so whether it means anything here is a
# property of /usr/lib/apt/apt.systemd.daily, which is readable. The three
# interval keys above are the ones that carry the policy regardless; this is
# the extra lock, and the build log should say whether it is a real one.
DAILY=/usr/lib/apt/apt.systemd.daily
if [ -f "$DAILY" ]; then
    if grep -q 'APT::Periodic::Enable' "$DAILY"; then
        note "${DAILY} honours APT::Periodic::Enable (master switch is real here)"
    else
        note "${DAILY} does not read APT::Periodic::Enable on this image;"
        note "  the three interval keys (all 0) are what stop it. That is enough:"
        note "  with no list update and no download, there is nothing to install."
    fi
else
    note "${DAILY} is not present - nothing invokes apt on a timer at all"
fi

# ---------------------------------------------------------------------------
# 10. unattended-upgrades: is it even here?
# ---------------------------------------------------------------------------
#
# Deliberately NOT installed by this stage. The decision (argued in full in
# ${CONF_D}/70fpms-update-policy) is that this board takes security updates
# when a human asks for them, so the configuration must be correct and
# verifiable, but the package's presence is the base image's business. Say
# which case this image is in, because the operator instructions differ.
if [ -x /usr/bin/unattended-upgrade ] || [ -x /usr/bin/unattended-upgrades ]; then
    note "unattended-upgrades IS installed; the security-only config above governs it"
    note "  operator runs it deliberately:  sudo unattended-upgrade --dry-run -v"
else
    note "unattended-upgrades is NOT installed in this image."
    note "  ${CONF_D}/71fpms-unattended-upgrades is inert but correct, and will"
    note "  govern it the moment anyone installs it. Until then the operator path"
    note "  is the one in docs/UPDATING.md: apt update && apt upgrade, by hand."
fi

# ---------------------------------------------------------------------------
# 11. numpy, re-read after our files landed
# ---------------------------------------------------------------------------
#
# Stage 20 verified its own pin when it wrote it. What is new here is that
# fpms-numpy-ceiling now sits alongside it, and two preferences files
# disagreeing about one package is the kind of thing that reads fine and
# behaves badly. The two are disjoint by construction (1.x vs 2.x/3.x); this
# confirms it against the apt that will actually run on the rover.
NPOL="$(apt-cache policy python3-numpy 2>&1 || true)"
ncand=""
while IFS= read -r line; do
    case "$line" in
        *"Candidate: "*) [ -n "$ncand" ] || ncand="${line##*Candidate: }" ;;
    esac
done <<EOF
$NPOL
EOF
case "$ncand" in
    1.*|1:1.*) note "python3-numpy candidate is ${ncand} (1.x - correct)" ;;
    "")        warn "apt-cache policy python3-numpy reported no candidate at all." ;;
    *)         fail \
"python3-numpy's candidate is '${ncand}', which is not a 1.x.

ROS Humble's C extensions are built against the NumPy 1.x ABI; a 2.x breaks
tf_transformations with 'np.maximum_sctype was removed', reported from a
module nowhere near the cause, and selftest/verify_image.sh fails such an
image outright. Check ${PREF_D}/fpms-numpy (stage 20) and
${PREF_D}/fpms-numpy-ceiling (this stage)." ;;
esac

# The pip half of the same policy is not apt's business and apt cannot see it:
# pip installs to /usr/local/lib/python3.10/dist-packages, which comes BEFORE
# apt's /usr/lib/python3/dist-packages on sys.path. A pip-installed numpy 2.x
# therefore wins for every FPMS service while apt keeps reporting 1.21.5 and
# every check in this stage still passes. Stage 20's
# /etc/pip.conf -> /etc/fpms/pip-constraints.txt is the only thing standing
# there; confirm it survived, because this is the one interaction between the
# two package managers that can defeat everything above.
if [ -f /etc/pip.conf ] && [ -f /etc/fpms/pip-constraints.txt ]; then
    note "pip half of the numpy policy is in place (/etc/pip.conf -> /etc/fpms/pip-constraints.txt)"
else
    warn "the pip half of the numpy policy is missing: /etc/pip.conf or"
    warn "/etc/fpms/pip-constraints.txt is not there. apt's pins CANNOT cover"
    warn "this - a pip numpy in /usr/local shadows apt's for every service and"
    warn "apt will never notice. See scripts/20-python-deps.sh."
fi

# ---------------------------------------------------------------------------
# 12. what an operator will need
# ---------------------------------------------------------------------------
echo "    ---"
echo "    UPDATE POLICY INSTALLED. On the rover:"
echo "      what is frozen:   apt-mark showhold"
echo "      why:              cat ${PREF_D}/fpms-kernel"
echo "      exactly what:     cat ${GEN}"
echo "      is it working:    sudo apt-get -s dist-upgrade --ignore-hold"
echo "      security only:    sudo unattended-upgrade --dry-run -v"
echo "    Nothing on this board updates itself on a timer, and nothing reboots"
echo "    itself. The deliberate-upgrade procedure is in docs/UPDATING.md."
[ "$LISTS_FRESH" = 1 ] || echo "    (proofs above ran on pre-existing apt lists - see the warning)"

echo "--- 70-update-policy OK"
