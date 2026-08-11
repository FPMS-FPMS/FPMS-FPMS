#!/usr/bin/env bash
# Stage 00 - base system: identity, time, discovery, broker, tools.
#
# WHERE THIS RUNS, AND WHY IT SHAPES EVERYTHING BELOW
# ===================================================
# Inside an aarch64 chroot under qemu-user emulation, on Joshua Riek's
# ubuntu-rockchip Ubuntu 22.04 SERVER arm64 image, entered by build.sh's
# in_chroot() with `env -i`, DEBIAN_FRONTEND=noninteractive, LC_ALL=C, and a
# /usr/sbin/policy-rc.d that returns 101 so nothing can start. Four
# consequences shape this file:
#
#   1. systemctl runs OFFLINE. `enable` only writes symlinks, which is exactly
#      what we want -- but it can also fail outright with "System has not been
#      booted with systemd". Every enable in the previous revision of this file
#      ended in `>/dev/null 2>&1 || true`, so a systemd that refused would have
#      produced an image with NOTHING enabled: no ssh, no avahi, no broker, a
#      rover that boots dead, and a build log that said OK. Enables are
#      VERIFIED here, against the symlink that lands on disk.
#   2. The base image is NOT a blank Ubuntu. It already ships an `ubuntu` user,
#      very likely with a password and a "change it on first login" flag, and
#      it may ship its own first-boot setup flow. See "the ubuntu user".
#   3. Anything that forks per item is punishingly slow under emulation, so
#      man-db's trigger is switched off before the first install.
#   4. Nothing here may trust an exit code where it can check an outcome. That
#      is this project's recurring lesson and it is the reason for every
#      verification block below.
set -euo pipefail
echo "--- 00-base-system"

export DEBIAN_FRONTEND=noninteractive

# Belt and braces: systemctl detects the chroot by comparing /proc/1/root with
# /, and build.sh bind-mounts the HOST's /proc, so detection normally works.
# Saying it explicitly removes the one case -- a host without /proc mounted, or
# a qemu that reports /proc/1 oddly -- where systemctl would try to reach a bus
# that is not there and fail every enable below.
export SYSTEMD_OFFLINE=1

fail() { echo ""; echo "FATAL: 00-base-system: $*" >&2; echo ""; exit 1; }
note() { echo "    $*"; }
warn() { echo "    WARNING: $*" >&2; }

# build.sh passes these through `env -i`. Under `set -u` a missing one would
# die with "unbound variable" and a line number, which tells a human nothing.
for v in FPMS_USER FPMS_HOME FPMS_HOSTNAME; do
    eval "val=\${$v:-}"
    [ -n "$val" ] || fail \
"$v is not set.
This stage is invoked by build.sh, which passes FPMS_USER, FPMS_HOME and
FPMS_HOSTNAME into the chroot explicitly. Running it by hand (build.sh --shell)
needs them exported first."
done

# ---------------------------------------------------------------------------
# packages
# ---------------------------------------------------------------------------

# man-db reindexes in a dpkg trigger that forks once per page. Native that is a
# few seconds; under qemu-user it is minutes of pure waste for documentation
# nobody reads on a headless rover.
echo 'man-db man-db/auto-update boolean false' | debconf-set-selections 2>/dev/null || true

# apt is the first thing in the build that needs the network, so it is also the
# first thing to expose a broken chroot resolver. build.sh copies the host's
# /etc/resolv.conf in; on an image where that path is a symlink into /run (this
# base uses systemd-resolved) the copy lands in the chroot's fresh tmpfs /run
# and there is no DNS at all. Say so here, in one line, instead of letting apt
# print forty lines of "Temporary failure resolving".
if ! getent hosts ports.ubuntu.com >/dev/null 2>&1 \
   && ! getent hosts archive.ubuntu.com >/dev/null 2>&1; then
    warn "the chroot cannot resolve ports.ubuntu.com."
    warn "Check /etc/resolv.conf INSIDE the chroot - if it is a dangling symlink"
    warn "into /run, build.sh's copy went into the tmpfs and apt has no DNS."
fi

# --- repair an interrupted dpkg --------------------------------------------
#
# THIS IS THE RESUME PATH'S MOST COMMON FAILURE, and it is self-inflicted.
#
# Every time a build is killed mid-`apt-get install` -- a session ending, a
# laptop hibernating, an operator pressing ctrl-C -- dpkg is left with a
# half-configured package and writes /var/lib/dpkg/updates. The NEXT apt
# command in that image then refuses outright:
#
#     E: dpkg was interrupted, you must manually run 'dpkg --configure -a'
#        to correct the problem.
#
# apt exits non-zero, the retry loop below burns all three attempts against a
# condition no retry can fix, and the stage dies five minutes in reporting
# "apt-get update failed three times" -- which points at DNS and the network,
# neither of which is wrong. The real fault is a state left inside the image by
# the previous run, and `--from 00` is precisely the command an operator uses
# to recover from that, so it must repair it rather than trip over it.
#
# Detected by asking dpkg to audit itself rather than by grepping apt's error,
# so it also catches a half-unpacked package that has not yet produced one.
if [ -n "$(ls -A /var/lib/dpkg/updates 2>/dev/null || true)" ] \
   || [ -n "$(dpkg --audit 2>/dev/null || true)" ]; then
    warn "dpkg was interrupted by a previous run - repairing before apt"
    if dpkg --configure -a 2>&1 | sed 's/^/      /'; then
        note "dpkg --configure -a completed"
    else
        # Not fatal on its own: a package that cannot configure here may well
        # be one apt is about to replace. Say so and let apt render judgement.
        warn "dpkg --configure -a did not fully succeed; continuing so that apt"
        warn "can report which package is actually wedged"
    fi
fi

# A retry, because one lost packet during a 20-minute emulated build should not
# cost the whole build. Still fatal after three: an image built on a partial
# package list is worse than no image.
apt_updated=0
for attempt in 1 2 3; do
    if apt-get update -qq -o Acquire::Retries=3; then apt_updated=1; break; fi
    warn "apt-get update failed (attempt ${attempt}/3), retrying in 5s"
    sleep 5
done
[ "$apt_updated" = 1 ] || fail \
"apt-get update failed three times.
Either the chroot has no DNS (see the resolver warning above), or one of the
suites in /etc/apt/sources.list.d is unreachable. Fix it and re-run with
  sudo ./build.sh --stage 00"

# Every package below exists in jammy/arm64. Two are here for reasons that are
# not obvious from the name:
#
#   gdisk           - growpart resizes a GPT disk by shelling out to `sgdisk`,
#                     and these images ARE GPT (build.sh runs sgdisk on them).
#                     cloud-guest-utils only RECOMMENDS gdisk, and this install
#                     is --no-install-recommends, so without naming it here
#                     growpart fails at first boot with "sgdisk not found".
#                     fpms-firstboot swallows that (`growpart ... || true`,
#                     because growpart also exits 1 when already grown) and
#                     resize2fs then finds nothing to do and returns 0 - so the
#                     rootfs is NEVER GROWN and firstboot reports success. That
#                     is precisely the silent-failure shape this project keeps
#                     being bitten by.
#   dosfstools      - fsck.vfat for /boot/firmware. If the FAT partition fails
#                     its boot-time fsck and does not mount, fpms-wifi-provision
#                     finds no fpms-wifi.conf and quietly falls back to the AP,
#                     which looks exactly like a typo in the operator's file.
apt-get install -y -qq --no-install-recommends -o Acquire::Retries=3 \
    ca-certificates curl wget gnupg lsb-release locales tzdata \
    sudo openssh-server \
    avahi-daemon avahi-utils libnss-mdns \
    systemd-timesyncd fake-hwclock \
    mosquitto mosquitto-clients \
    iw wireless-tools network-manager \
    usbutils v4l-utils udev \
    python3 python3-pip python3-venv \
    cloud-guest-utils gdisk e2fsprogs parted dosfstools \
    jq git vim-tiny less htop

# Verify what the rest of the image actually calls, rather than trusting that
# apt installed what we think it did. Each of these is invoked by a script that
# fences its own failures (fpms-firstboot, fpms-wifi-provision), so a missing
# binary would never surface as an error - only as a rover that did not grow,
# did not get a broker password, or did not join a network.
for cmd in growpart sgdisk resize2fs ssh-keygen mosquitto_passwd nmcli \
           avahi-daemon fake-hwclock jq findmnt; do
    command -v "$cmd" >/dev/null 2>&1 \
        || fail "'$cmd' is not on PATH after the package install. The image
would boot without it and the script that needs it fences its own failure, so
this would never be reported at runtime."
done
note "package set installed and the binaries firstboot needs are present"

# ---------------------------------------------------------------------------
# locale and time
# ---------------------------------------------------------------------------

# `locale-gen en_US.UTF-8` alone is not enough on every image: the argument
# form generates the locale but does NOT record it in /etc/locale.gen, so the
# next `dpkg-reconfigure locales` or a locales upgrade silently drops it.
# Uncomment the line first, then generate everything the file asks for.
if ! grep -qE '^[[:space:]]*en_US\.UTF-8[[:space:]]+UTF-8' /etc/locale.gen 2>/dev/null; then
    if grep -qE '^[[:space:]]*#[[:space:]]*en_US\.UTF-8[[:space:]]+UTF-8' /etc/locale.gen 2>/dev/null; then
        sed -i 's/^[[:space:]]*#[[:space:]]*\(en_US\.UTF-8[[:space:]]\+UTF-8\)/\1/' /etc/locale.gen
    else
        echo 'en_US.UTF-8 UTF-8' >> /etc/locale.gen
    fi
fi
locale-gen >/dev/null
update-locale LANG=en_US.UTF-8

# Verify the locale actually exists. locale-gen under LC_ALL=C in an emulated
# chroot is exactly the kind of step that prints nothing and does nothing.
#
# Held in a variable rather than piped into `grep -q`: with `pipefail`, a grep
# that exits on its first match can SIGPIPE the writer and turn a successful
# check into a failed pipeline.
locales_available="$(locale -a 2>/dev/null | tr '[:upper:]' '[:lower:]' || true)"
case "$locales_available" in
    *en_us.utf8*|*en_us.utf-8*) ;;
    *) fail "en_US.UTF-8 was not generated (locale -a does not list it).
Python and ROS will fall back to POSIX and any non-ASCII in a log line becomes
a UnicodeEncodeError traceback." ;;
esac
note "locale en_US.UTF-8 generated"

# UTC, fixed. Nothing in the FPMS code does timezone conversion - timestamps
# are unix seconds everywhere and the UI localises. A local timezone here just
# makes journal correlation across the rover and the laptop harder.
#
# Both halves say Etc/UTC: timedatectl reads the symlink, debconf reads
# /etc/timezone, and having them disagree is how a tzdata upgrade "helpfully"
# re-points the symlink at something else.
ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime
echo "Etc/UTC" > /etc/timezone

# The board has NO RTC backup battery: it boots to the filesystem epoch, and
# without this every timestamp is 1970 until the network comes up. TLS to the
# cloud ingest cannot validate a certificate with a 1970 clock.
#
# fake-hwclock at least makes time monotonic across reboots when there is no
# NTP reachable, which on a competition floor is most of the time.
#
# (enable_unit is defined below, next to the rest of the systemd handling.)

# ---------------------------------------------------------------------------
# systemd enables - verified, not hoped for
# ---------------------------------------------------------------------------
#
# In a chroot `systemctl enable` writes symlinks and never contacts PID 1. It
# still exits non-zero if it cannot read the unit, if the unit is masked on the
# base image, or if this systemd decides it is not offline after all. So: try
# to unmask, enable, then LOOK FOR THE SYMLINK. Failure here is fatal, because
# an image where mosquitto or ssh is not enabled is not recoverable in the
# field - it is a card-reader job.
enable_unit() {
    local unit="$1" state link

    state="$(systemctl is-enabled "$unit" 2>/dev/null || true)"
    if [ "$state" = "masked" ] || [ "$state" = "masked-runtime" ]; then
        warn "$unit is masked on the base image - unmasking"
        systemctl unmask "$unit" >/dev/null 2>&1 || true
    fi

    systemctl enable "$unit" >/dev/null 2>&1 || true

    # The outcome: a .wants (or .requires) symlink under /etc/systemd/system,
    # which is the only thing that makes the unit start at boot.
    link="$(find /etc/systemd/system -maxdepth 2 -name "$unit" -type l -print -quit 2>/dev/null || true)"
    if [ -n "$link" ]; then
        note "enabled  ${unit}  ->  ${link#/etc/systemd/system/}"
        return 0
    fi

    state="$(systemctl is-enabled "$unit" 2>/dev/null || echo unknown)"
    case "$state" in
        enabled|enabled-runtime|alias|static|indirect)
            # No symlink, but systemd says it will run: `static` and `indirect`
            # mean another unit pulls it in. Print the state so a human can
            # judge it rather than discovering it on the rover.
            note "enabled  ${unit}  (is-enabled: ${state}, no .wants link - pulled in by another unit)"
            return 0 ;;
    esac

    fail "could not enable ${unit} (is-enabled says '${state}', and no symlink
appeared under /etc/systemd/system). This is the failure that produces an image
where nothing starts at boot. Nothing downstream can detect it, so the build
stops here."
}

enable_unit systemd-timesyncd.service
enable_unit fake-hwclock.service

# Stamp the build date into fake-hwclock's file so a freshly flashed board
# starts from the build date instead of whatever was there when the package was
# unpacked. fake-hwclock only ever moves the clock FORWARD on load, so this can
# never push a board's clock backwards.
fake-hwclock save >/dev/null 2>&1 || warn "fake-hwclock save failed (not fatal)"

# DELIBERATELY NOT ENABLED: systemd-time-wait-sync. Making any FPMS unit wait
# for a synchronised clock would delay STOP authority at boot, which
# contradicts the one ordering guarantee the stack actually makes.

# ---------------------------------------------------------------------------
# the base image's own first-boot flow
# ---------------------------------------------------------------------------
#
# This is the single biggest assumption the previous revision of this file
# made: that it was starting from a blank Ubuntu. It is not. Riek's images ship
# a ready-made `ubuntu` user, and depending on the variant a first-login
# password-change flow, an oem-config-style setup wizard, or cloud-init.
#
# Any of those is a SECOND OWNER of exactly what fpms-firstboot owns - the
# user, the hostname, /etc/hosts, the SSH host keys, the network config - and a
# setup wizard on a headless rover PROMPTS ON THE SERIAL CONSOLE AND WAITS,
# which on a board with no monitor is indistinguishable from a hang.
#
# The base image cannot be inspected from a text editor, so this detects by
# name, neutralises what is there, and PRINTS WHAT IT FOUND either way. The
# build log is the only place anyone will ever see this. IF IT NAMES SOMETHING
# YOU DID NOT EXPECT, look at that unit before shipping the image; every mask
# here is one `systemctl unmask` away from being undone.
#
# systemd-firstboot.service is caught by the glob below and that is DELIBERATE.
# Stage 90 empties /etc/machine-id (it must - a shared machine-id across a
# fleet breaks DHCP leases and journal identity), which is exactly what puts
# systemd into ConditionFirstBoot=yes. Its first-boot wizard then runs with
# StandardInput=tty and --prompt-locale --prompt-timezone --prompt-root-password
# on the console. On a headless rover with a serial console that is a boot that
# never finishes and no way to see why. Everything it would ask about - locale,
# timezone, hostname - this stage has already decided, so masking it loses
# nothing and removes a whole class of unattended-boot hang.
found_setup=0
mask_unit() {
    local unit="$1"
    systemctl disable "$unit" >/dev/null 2>&1 || true
    # `systemctl mask` refuses on some systemd versions when the unit file is
    # gone; the symlink to /dev/null IS the mask, so write it directly and then
    # check it. Masking by name also survives a package coming back later.
    systemctl mask "$unit" >/dev/null 2>&1 || true
    [ -L "/etc/systemd/system/$unit" ] || ln -sf /dev/null "/etc/systemd/system/$unit"
    [ -L "/etc/systemd/system/$unit" ] || fail "could not mask $unit"
}

# /lib and /usr/lib are the same directory on a usrmerged 22.04, so the same
# unit turns up twice; keep a seen-list so the log names each one once.
seen=" "
for d in /etc/systemd/system /lib/systemd/system /usr/lib/systemd/system; do
    [ -d "$d" ] || continue
    for f in "$d"/*oem-config* "$d"/*oem_config* "$d"/ubiquity*.service \
             "$d"/*first-boot*.service "$d"/*firstboot*.service; do
        [ -e "$f" ] || continue
        u="$(basename "$f")"
        case "$u" in
            fpms-*) continue ;;          # ours; stage 60 enables it
            *.service|*.target) ;;       # only units, not .d directories
            *) continue ;;
        esac
        case "$seen" in *" $u "*) continue ;; esac
        seen="${seen}${u} "
        found_setup=1
        mask_unit "$u"
        note "NEUTRALISED base-image setup unit: $u  (masked -> /dev/null)"
    done
done

# cloud-init, if present, would re-do the user, the hostname, /etc/hosts and
# the SSH host keys at first boot, on its own schedule, fighting
# fpms-firstboot. Disable it the documented, reversible way rather than by
# purging: existing netplan/NetworkManager config on disk is untouched, and an
# operator who needs it back deletes one file.
if [ -d /etc/cloud ] || command -v cloud-init >/dev/null 2>&1; then
    found_setup=1
    mkdir -p /etc/cloud
    : > /etc/cloud/cloud-init.disabled
    note "NEUTRALISED cloud-init (touch /etc/cloud/cloud-init.disabled)"
    note "  re-enable with: sudo rm /etc/cloud/cloud-init.disabled"
fi

[ "$found_setup" = 1 ] || note "no base-image first-boot setup flow found by name"

# ---------------------------------------------------------------------------
# identity
# ---------------------------------------------------------------------------
echo "${FPMS_HOSTNAME}" > /etc/hostname
if ! grep -qE "^127\.0\.1\.1[[:space:]]+${FPMS_HOSTNAME}" /etc/hosts; then
    sed -i '/^127\.0\.1\.1[[:space:]]/d' /etc/hosts
    echo "127.0.1.1 ${FPMS_HOSTNAME}" >> /etc/hosts
fi

# avahi is what makes fpms-pi.local resolve, and it is assumed by the console,
# rosbridge, Foxglove and every deploy script in this project. NOTHING in the
# repository ever installed it - that gap is why "the rover's IP has changed
# more than seven times and every note that wrote one down was wrong the next
# day" was a recurring problem rather than a solved one.
enable_unit avahi-daemon.service

# Do NOT disable IPv6. The Pi's mDNS has historically answered over IPv6
# link-local, and at least one resolver on the operator laptop depended on it.
#
# Verified rather than `|| true`: if this sed silently does nothing, .local
# resolution FROM the rover breaks, which is how the mosquitto bridge and every
# deploy script find the laptop.
[ -f /etc/nsswitch.conf ] || fail "/etc/nsswitch.conf does not exist"
sed -i 's/^hosts:.*/hosts: files mdns4_minimal [NOTFOUND=return] dns mdns4/' \
    /etc/nsswitch.conf
grep -qE '^hosts:.*mdns4_minimal' /etc/nsswitch.conf \
    || fail "/etc/nsswitch.conf has no mdns4_minimal in its hosts line.
.local names will not resolve FROM the rover, which breaks the mosquitto bridge
and every deploy script that dials the laptop by name."
note "nsswitch hosts: $(grep -E '^hosts:' /etc/nsswitch.conf)"

# ---------------------------------------------------------------------------
# the ubuntu user
# ---------------------------------------------------------------------------
#
# The name and home are NOT configurable in practice. Every unit hardcodes
# User=ubuntu and /home/ubuntu, and three Python files hardcode absolute paths
# under it with no override at all (the teleop origin anchor, read by missions
# and written by teleop, and the YOLO directory default).
#
# THE USER ALMOST CERTAINLY ALREADY EXISTS on this base image, with its own
# home, its own primary group and - the part that matters - its own password
# state. Everything below is written for "already there" first and "create it"
# second.
if id -u "${FPMS_USER}" >/dev/null 2>&1; then
    note "user '${FPMS_USER}' already exists on the base image (uid $(id -u "${FPMS_USER}"))"
else
    useradd -m -s /bin/bash "${FPMS_USER}"
    note "created user '${FPMS_USER}'"
fi

# The home path is load-bearing and unoverridable downstream, so a base image
# that put it somewhere else must stop the build rather than produce an image
# where three Python files write to a directory nobody owns.
actual_home="$(getent passwd "${FPMS_USER}" | cut -d: -f6)"
[ "$actual_home" = "${FPMS_HOME}" ] || fail \
"user '${FPMS_USER}' has home '${actual_home}', not '${FPMS_HOME}'.
Every systemd unit and three Python files hardcode ${FPMS_HOME} with no
override. Fix the base image or the config; do not ship this."

# The previous revision assumed `useradd -m` had run and therefore that the
# home directory existed. On an image where the user was already present but
# the home was not (or was mounted later), `install -d` would have created it
# 0700 as a side effect of creating .ssh. Be explicit.
if [ ! -d "${FPMS_HOME}" ]; then
    warn "${FPMS_HOME} does not exist - creating it"
    install -d -m 0755 "${FPMS_HOME}"
fi
FPMS_GROUP="$(id -gn "${FPMS_USER}")"
chown "${FPMS_USER}:${FPMS_GROUP}" "${FPMS_HOME}"

# Shell: an existing user could have /bin/sh or /usr/sbin/nologin. Every doc in
# this project assumes an interactive bash over ssh.
case "$(getent passwd "${FPMS_USER}" | cut -d: -f7)" in
    /bin/bash) ;;
    *) usermod -s /bin/bash "${FPMS_USER}"; note "shell set to /bin/bash" ;;
esac

# usermod -aG with a list fails ENTIRELY if any one group is missing, which
# under `set -e` kills the stage on a base image that happens not to ship
# plugdev. Add them one at a time and verify membership afterwards - a user
# silently not in dialout is a serial port that cannot be opened three stages
# later, reported as "the drive board is dead".
for g in sudo dialout video plugdev audio; do
    if ! getent group "$g" >/dev/null 2>&1; then
        warn "group '$g' does not exist on this base image - skipped"
        continue
    fi
    usermod -aG "$g" "${FPMS_USER}"
done
user_groups=" $(id -nG "${FPMS_USER}") "
for g in sudo dialout video plugdev audio; do
    getent group "$g" >/dev/null 2>&1 || continue
    case "$user_groups" in
        *" $g "*) ;;
        *) fail "${FPMS_USER} is not in group '$g' after usermod -aG. On this
rover that is not cosmetic: without dialout the drive board's tty cannot be
opened, and the symptom three stages later is 'the drive board is dead'." ;;
    esac
done
note "groups:${user_groups}"

# Passwordless sudo: the deploy tooling (deploy_rover.py) requires it.
cat > /etc/sudoers.d/90-fpms-user <<EOF
${FPMS_USER} ALL=(ALL) NOPASSWD:ALL
EOF
chmod 0440 /etc/sudoers.d/90-fpms-user
visudo -cf /etc/sudoers.d/90-fpms-user >/dev/null

# --- the password, and the first-login trap ---------------------------------
#
# No password is ever usable. Login is by key only (below). A baked default
# password on a fleet image is a credential leak the moment one image is shared
# - and this base image ships one: its documented behaviour is `ubuntu:ubuntu`,
# changed at first login.
#
# `passwd -l` was not enough for two reasons:
#
#   1. It only prefixes the existing hash with '!'. The base image's known,
#      published hash is still sitting there to be un-locked. Setting the field
#      to '*' leaves nothing to restore.
#   2. It does NOT clear the "must change password at next login" flag that the
#      base image sets (shadow field 3 == 0). That flag is not cosmetic on a
#      key-only image: PAM's account stage returns NEW_AUTHTOK_REQD, sshd tries
#      to run a password-change conversation, and with
#      KbdInteractiveAuthentication no it cannot - so the KEY LOGIN IS REFUSED
#      as well. The rover would be unreachable by every route at once, and the
#      only symptom is a generic authentication failure in the client.
usermod -p '*' "${FPMS_USER}"
chage -d "$(date -u +%Y-%m-%d)" -m 0 -M -1 -I -1 -E -1 "${FPMS_USER}"

shadow_line="$(getent shadow "${FPMS_USER}" || true)"
[ -n "$shadow_line" ] || fail "no /etc/shadow entry for ${FPMS_USER}"
pw_field="$(printf '%s' "$shadow_line" | cut -d: -f2)"
lastchg="$(printf '%s' "$shadow_line" | cut -d: -f3)"
case "$pw_field" in
    '*'|'!'*)  ;;                         # no usable password. Correct.
    '')        fail "${FPMS_USER} has an EMPTY password field - that means login
with no password at all. Refusing to build this image." ;;
    *)         fail "${FPMS_USER} still has a usable password hash after
usermod -p '*'. The base image's default credential would ship with the image." ;;
esac
[ -n "$lastchg" ] && [ "$lastchg" != "0" ] \
    || fail "${FPMS_USER} still has the 'must change password at next login'
flag set (shadow lastchg='${lastchg}'). On a key-only image that refuses key
logins too - see the comment above."
note "password disabled ('${pw_field}') and expiry cleared (lastchg=${lastchg})"

# ---------------------------------------------------------------------------
# ssh
# ---------------------------------------------------------------------------
#
# HANDOFF.md records that the previously-used shared password should be treated
# as exposed and rotated, and that the same secret was reused for the dashboard
# login. Key-only closes that.
#
# NOTE FOR WHOEVER RUNS deploy_rover.py: it authenticates with a PASSWORD today
# (paramiko connect(..., password=...)). discovery.py already has a key_path
# parameter with no caller. Those tools need to be pointed at a key, or they
# break against this image on day one. That is a deliberate, visible tradeoff.
mkdir -p /etc/ssh/sshd_config.d
cat > /etc/ssh/sshd_config.d/10-fpms.conf <<'EOF'
# FPMS-OS ssh policy.
PasswordAuthentication no
PermitRootLogin no
KbdInteractiveAuthentication no
PubkeyAuthentication yes
EOF

# Host keys are NOT generated here. fpms-firstboot generates them, so every
# flashed board gets its own -- a shared host key across a fleet is a real
# problem and it also makes every laptop's known_hosts conflict.
#
# ORDERING, verified against overlay/etc/systemd/system/fpms-firstboot.service:
# that unit is DefaultDependencies=no, WantedBy=sysinit.target and
# Before=network-pre.target; ssh.service is After=network.target, which is
# ordered after network-pre.target. So the keys exist before sshd is asked to
# start, and firstboot restarts ssh afterwards anyway.
#
# BELT AND BRACES, because "silence is never success": if firstboot's
# ssh-keygen ever fails (it warns and continues by design - it must never block
# the boot), sshd's own `sshd -t` pre-check fails on a keyless host and the
# unit stays down until the next reboot, on the one interface anybody has to
# reach the rover. The drop-in below regenerates missing keys immediately
# before sshd starts. `ssh-keygen -A` only creates what is absent, so when
# firstboot has done its job this is a no-op.
#
# The empty `ExecStartPre=` is required: drop-ins APPEND to the list, and the
# vendor unit's `sshd -t` would otherwise run first and fail before we got
# there. Resetting and re-adding both keeps the vendor's syntax check.
rm -f /etc/ssh/ssh_host_*
if ls /etc/ssh/ssh_host_* >/dev/null 2>&1; then
    fail "ssh host keys survived the rm - the image would ship a shared host key
across the whole fleet, and every laptop's known_hosts would conflict."
fi

install -d -m 0755 /etc/systemd/system/ssh.service.d
cat > /etc/systemd/system/ssh.service.d/10-fpms-hostkeys.conf <<'EOF'
# Generate any missing SSH host keys before sshd's own config check runs.
# The image deliberately ships none; fpms-firstboot normally makes them, but it
# is fenced against blocking the boot, so this is the backstop that keeps the
# only remote way into the rover from depending on that.
[Service]
ExecStartPre=
ExecStartPre=-/usr/bin/ssh-keygen -A
ExecStartPre=/usr/sbin/sshd -t
EOF

install -d -m 0700 -o "${FPMS_USER}" -g "${FPMS_GROUP}" "${FPMS_HOME}/.ssh"
touch "${FPMS_HOME}/.ssh/authorized_keys"
chmod 0600 "${FPMS_HOME}/.ssh/authorized_keys"
chown -R "${FPMS_USER}:${FPMS_GROUP}" "${FPMS_HOME}/.ssh"

# Optional build-time key seeding. Without a key in this file the flashed image
# has no way in over SSH at all (password auth off, no usable password), so the
# builder gets one, loudly, at the end of the stage.
seed_key="${FPMS_SSH_PUBKEY:-}"
for f in /opt/fpms-os/authorized_keys /opt/fpms-os/overlay/boot/authorized_keys; do
    if [ -z "$seed_key" ] && [ -r "$f" ]; then
        seed_key="$(cat "$f")"
    fi
done
if [ -n "$seed_key" ]; then
    # Line at a time, and skipping any that is already there: re-running the
    # stage must not append the same key twice.
    while IFS= read -r k; do
        [ -n "$k" ] || continue
        grep -qxF "$k" "${FPMS_HOME}/.ssh/authorized_keys" 2>/dev/null \
            || printf '%s\n' "$k" >> "${FPMS_HOME}/.ssh/authorized_keys"
    done <<< "$seed_key"
    note "seeded $(wc -l < "${FPMS_HOME}/.ssh/authorized_keys") authorized key(s)"
fi

enable_unit ssh.service

# ---------------------------------------------------------------------------
# mosquitto
# ---------------------------------------------------------------------------
# Config comes from the overlay (stage 50). The password file is generated at
# first boot (fpms-firstboot is Before=mosquitto.service, so the real entry is
# written before the broker ever reads it). Create the file now so mosquitto
# does not refuse to start on a missing password_file before firstboot has run.
getent passwd mosquitto >/dev/null 2>&1 \
    || fail "the mosquitto package did not create its 'mosquitto' user."
install -d -m 0755 /etc/mosquitto
touch /etc/mosquitto/fpms.passwd
chmod 0600 /etc/mosquitto/fpms.passwd
chown mosquitto:mosquitto /etc/mosquitto/fpms.passwd

# Verified, not `|| true`. The broker drops privileges to the mosquitto user;
# a 0600 file owned by root is one it cannot read, and the failure mode is the
# one this project fears most - every service retries forever against a broker
# that rejects it, nothing crashes, and the dashboard is silently empty.
[ "$(stat -c %U /etc/mosquitto/fpms.passwd)" = "mosquitto" ] \
    || fail "/etc/mosquitto/fpms.passwd is not owned by the mosquitto user."
enable_unit mosquitto.service

# ---------------------------------------------------------------------------
# directories
# ---------------------------------------------------------------------------
install -d -m 0755 /etc/fpms
install -d -m 0755 -o "${FPMS_USER}" -g "${FPMS_GROUP}" /var/lib/fpms
install -d -m 0755 -o "${FPMS_USER}" -g "${FPMS_GROUP}" "${FPMS_HOME}/yolo"

# ---------------------------------------------------------------------------
# ModemManager
# ---------------------------------------------------------------------------
# It probes unknown serial devices by toggling handshake lines. On this board
# RTS drives the ESP32-S3's EN pin, so a probe is a HARDWARE RESET of the drive
# board, costing a 90-225 second re-link. The udev rules mark both CP2102s
# ID_MM_DEVICE_IGNORE, but the surest fix is not to have it installed at all -
# nothing on this rover is a modem.
#
# A blind `apt-get purge ... || true` was two risks in one line: it hid whether
# the package was ever there, and apt would have removed anything that depended
# on it without comment. network-manager only RECOMMENDS modemmanager, so this
# is expected to be clean - check rather than assume.
mm_status="$(dpkg-query -W -f='${Status}' modemmanager 2>/dev/null || true)"
case "$mm_status" in *"ok installed"*) mm_installed=1 ;; *) mm_installed=0 ;; esac
if [ "$mm_installed" = 1 ]; then
    extra="$(apt-get -s purge -y modemmanager 2>/dev/null \
             | awk '/^(Purg|Remv) /{print $2}' \
             | grep -vE '^(modemmanager|libmm-glib0|libqmi-glib5|libmbim-glib4|libqrtr-glib0)$' \
             || true)"
    if [ -n "$extra" ]; then
        warn "NOT purging modemmanager: apt would also remove:"
        warn "$(printf '%s' "$extra" | tr '\n' ' ')"
        warn "Left installed and masked instead."
    else
        apt-get purge -y -qq modemmanager >/dev/null
        note "purged modemmanager"
    fi
else
    note "modemmanager was not installed on the base image"
fi

# Mask it by NAME regardless. Masking is the guarantee the purge is not: it
# survives a later apt run pulling ModemManager back in as a Recommends, which
# is exactly how it would return, and it costs nothing if the package is gone.
mask_unit ModemManager.service
note "masked   ModemManager.service"

# ---------------------------------------------------------------------------
# what the builder must know before flashing
# ---------------------------------------------------------------------------
if [ ! -s "${FPMS_HOME}/.ssh/authorized_keys" ]; then
    echo ""
    echo "  ####################################################################"
    echo "  #  ${FPMS_HOME}/.ssh/authorized_keys IS EMPTY."
    echo "  #"
    echo "  #  This image has PasswordAuthentication no and no usable password,"
    echo "  #  so as built there is NO WAY IN over SSH and no console login."
    echo "  #  That is the SPEC's key-only policy working as intended, but it"
    echo "  #  needs a key from somewhere:"
    echo "  #"
    echo "  #    - export FPMS_SSH_PUBKEY='ssh-ed25519 AAAA...' and re-run"
    echo "  #      (build.sh must pass it through in_chroot's env -i list), or"
    echo "  #    - put the key in a file at fpms-os/authorized_keys, or"
    echo "  #    - have first boot read one off the FAT boot partition"
    echo "  #      (fpms-firstboot does NOT do this today - it reads wifi,"
    echo "  #       hostname, broker host and broker password only)."
    echo "  ####################################################################"
    echo ""
fi

echo "--- 00-base-system OK"
