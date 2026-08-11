# tools/ — starting a 15-hour build and walking away

A full FPMS-OS image build is **15–20 hours**. Stage 10 alone was measured at
**14 h 20 m** under qemu-user emulation, and nothing caches the micro-ROS
colcon build, so every death costs those hours again from scratch.

Three builds have died. Two of them were launched with `nohup setsid` from
inside WSL — which *should* have detached them — and died anyway when the
controlling process went away.

These two scripts exist so that does not happen a fourth time.

| File | What it does |
|---|---|
| `run-build-detached.ps1` | Pre-flights, refuses to double-start, launches the build detached, and prints the proof that it is detached. Returns in ~2 s. |
| `build-status.ps1` | Read-only. Running or not, which stage, how long each finished stage took, log tail, image, free space. Cheap enough to run every minute. |

Neither script modifies the build. `build.sh`, `scripts/`, and `overlay/` are
untouched by both.

---

## Start a build and walk away

```powershell
cd C:\Users\ruchi\fpms-os-wt\cloud\dashboard\rover\fpms-os

# see what it would do, touch nothing, never invoke wsl.exe
.\tools\run-build-detached.ps1 -DryRun

# full build
.\tools\run-build-detached.ps1

# resume after a stage failure - this is the difference between a two-minute
# retry and another overnight run
.\tools\run-build-detached.ps1 -From 20
```

| Parameter | Default | Notes |
|---|---|---|
| `-Distro` | `Ubuntu-22.04` | Must be a WSL2 distro registered to *this* Windows user. |
| `-RepoPath` | auto-detected | Linux path of the checkout **inside** the distro. Must be on the distro's own ext4, never `/mnt/c`. |
| `-From NN` | — | `build.sh --from NN`: run stage NN and everything after it, against the existing image. |
| `-Fresh` | off | `build.sh --fresh`: start again from the vendor base. **Discards the image**, so it is refused when one exists unless you add `-Force`. |
| `-NoDownload` | off | `build.sh --no-download`. |
| `-MinFreeGB` | `20` | Refuse to start below this on the Windows volume backing the VHDX. |
| `-DryRun` | off | Windows-side pre-flight, print the runner, exit. Does not invoke `wsl.exe`. |

**Before you walk away**, do these two things — the script warns about the
first and cannot check the second:

```powershell
powercfg /change standby-timeout-ac 0     # sleep suspends (and usually kills) the WSL2 VM
powercfg /change hibernate-timeout-ac 0
```

and **stay signed in**. Lock the screen (`Win`+`L`); do not sign out. A WSL2
instance belongs to a Windows sign-in session, and signing out takes it with
you no matter what the build is parented to.

---

## Check on it

```powershell
.\tools\build-status.ps1            # everything
.\tools\build-status.ps1 -Tail 100  # more log
```

Safe to run at any time, including while the build is at its most delicate.
It never writes, never mounts, never signals, and it asks Windows whether the
distro is running *before* it invokes into it — so running it against a
stopped distro does not boot one.

Exit codes, for scripting a poll: `0` running, `1` not running, `2` no such
distro, `3` distro stopped, `4` the probe timed out (a wedged VM).

To watch the log live instead:

```powershell
wsl -d Ubuntu-22.04 -u root -e tail -f /root/fpms-build/rover/fpms-os/.build/detached-*.log
```

### "It has printed nothing for hours"

That is what a healthy stage 10 looks like. `10-ros-humble.sh` compiles the
Micro XRCE-DDS Agent, Fast-CDR, and Fast-DDS from source under emulation with
the output redirected to `/dev/null`. **14 hours of silence is normal.**

`build-status.ps1` reports three liveness signals that do not depend on the
log: heartbeat age, `qemu-aarch64` processes and their CPU time, and the mtime
of `uros_ws/build` inside the mounted image. If those are moving, it is
working. **Do not kill a quiet build.**

---

## Stopping a build cleanly

`build.sh` traps `TERM` and runs `cleanup()`: it unmounts the bind mounts in
reverse order and detaches the loop device. That trap is the only thing
standing between you and a wedged host, so **give it the chance to run**.
Never `kill -9` the build, and never `wsl --terminate` your way out of it.

```powershell
$D = 'Ubuntu-22.04'

# 1. ask it to stop. The runner forwards TERM to build.sh, whose trap unmounts
#    and detaches before exiting.
wsl -d $D -u root -e bash -c "kill -TERM `$(awk -F= '/^pid=/{print `$2}' /var/tmp/fpms-os-build/build.lock)"

# 2. wait for it to actually go. Give it a minute; cleanup unmounts an ext4
#    with hours of dirty pages behind it.
wsl -d $D -u root -e bash -c 'for i in $(seq 1 60); do pgrep -f "[b]uild\.sh" >/dev/null || break; sleep 2; done; pgrep -af "[b]uild\.sh" || echo "build.sh is gone"'

# 3. VERIFY the mounts and the loop device are released. Do not skip this.
wsl -d $D -u root -e bash -c 'mount | grep "\.build/mnt" ; losetup -a | grep fpms-os ; echo "--- if both lists above are empty, it is clean ---"'
```

If step 3 is not empty, release it by hand — **in this order**, innermost
first:

```powershell
wsl -d $D -u root -e bash -c '
  R=/root/fpms-build/rover/fpms-os          # your repo path
  for m in dev/pts dev proc sys run boot/firmware ""; do
      umount "$R/.build/mnt/$m" 2>/dev/null
  done
  for L in $(losetup -j "$R"/fpms-os-*.img 2>/dev/null | cut -d: -f1); do
      partx -d "$L" 2>/dev/null; losetup -d "$L"
  done
  mount | grep "\.build/mnt"; losetup -a | grep fpms-os; echo clean-check-done'
```

### Cleaning up after a dead build

> **Release every mount before you delete anything.**
> `.build/mnt/dev` is a **bind mount of the distro's own `/dev`**, and
> `.build/mnt/proc`, `/sys`, `/run` are live kernel filesystems. An `rm -rf`
> on `.build` that crosses one of those does not stop at the image — it walks
> into the host. Run the release block above, confirm both lists are empty,
> and only then delete.

The image itself is **not** scratch space. `--from N` resumes against it and
is worth 14 hours. Delete it only when you have decided to.

---

## What kills a build, and what it survives

| Event | Build survives? | Why |
|---|---|---|
| Closing the terminal / tab you launched from | **Yes** | The launcher's `wsl.exe` already exited. Nothing on the Windows side holds the build. |
| The agent, script, or PowerShell session that ran the launcher exiting | **Yes** | Same. There is no child process left in that tree to kill. |
| `Ctrl-C` in that window | **Yes** | There is nothing left in that window to interrupt. |
| Killing every `wsl.exe` (`taskkill /IM wsl.exe /F`) | **Yes** | Those are relays. The build's parent is PID 1 inside the distro, and none of its fds belong to a relay. |
| Signing out of Windows | **No** | The WSL2 instance belongs to the sign-in session. Lock the screen instead. |
| Sleep / hibernate | **Usually no** | The VM is suspended, and in practice usually does not come back healthy. Set both timeouts to 0. |
| `wsl --shutdown` | **No — always fatal** | Stops the utility VM outright. Nothing in user space can defend against it. This is the one command that will always kill a build. |
| `wsl --terminate <distro>` | **No** | Same, for one distro. Also skips `build.sh`'s cleanup, so it leaks the loop device. |
| Host reboot, Windows Update restart, `wsl --update` | **No** | |
| Docker Desktop restarting or updating | **Possibly** | It can restart the WSL subsystem. Do not touch it during a build. |
| C: running out of space | **No, and it lies about why** | The guest sees I/O errors that look like filesystem corruption, hours in. |
| A stage failing | n/a | The image is preserved. Resume with `-From <stage>`. |

After a death, `build-status.ps1` tells you *which* death it was: a lock file
with a **stale heartbeat** means the whole distro went away underneath the
build (shutdown, sign-out, sleep, reboot); a lock file with a fresh heartbeat
and no `build.sh` means the build process itself died; a `last-run` record
means it exited on its own and the record has the exit code.

---

## Why `nohup setsid` was not enough, and what changed

`setsid` removes the controlling terminal. `nohup` ignores `SIGHUP`. Neither
of them **changes the file descriptors the process inherited**.

Launched as

```
wsl -d Ubuntu-22.04 -u root -- bash -lc 'nohup setsid ./build.sh &'
```

from a script or an agent, `wsl.exe`'s stdout is a **pipe owned by the
caller**. `nohup` only redirects stdout when stdout is a *tty* — here it is a
pipe, so nohup leaves it alone, and the detached build inherits that pipe as
fd 1. `build.sh` then routes everything through
`exec > >(stdbuf -oL tee -a "$LOGFILE")`, so `tee`'s stdout is that pipe too.

When the caller exits, the read end closes. The next write raises `SIGPIPE`
and the build dies — hours later, with nothing in the log, looking exactly
like "something killed it". That is the most likely mechanism behind the two
mystery deaths, and it is invisible to both `setsid` and `nohup`.

The launcher fixes it directly:

```
setsid --fork nohup /bin/bash runner.sh </dev/null >>LOGFILE 2>&1
```

* `</dev/null` and `>>LOGFILE 2>&1` — **all three fds are a device or a file
  on the distro's ext4.** No descriptor in the build belongs to any Windows
  process, so nothing a caller does can break a write.
* `setsid --fork` — new session, no controlling terminal, and setsid's parent
  exits immediately, so the runner is reparented to PID 1 (`/init`) within
  milliseconds. It is a distro-level daemon.
* `nohup` — belt and braces; the ignored `SIGHUP` is inherited across every
  `exec`, so all nine stages inherit it too.
* **The launching `wsl.exe` returns in about a second.** This is why
  `Start-Process -WindowStyle Hidden` is *not* used: a hidden window is still
  a process in the caller's tree, and a Windows Job Object with
  `JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE` — which terminals and agent harnesses
  do use — kills everything in the job when the caller dies. The best defence
  against having your process killed is not owning one.

### Proving it is really detached — 10 minutes, not 15 hours

Do this once, on a build you are willing to stop:

1. `.\tools\run-build-detached.ps1` and read the "here is the proof" block.
   `PPID` must be `1`, `tty` must be `<none>`, and **fd 1 must be the log
   file, not `pipe:[...]`**. The script complains loudly if any of that is
   wrong.
2. Close every terminal, then `taskkill /IM wsl.exe /F` from a new one.
3. Wait five minutes. Run `.\tools\build-status.ps1`.
4. The log is still growing and the pid is unchanged → it is detached.

That is the definitive test. What it does **not** prove is that WSL2 will
never reap a distro whose only remaining processes are parented to PID 1 over
a full 15-hour night; everything observable says it does not (this is how
people run `sshd` in WSL), but the only proof of that is an overnight run that
survives with every window closed.

### Keeping the distro alive

WSL2 stops the VM when the last process in it exits, so **the build is its own
keepalive** — while it runs, the distro stays up. The runner also touches
`/var/tmp/fpms-os-build/heartbeat` every 30 s, which guarantees a live process
across the seam where `build.sh` exits and the runner finishes its bookkeeping,
and gives `build-status.ps1` its "was it the build or the whole VM?" signal.

If you want no doubt at all, put this in `%USERPROFILE%\.wslconfig` while
building (it governs how long the VM lingers with nothing running in it):

```ini
[wsl2]
vmIdleTimeout=-1
```

Nothing on this list defends against `wsl --shutdown`. Nothing can.

---

## The Git Bash / MSYS path trap

You are meant to drive these from **PowerShell**, where it does not arise.

From Git Bash or any other MSYS2 shell, MSYS rewrites anything that looks like
a Unix absolute path into a Windows path **before `wsl.exe` sees it**:

```sh
$ wsl -d Ubuntu-22.04 -u root -- ls /mnt/c/Users
ls: cannot access 'C:/Users': No such file or directory
```

The argument was mangled in the shell, not in WSL. Prefix the call:

```sh
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-22.04 -u root -- ls /mnt/c/Users
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-22.04 -u root -- cat /var/tmp/fpms-os-build/build.lock
```

A single leading `/` is enough to trigger it, so every copy-pasted `/root/...`
and `/var/tmp/...` in this file is affected when pasted into Git Bash. The
scripts themselves are immune: PowerShell passes arguments through unchanged,
and they hand bash its script on **stdin** rather than as a quoted argument,
so there is no shell-quoting layer to get wrong in either direction.

---

## Where things are

| Thing | Path (inside the distro) |
|---|---|
| Lock file for the running build | `/var/tmp/fpms-os-build/build.lock` |
| Heartbeat | `/var/tmp/fpms-os-build/heartbeat` |
| Record of the last finished run | `/var/tmp/fpms-os-build/last-run` |
| Generated runner | `/var/tmp/fpms-os-build/runner.sh` |
| Wrapper log (everything, including the launch banner) | `<repo>/.build/detached-<timestamp>.log` |
| `build.sh`'s own log (stage timings) | `<repo>/.build/build-<timestamp>.log` |
| The image | `<repo>/fpms-os-<version>-<date>.img[.xz]` |

State lives in `/var/tmp`, not in `.build/`, on purpose: `.build/` is the
directory an operator deletes to reclaim disk and the directory that holds the
live bind mounts. The lock and the heartbeat have to outlive both.

---

## Further reading

* `docs/BUILDING.md` §1 — WSL2 specifics, why never `/mnt/c`, the parent
  `rover/` tree stage 30 needs.
* `docs/BUILDING.md` §2 — disk space, and exactly where the build dies at each
  level of free space.
* `docs/BUILDING.md` §3 — measured timings, and why stage 10 looks frozen.
* `docs/BUILDING.md`, failure mode **D** — leaked loop devices and the
  unmount order.
