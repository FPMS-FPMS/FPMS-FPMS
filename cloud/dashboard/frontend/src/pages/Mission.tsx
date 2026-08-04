import { useEffect, useRef, useState } from "react";

import { Card, CardHeader } from "../components/Card";
import { ArenaMap } from "../components/ArenaMap";
import { ErrorBoundary } from "../components/ErrorBoundary";
import { StatusPill } from "../components/StatusPill";
import {
  MissionConsole,
  MissionEncoders,
  Stat,
  useMissionConsole,
} from "../components/MissionConsole";
import { useChannel } from "../lib/ws";
import { useThings } from "../lib/things";
import { emptyCapabilities, useRoverCapabilities } from "../lib/capabilities";
import {
  MISSION_ABORT_NOTE,
  num,
  poseEnvelopeFromMm,
  readEnvelope,
  useMission,
  useRoverEvents,
  type MissionPlan,
} from "../lib/mission";

/**
 * RUN THE ROVER — the page the mission console lives on.
 *
 * Before this existed, "send the rover to a corner" was assembled by the
 * operator out of three places: a PLAN row and a mission row buried under the
 * joystick on Drive, a progress card next to it, and the fleet bar in the
 * Layout that could abort but could not start. The one task this machine
 * exists to do had no home. This page is that home.
 *
 * THE CONTROLS THEMSELVES ARE NOT IN THIS FILE. They are
 * `components/MissionConsole` — the arm gate, the four targets, PLAN, FOLLOW,
 * the backend, SET COORDINATE and TEST ENCODERS — because the operator standing
 * beside the arena watches the LiDAR tab, and the buttons had to be on the page
 * with the live map too. Every rule about when a mission may be sent is stated
 * once, there, and both pages inherit it rather than reimplementing it. Read
 * that file for WHY FOLLOW is two clicks behind a plan it has seen.
 *
 * What is left here is the page: which rover, the stop button, the loud states,
 * the map with the planned route on it, and what the executor says it is doing.
 *
 * ---------------------------------------------------------------------------
 * THE RULES THIS PAGE INHERITS, EACH FROM A REAL INCIDENT
 * ---------------------------------------------------------------------------
 *   A MISSING NUMBER IS `--`, NEVER 0. The executor omits fields it does not
 *   have; "0 mm remaining" reads as ARRIVED when it means NO IDEA. Every
 *   readout here goes through `mm`/`secs`/`volts` below, which take
 *   `number | null` and cannot be handed a default.
 *
 *   AN UNKNOWN PHASE COUNTS AS RUNNING. `useMission` decides that (see
 *   MISSION_IDLE_PHASES) and this page never second-guesses it. Showing
 *   "idle" while the rover drives is how someone walks up to a moving machine.
 *
 *   STALE TELEMETRY DURING A RUN IS LOUD. `feed.blind` — it was driving and
 *   the feed stopped — gets a full-width rose banner with the instruction
 *   attached, not a chip.
 *
 *   POSE IS DEAD-RECKONED FROM AN ASSUMED START. It is never presented as
 *   measured. The map raises its own amber ASSUMED styling and this page does
 *   not override it; it only forwards the best pose that exists.
 *
 *   ABORT IS `stop`, NOT `mission {name:"abort"}`. See MISSION_ABORT_NOTE:
 *   "abort" is not a commandable mission and the rover answered "unknown
 *   mission" while aborting nothing.
 */

/* --------------------------------------------------------------- formatting */

/**
 * Every readout on this page goes through one of these.
 *
 * They take `number | null` and have NO default parameter, which is the point:
 * there is no call site at which a missing reading can quietly become a zero.
 */
function mm(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${Math.round(v)} mm`;
}

function secs(v: number | null | undefined): string {
  if (v === null || v === undefined) return "--";
  const s = Math.max(0, Math.round(v));
  if (s < 60) return `${s}s`;
  return `${Math.floor(s / 60)}m ${String(s % 60).padStart(2, "0")}s`;
}

function volts(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${v.toFixed(1)} V`;
}

/** "3/7", or "--" when either half is absent. Never "0/0". */
function ratio(i: number | null | undefined, n: number | null | undefined): string {
  return i === null || i === undefined || n === null || n === undefined
    ? "--"
    : `${i}/${n}`;
}

/* --------------------------------------------------------------------- page */

export default function Mission() {
  return (
    <ErrorBoundary label="Mission">
      <MissionPage />
    </ErrorBoundary>
  );
}

function MissionPage() {
  const things = useThings();
  const [picked, setPicked] = useState<string | null>(null);
  const thing = picked && things.includes(picked) ? picked : things[0] ?? null;

  // ---------------------------------------------------------------- feeds ---
  const feed = useMission(thing);
  const m = feed.mission;

  const driveCh = useChannel<any>(thing ? `drive:${thing}` : null);
  const tele = readEnvelope(driveCh.data);
  const poseCh = useChannel<any>(thing ? `pose:${thing}` : null);
  const planCh = useChannel<any>(thing ? `mission_plan:${thing}` : null);
  const events = useRoverEvents(thing);

  // --------------------------------------------------------- capabilities ---
  const capsByThing = useRoverCapabilities();
  const caps = thing ? capsByThing[thing] ?? emptyCapabilities() : emptyCapabilities();

  // ------------------------------------------------------------- console ----
  // The gate, the plans and every command live in the console; the page holds
  // the object so its own stop button can reach `abort`, which is the thing
  // that SHUTS the arm gate.
  const mc = useMissionConsole({ thing, caps, plan: planCh, feed, events, drive: driveCh });

  const running = mc.running;
  const route = mc.focusPlan?.waypoints ?? null;

  const abortRef = useRef(mc.abort);
  abortRef.current = mc.abort;
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") abortRef.current();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  // ------------------------------------------------------------- map pose ---
  /**
   * Best pose that EXISTS, never one we made up.
   *
   * The mission executor wins over the teleop bridge when both have one: they
   * read the same odometry, but the executor is the process actually driving,
   * and two positions on screen with no way to tell which the rover steers by
   * is worse than one. When nothing has a position we pass the heartbeat
   * envelope straight through so `readPose` raises its own amber ASSUMED
   * styling — that fallback is load-bearing and must never be replaced with a
   * manufactured coordinate.
   */
  const mapPose =
    poseEnvelopeFromMm(m?.poseX ?? null, m?.poseY ?? null, m?.poseHeadingDeg ?? null) ??
    poseEnvelopeFromMm(num(tele?.x_mm), num(tele?.y_mm), num(tele?.heading_deg));
  const poseSource =
    mapPose === null
      ? null
      : m?.poseX !== null && m?.poseX !== undefined
        ? "mission executor"
        : "teleop bridge";

  /**
   * Fraction of the planned distance covered, or null.
   *
   * Both halves must be present. A bar drawn from travelled alone would sit at
   * 100 % the instant `distance_remaining_mm` went missing, which reads as
   * ARRIVED — the same lie as rendering a missing number as zero.
   */
  const progress =
    m?.travelledMm !== null && m?.travelledMm !== undefined &&
    m?.remainingMm !== null && m?.remainingMm !== undefined &&
    m.travelledMm + m.remainingMm > 0
      ? Math.min(1, Math.max(0, m.travelledMm / (m.travelledMm + m.remainingMm)))
      : null;

  return (
    <div className="space-y-5">
      <div className="flex flex-wrap items-end justify-between gap-4">
        <div>
          <div className="lbl">Mission console · arm, plan, follow</div>
          <h1 className="h-page mt-1">Run the rover</h1>
        </div>
        <div className="flex flex-wrap items-center gap-2">
          {things.length === 0 ? (
            <span className="chip-warn">no rovers reporting</span>
          ) : (
            things.map((t) => (
              <button
                key={t}
                onClick={() => setPicked(t)}
                className={`chip ${t === thing ? "border-ember-500/40 bg-ember-500/10 text-ember-200" : ""}`}
                title={`Run missions on ${t}. Switching rovers clears every plan and disarms the console.`}
              >
                <span className="font-mono">{t}</span>
              </button>
            ))
          )}
        </div>
      </div>

      {/* ABORT. Sticky at the same offset the Drive tab pins its stop bar, so
          the two consoles have it in the same place; never disabled, because
          greying it out removes the control at exactly the moment the previous
          press might have been the one that failed. It is above everything
          else on the page on purpose, including the arm gate. */}
      <div className="sticky top-[86px] z-20 md:top-[58px]">
        <button
          onClick={mc.abort}
          className="flex w-full items-center justify-center gap-3 rounded-xl border border-rose-500/50 bg-rose-600/25 px-4 py-5 text-lg font-semibold tracking-wide text-rose-50 shadow-lg shadow-black/50 backdrop-blur transition hover:bg-rose-600/40 active:scale-[0.995]"
          title={`Abort the run and halt the motors on ${thing ?? "the selected rover"}. Always enabled, armed or not. Shortcut: Esc. Also disarms this console. ${MISSION_ABORT_NOTE}`}
        >
          <span className="inline-block h-3 w-3 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          ABORT — STOP {thing ? thing.toUpperCase() : "THE ROVER"}
          <span className="rounded border border-rose-300/30 bg-black/30 px-1.5 py-0.5 font-mono text-[11px] font-normal text-rose-200">
            Esc
          </span>
        </button>
      </div>

      {/* BLIND: it was driving, and the telemetry stopped. The single loudest
          state on the page, and the only one that carries an instruction. */}
      {feed.blind && (
        <div className="flex items-start gap-3 rounded-xl border-2 border-rose-500/50 bg-rose-950/50 p-4">
          <span className="mt-0.5 inline-block h-3 w-3 shrink-0 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          <div>
            <div className="text-base font-semibold text-rose-100">
              TELEMETRY LOST WHILE DRIVING — {Math.round((feed.ageMs ?? 0) / 1000)}s
              of silence
            </div>
            <p className="mt-1 text-sm text-rose-200/85">
              The last thing this rover reported was a mission in progress, and
              nothing has arrived since. <b>Treat it as still moving.</b> Every
              number below is history, not state. Press ABORT before approaching
              the arena — a machine you cannot see is not a machine you can
              assume has stopped.
            </p>
          </div>
        </div>
      )}

      {mc.lockReason && (
        <div className="flex items-start gap-3 rounded-xl border-2 border-rose-500/50 bg-rose-950/40 p-4">
          <span className="mt-0.5 inline-block h-3 w-3 shrink-0 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          <div>
            <div className="text-base font-semibold text-rose-100">
              {mc.rosDown ? "ROS DOWN — the rover cannot move" : "LINK STALE — rover state unknown"}
            </div>
            <p className="mt-1 text-sm text-rose-200/85">
              {mc.lockReason} FOLLOW is disabled until it recovers. <b>PLAN, TEST
              ENCODERS and ABORT still work</b> — a preview and a readback move
              nothing, and an abort has to work precisely when the link check
              says things are wrong.
            </p>
          </div>
        </div>
      )}

      {/* ------------------------------------------------- 0/1/2 · the console */}
      <Card>
        <MissionConsole mc={mc} />
      </Card>

      <div className="grid gap-5 lg:grid-cols-2">
        {/* --------------------------------------------------- 3. the map */}
        <Card>
          <CardHeader
            title="The route it will drive"
            subtitle={thing ? `${thing} · arena map` : "no rover selected"}
            right={
              <div className="flex flex-wrap items-center gap-2">
                <span
                  className={route ? "chip font-mono" : "chip font-mono text-slate-500"}
                  title={route ? "The previewed route, drawn as a dashed line" : "Nothing has been previewed"}
                >
                  {route ? `${mc.focus} · ${route.length} pts` : "no route"}
                </span>
                <StatusPill
                  connected={feed.connected}
                  lastAt={feed.lastAt}
                  messages={feed.messages}
                />
              </div>
            }
          />
          {thing ? (
            <ErrorBoundary label={`${thing} arena map`}>
              <ArenaMap thing={thing} poseEnvelope={mapPose ?? poseCh.data} route={route} />
            </ErrorBoundary>
          ) : (
            <div className="rounded-xl border border-white/5 bg-black/30 p-8 text-center text-sm text-slate-400">
              No rover is reporting, so there is nothing to draw. This is not the
              same as an idle rover.
            </div>
          )}

          {mc.focusPlan ? <PlanDetail plan={mc.focusPlan} live={m?.legI ?? null} /> : (
            <p className="mt-3 text-xs text-slate-500">
              No route is drawn. Press PLAN on one of the four runs — the map is
              the only place the rover's intention is visible before it moves.
            </p>
          )}

          <div className="mt-3 flex flex-wrap items-center gap-2 text-[11px] text-slate-500">
            <span className="font-mono">
              pose ·{" "}
              {poseSource ? (
                <span className="text-slate-300">{poseSource}</span>
              ) : (
                <span className="text-amber-300">assumed start corner</span>
              )}
            </span>
            <span>
              Dead-reckoned from an assumed start, and it drifts. The map draws
              an assumed pose in amber for that reason — nothing localises this
              rover, so no number on it is a measurement of where it truly is.
            </span>
          </div>
        </Card>

        {/* ------------------------------------------------- 4. progress */}
        <Card glow={running || feed.blind}>
          <CardHeader
            title="What it is doing now"
            subtitle="Live from the mission executor"
            right={
              <span
                className={
                  feed.blind ? "chip-bad" : running ? "chip-hot" : feed.messages ? "chip" : "chip-warn"
                }
                title={
                  feed.blind
                    ? "The rover was driving when telemetry stopped."
                    : running
                      ? "A phase the executor reports as in progress. Phases this dashboard does not recognise count as RUNNING."
                      : feed.messages
                        ? "The executor reports a phase that positively means not driving."
                        : "Nothing has ever arrived on this channel — the executor may not be running. That is not the same as idle."
                }
              >
                {(running || feed.blind) && (
                  <span
                    className={`inline-block h-2 w-2 shrink-0 rounded-full pulse-dot ${
                      feed.blind ? "bg-rose-300 text-rose-300" : "bg-ember-300 text-ember-300"
                    }`}
                  />
                )}
                {feed.blind
                  ? "DRIVING — TELEMETRY LOST"
                  : running
                    ? "RUNNING"
                    : feed.messages
                      ? "IDLE"
                      : "NO TELEMETRY"}
              </span>
            }
          />

          {!feed.messages ? (
            <p className="text-sm text-slate-400">
              Nothing has arrived on{" "}
              <span className="font-mono">
                {thing ? `mission:${thing}` : "the mission channel"}
              </span>
              . The executor may not be running at all. <b>This is not “idle”</b>{" "}
              — an idle rover reports that it is idle, and this one is reporting
              nothing.
            </p>
          ) : (
            <>
              {/* Route progress. Only drawn when BOTH halves exist — see the
                  `progress` comment; a bar fed by travelled alone would read
                  ARRIVED the moment the remaining distance went missing. */}
              <div className="mb-4">
                <div className="flex items-baseline justify-between">
                  <span className="lbl text-[10px]">route progress</span>
                  <span className="font-mono text-[11px] text-slate-400">
                    {progress === null ? "--" : `${Math.round(progress * 100)}%`}
                  </span>
                </div>
                <div className="mt-1 h-2 w-full overflow-hidden rounded-full bg-white/[0.06]">
                  {progress === null ? null : (
                    <div
                      className={`h-full rounded-full ${feed.blind ? "bg-rose-400/70" : "bg-ember-400/80"}`}
                      style={{ width: `${progress * 100}%` }}
                    />
                  )}
                </div>
                {progress === null ? (
                  <div className="mt-1 text-[10px] text-slate-500">
                    The executor is not reporting both travelled and remaining
                    distance, so there is no fraction to draw. An empty bar here
                    means UNKNOWN, not zero.
                  </div>
                ) : null}
              </div>

              <div className="grid grid-cols-2 gap-x-4 gap-y-4 sm:grid-cols-3">
                <Stat
                  label="mission"
                  value={m?.mission ?? "--"}
                  note={m?.backend ? `via ${m.backend}` : undefined}
                />
                <Stat
                  label="phase"
                  value={m?.phase ?? "--"}
                  note={
                    running && m?.phase
                      ? "in progress"
                      : m?.phase
                        ? "not driving"
                        : undefined
                  }
                />
                <Stat
                  label="leg"
                  value={m?.legLabel ?? m?.leg ?? "--"}
                  note={`leg ${ratio(m?.legI, m?.legsN)}`}
                  accent
                />
                <Stat
                  label="segment"
                  value={ratio(m?.segmentI, m?.segmentsN)}
                  note={m?.segmentKind ?? undefined}
                />
                <Stat label="distance remaining" value={mm(m?.remainingMm)} />
                <Stat label="travelled" value={mm(m?.travelledMm)} />
                <Stat
                  label="ETA"
                  value={secs(m?.etaS)}
                  note={
                    m?.elapsedS !== null && m?.elapsedS !== undefined
                      ? `elapsed ${secs(m.elapsedS)}`
                      : undefined
                  }
                />
                <Stat
                  label="front clearance"
                  value={mm(m?.frontMm)}
                  note="obstacle guard"
                />
                <Stat label="battery" value={volts(m?.battV)} />
                <Stat
                  label="target"
                  value={
                    m?.targetX !== null && m?.targetX !== undefined &&
                    m?.targetY !== null && m?.targetY !== undefined
                      ? `${Math.round(m.targetX)}, ${Math.round(m.targetY)}`
                      : "--"
                  }
                  note="arena mm"
                />
                <Stat
                  label="pose (assumed)"
                  value={
                    m?.poseX !== null && m?.poseX !== undefined &&
                    m?.poseY !== null && m?.poseY !== undefined
                      ? `${Math.round(m.poseX)}, ${Math.round(m.poseY)}`
                      : "--"
                  }
                  note={
                    m?.poseHeadingDeg !== null && m?.poseHeadingDeg !== undefined
                      ? `${Math.round(m.poseHeadingDeg)}° · dead-reckoned`
                      : "dead-reckoned"
                  }
                />
                <Stat
                  label="telemetry age"
                  value={feed.ageMs === null ? "--" : `${Math.round(feed.ageMs / 1000)}s`}
                  note={m?.odomSource ? `odom ${m.odomSource}` : undefined}
                  bad={feed.blind}
                />
              </div>

              <div className="mt-4 flex flex-wrap items-center gap-2">
                {/* Staleness is only shouted about while running: the executor
                    throttles idle telemetry on purpose, so flagging a quiet idle
                    feed would train the operator to ignore the chip that matters. */}
                {feed.stale && running ? (
                  <span className="chip-hot">
                    stale {Math.round((feed.ageMs ?? 0) / 1000)}s — treat as moving
                  </span>
                ) : null}
                {m?.linkOk === false ? <span className="chip-bad">micro-ROS link down</span> : null}
                {m?.lidarOk === false ? (
                  <span className="chip-bad">LiDAR stale — the obstacle guard is blind</span>
                ) : null}
                {m?.poseAssumed ? (
                  <span
                    className="chip-warn"
                    title="Nothing localises this rover; the pose is dead-reckoned from an assumed start and drifts."
                  >
                    pose assumed
                  </span>
                ) : null}
              </div>

              {events.done ? (
                <div className="mt-4 rounded-lg border border-white/10 bg-white/[0.04] px-3 py-2 text-[11px] text-slate-300">
                  <span className="lbl text-[10px]">last outcome</span>{" "}
                  <span className="font-mono">
                    {String(events.done.data.mission ?? events.done.name ?? "--")} ·{" "}
                    {String(events.done.data.outcome ?? events.done.data.state ?? "--")}
                  </span>
                  <div className="mt-0.5 text-[10px] text-slate-500">
                    From events/mission_done, which reports the MEASURED outcome
                    rather than the requested one.
                  </div>
                </div>
              ) : null}
            </>
          )}
        </Card>
      </div>

      {/* ------------------------------------------------- 5. encoders */}
      <MissionEncoders mc={mc} />
    </div>
  );
}

/* --------------------------------------------------------------- components */

/**
 * The planned route in full, under the map.
 *
 * NOMINAL is the operative word and it is stated rather than implied: the
 * executor re-measures its bearing after every leg and inserts correction
 * turns, so the driven path will not match the dashed line exactly. The leg
 * list is what "follows the route" actually means, in order, with the live leg
 * marked when the rover is on it.
 */
function PlanDetail({ plan, live }: { plan: MissionPlan; live: number | null }) {
  return (
    <div className="mt-3 rounded-lg border border-sky-500/30 bg-sky-500/5 px-3 py-2 text-xs text-sky-100/90">
      <div className="font-mono">
        {plan.mission ?? "--"} · {mm(plan.distanceMm)} ·{" "}
        {plan.segments === null ? "--" : plan.segments} segments · ETA{" "}
        {plan.etaS === null ? "--" : `~${secs(plan.etaS)}`}
        {plan.backend ? ` · via ${plan.backend}` : ""}
      </div>

      {plan.routeText ? (
        <div className="mt-1 text-[11px] text-sky-200/80">{plan.routeText}</div>
      ) : null}

      {plan.legs ? (
        <ol className="mt-2 space-y-1">
          {plan.legs.map((l, i) => {
            const isLive = live !== null && l.i !== null && l.i === live;
            return (
              <li
                key={`${l.name ?? "leg"}-${i}`}
                className={`flex flex-wrap items-baseline gap-x-2 rounded px-1.5 py-0.5 font-mono text-[11px] ${
                  isLive ? "bg-ember-500/15 text-ember-100" : "text-sky-100/85"
                }`}
              >
                <span className="opacity-70">{l.i ?? i + 1}.</span>
                <span>{l.label ?? l.name ?? "--"}</span>
                <span className="opacity-70">{l.name ?? ""}</span>
                <span className="opacity-70">
                  {l.x_mm === null || l.y_mm === null
                    ? "--, --"
                    : `${Math.round(l.x_mm)}, ${Math.round(l.y_mm)}`}
                </span>
                <span className="opacity-70">{mm(l.distanceMm)}</span>
                <span className="opacity-70">
                  {l.segments === null ? "--" : `${l.segments} seg`}
                </span>
                {isLive ? <span className="chip-hot text-[9px]">ON THIS LEG</span> : null}
              </li>
            );
          })}
        </ol>
      ) : null}

      <div className="mt-2 text-[11px] text-sky-200/70">
        From{" "}
        {plan.fromX === null || plan.fromY === null
          ? "-- , --"
          : `${Math.round(plan.fromX)}, ${Math.round(plan.fromY)}`}
        {plan.fromHeadingDeg === null ? "" : ` at ${Math.round(plan.fromHeadingDeg)}°`}
        {plan.returnStrategy ? ` · return: ${plan.returnStrategy}` : ""}. Nominal
        route: the executor re-measures its bearing after every leg and inserts
        correction turns, so the driven path will differ from the dashed line.
        {plan.poseAssumed
          ? " The start pose is ASSUMED — nothing localises this rover, so the whole route is only as right as that assumption."
          : ""}
      </div>
    </div>
  );
}
