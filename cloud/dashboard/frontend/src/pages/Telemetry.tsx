import { useEffect, useRef, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { ErrorBoundary } from "../components/ErrorBoundary";
import { CameraView } from "../components/CameraView";
import { useChannel } from "../lib/ws";
import { useThings } from "../lib/things";
// `num` and `readEnvelope` are READ from lib/mission rather than restated here.
// Both encode rules this page depends on — "a missing number is null, never 0"
// and "the bridge envelope is {thing, subtype, ts, data}" — and a second copy is
// a second place for those rules to drift.
import { num, readEnvelope } from "../lib/mission";

/**
 * ===========================================================================
 * TELEMETRY — one rover's odometry, IMU, LiDAR and camera on one screen.
 * ===========================================================================
 *
 * This page exists because those four feeds were on four tabs, and the
 * questions an operator actually asks span them: "the yaw is not moving — is
 * the gyro dead, or is the wheel odometry lying?" cannot be answered from any
 * one of them alone. Nothing here commands the rover. It is a read-only
 * instrument panel, and its entire job is to be TRUE when the rover is silent,
 * which is most of the time.
 *
 * ---------------------------------------------------------------------------
 * THE FIVE RULES THIS PAGE IS BUILT AROUND
 * ---------------------------------------------------------------------------
 * Each one is a defect the dashboard audit found somewhere else in this
 * codebase. They are not stylistic preferences.
 *
 *  1. A MISSING VALUE IS AN EM-DASH, NEVER 0. The AWS tab renders five gauges
 *     as a large mono `0` when the payload is absent, which is indistinguishable
 *     from a measured zero. Every number below goes through `fmt`/`num`, and a
 *     field the rover did not send renders `—`. This matters most on this page
 *     of all of them: `vx = 0` means "stopped" and `vx = —` means "nobody said",
 *     and an operator standing next to a moving rover must be able to tell those
 *     apart at a glance.
 *
 *  2. NO GREEN BADGE FOR A FEED THAT HAS NEVER DELIVERED. `StatusPill` used to
 *     skip its freshness test entirely when `lastAt === null`, so five tabs
 *     showed a pulsing emerald "live · 0 pkts" for a rover that had never sent a
 *     byte. That component has since been fixed, but this page does not lean on
 *     it: each feed here has its OWN staleness budget (a 20 Hz odometry stream
 *     and a 2 Hz camera are not stale at the same age), and one pill driven by
 *     one 3 s constant would disagree with the age printed underneath it.
 *     `useFeed` below computes health against a 1 s wall-clock tick — the same
 *     pattern as `lib/mission.useMission` — because a feed that simply STOPS
 *     produces no re-render, and without a tick it would render "live" forever.
 *
 *  3. `odom:` IS THE ODOMETRY FRAME, NOT THE ARENA FRAME. See the long note on
 *     `OdomCard`. Nothing on this page draws an arena map, on purpose.
 *
 *  4. THE GYRO READS EXACTLY 0.0 AT REST AND THAT IS NOT A DEAD SENSOR. See
 *     `GyroPanel`, which shows the deadbanded value and the raw LSB side by
 *     side and keeps a running min/max of the raw so the claim "it does vary"
 *     is evidence on screen rather than a sentence in a comment.
 *
 *  5. ANYTHING THAT INDEXES INTO A PAYLOAD IS INSIDE AN ErrorBoundary. A
 *     partial envelope crashing the page is a recorded failure here
 *     (`pose.data.data.x_m.toFixed()` blanked the whole dashboard once), and
 *     ThermalView/AnalystPanel are still unwrapped. Every card below is
 *     wrapped, individually, so one malformed feed costs one card.
 *
 * ---------------------------------------------------------------------------
 * CHANNELS
 * ---------------------------------------------------------------------------
 *   odom:<thing>    x_m, y_m, yaw_deg, vx, vyaw, frame_source
 *   imu:<thing>     angular_velocity{x,y,z}, linear_acceleration{x,y,z},
 *                   diag_raw_gyro_z
 *   scan:<thing>    ranges[], angle_min, angle_increment, valid_count,
 *                   min_range_m
 *   camera:<thing>  the existing MQTT camera envelope
 *   drive:<thing>   the existing teleop/firmware health envelope
 *
 * Every field of every one of them is treated as optional, because every one of
 * them is: the publishers omit what they do not have rather than zero-filling,
 * and firmware revisions add fields without warning.
 */

/** A value nobody reported. NEVER "0", and never an empty cell. */
const DASH = "—";

/**
 * Per-feed staleness budgets, milliseconds.
 *
 * These are deliberately different numbers. Odometry and IMU are fast loops and
 * a two-second gap in either is a fault; a camera frame is expensive and 5 s
 * between frames is a slow link, not a dead one. Collapsing them onto one
 * constant would either cry wolf on the camera or stay quiet while the control
 * loop's own inputs went missing.
 */
const STALE_MS = {
  odom: 2000,
  imu: 2000,
  scan: 4000,
  camera: 5000,
  drive: 5000,
  /**
   * The board bridge publishes on a fixed 5 Hz timer (FPMS_BRIDGE_MQTT_HZ), on
   * its own daemon thread, whether or not the rover is moving — so a gap here
   * is the publisher dying, never a parked rover. 3 s is the same budget the
   * payload applies to itself in its `fresh` flag, so this pill and that flag
   * cannot disagree about what stale means.
   */
  board: 3000,
} as const;

/**
 * How far the backend's own timestamp may lag the moment we received the frame
 * before we call the frame a REPLAY rather than news.
 *
 * `hub._latest` on the backend replays the last frame ever broadcast on a
 * channel to every new subscriber, with no expiry. `lib/ws` stamps `lastAt` on
 * receipt and never reads the envelope's own `ts`, so a rover that died an hour
 * ago hands a freshly-loaded page a "live · 1 pkts" feed. Comparing the two
 * catches it.
 *
 * The threshold is generous because it is a difference between TWO CLOCKS — the
 * browser's and the dashboard host's — which need not agree, and a false
 * "replayed" warning would be its own lie. Half a minute is far longer than any
 * plausible skew on a LAN and far shorter than a session.
 */
const REPLAY_SKEW_MS = 30_000;

/* ------------------------------------------------------------- feed health --- */

type FeedHealth = "live" | "stale" | "never";

type Feed = {
  key: string;
  label: string;
  /** The bridge subtype, known even when no rover is selected. */
  subtype: string;
  /** null when no rover is selected — no socket is opened at all. */
  channel: string | null;
  /** The payload, or null when nothing has ever arrived. */
  data: Record<string, unknown> | null;
  /** The whole envelope, for the one consumer (CameraView) that wants it. */
  envelope: unknown;
  health: FeedHealth;
  /** Since WE received the newest frame. null = never received one. */
  ageMs: number | null;
  /** Since the BACKEND stamped it. Differs from ageMs on a replayed frame. */
  stampAgeMs: number | null;
  replayed: boolean;
  messages: number;
  connected: boolean;
  staleMs: number;
};

/** Epoch seconds or milliseconds, decided by magnitude — as Drive/Control do. */
function normalizeTs(ts: unknown): number | null {
  const n = num(ts);
  if (n === null) return null;
  return n > 1e11 ? n / 1000 : n;
}

/**
 * One channel, with an honest health verdict.
 *
 * The verdict is NEVER derived from `connected` alone. `/ws/{channel}` accepts
 * any channel name unconditionally, so the browser's socket being open proves
 * the dashboard process is running and says nothing whatsoever about whether a
 * rover exists. Only a real `lastAt` can produce "live", and only recently.
 */
function useFeed(
  key: string,
  label: string,
  thing: string | null,
  subtype: string,
  staleMs: number,
  now: number,
): Feed {
  const channel = thing ? `${subtype}:${thing}` : null;
  const ch = useChannel<any>(channel);

  const never = ch.messages === 0 || ch.lastAt === null;
  const ageMs = ch.lastAt === null ? null : Math.max(0, now - ch.lastAt);

  const stampS = normalizeTs((ch.data as Record<string, unknown> | null)?.ts);
  const stampAgeMs = stampS === null ? null : Math.max(0, now - stampS * 1000);
  const replayed =
    !never &&
    stampAgeMs !== null &&
    ageMs !== null &&
    stampAgeMs - ageMs > REPLAY_SKEW_MS;

  const health: FeedHealth = never
    ? "never"
    : replayed || (ageMs !== null && ageMs > staleMs)
      ? "stale"
      : "live";

  return {
    key,
    label,
    subtype,
    channel,
    data: readEnvelope(ch.data),
    envelope: ch.data,
    health,
    ageMs,
    stampAgeMs,
    replayed,
    messages: ch.messages,
    connected: ch.connected,
    staleMs,
  };
}

/** 1 s wall-clock tick. Staleness is a function of time, not of arriving data. */
function useNow(): number {
  const [now, setNow] = useState<number>(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);
  return now;
}

/**
 * The freshness badge.
 *
 * Three states, and the third one is the point. "Nothing has ever arrived" is
 * not a dropout and must not be dressed as one — it gets a hollow, still,
 * grey dot, so an operator does not go hunting for a link that never existed.
 * Green is only ever reachable from a real, recent timestamp.
 */
function FeedPill({ feed }: { feed: Feed }) {
  if (feed.channel === null) {
    return (
      <span
        className="chip font-mono"
        title="No rover is selected, so this page has not opened a socket for this channel."
        role="status"
      >
        <span className="inline-block h-2 w-2 shrink-0 rounded-full border border-slate-600 bg-transparent" />
        not subscribed
      </span>
    );
  }

  const cls =
    feed.health === "live" ? "chip-ok" : feed.health === "stale" ? "chip-warn" : feed.connected ? "chip" : "chip-warn";
  const dot =
    feed.health === "live"
      ? "bg-emerald-300 text-emerald-300 pulse-dot"
      : feed.health === "stale"
        ? "bg-amber-300 text-amber-300 pulse-dot"
        : feed.connected
          ? "border border-slate-500 bg-transparent"
          : "bg-amber-300 text-amber-300 pulse-dot";
  const word =
    feed.health === "live"
      ? `live · ${ageWord(feed.ageMs)}`
      : feed.health === "stale"
        ? feed.replayed
          ? "REPLAYED"
          : `NO SIGNAL · ${ageWord(feed.ageMs)}`
        : "NO DATA";

  const title =
    feed.health === "never"
      ? feed.connected
        ? `Nothing has ever arrived on ${feed.channel}. The dashboard websocket is open, but /ws/{channel} accepts any channel name — an open socket is not evidence that a rover is publishing.`
        : `Nothing has ever arrived on ${feed.channel}, AND the browser cannot reach the dashboard backend. Fix the dashboard link before reading anything into the rover's silence.`
      : feed.replayed
        ? `This frame was handed over by the backend's replay cache on connect: the dashboard stamped it ${ageWord(feed.stampAgeMs)} ago but we only received it ${ageWord(feed.ageMs)} ago. It is history, not news. (Assumes this browser's clock and the dashboard host's clock roughly agree.)`
        : feed.health === "stale"
          ? `No frame on ${feed.channel} for over ${(feed.staleMs / 1000).toFixed(0)}s.`
          : `Newest frame ${ageWord(feed.ageMs)} ago on ${feed.channel}.`;

  return (
    <span className={`${cls} font-mono`} title={title} role="status">
      <span className={`inline-block h-2 w-2 shrink-0 rounded-full ${dot}`} />
      <span className={feed.health === "live" ? "" : "font-semibold uppercase tracking-wide"}>
        {word}
      </span>
      <span className="text-2xs opacity-70">· {feed.messages} pkts</span>
    </span>
  );
}

function ageWord(ms: number | null): string {
  if (ms === null) return DASH;
  if (ms < 1000) return `${Math.round(ms)}ms`;
  if (ms < 90_000) return `${(ms / 1000).toFixed(1)}s`;
  return `${Math.round(ms / 60_000)}m`;
}

/* ---------------------------------------------------------------- helpers --- */

/** A number, or the em-dash. There is no third option and no `?? 0`. */
function fmt(v: unknown, digits: number, suffix = ""): string {
  const n = num(v);
  return n === null ? DASH : `${n.toFixed(digits)}${suffix}`;
}

function str(v: unknown): string | null {
  return typeof v === "string" && v.trim() !== "" ? v : null;
}

/** Tri-state: a boolean the rover did not send is not `false`. */
function flag(v: unknown): string {
  return v === true ? "yes" : v === false ? "no" : DASH;
}

function Field({
  label,
  value,
  hint,
  tone,
  title,
  big = false,
}: {
  label: string;
  value: string;
  hint?: string;
  tone?: string;
  title?: string;
  big?: boolean;
}) {
  const missing = value === DASH;
  return (
    <div className="rounded-lg border border-white/5 bg-black/30 px-3 py-2" title={title}>
      <div className="lbl text-[10px]">{label}</div>
      <div
        className={`mt-1 font-mono tabular-nums ${big ? "text-2xl" : "text-sm"} ${
          tone ?? (missing ? "text-slate-600" : "text-slate-100")
        }`}
      >
        {value}
      </div>
      {hint ? <div className="mt-0.5 font-mono text-[10px] text-slate-500">{hint}</div> : null}
    </div>
  );
}

/**
 * What a card says instead of showing numbers when its feed has never spoken.
 *
 * Rendered INSTEAD of the readouts, not above them, so there is no screenful of
 * em-dashes to scan past — and it names the channel, because "which topic is
 * silent" is the first thing anyone debugging this needs.
 */
function Silent({ feed, what }: { feed: Feed; what: string }) {
  return (
    <div className="rounded-lg border border-white/10 bg-black/30 px-3 py-3 text-sm text-slate-400">
      {feed.channel === null ? (
        <>
          <b className="text-slate-300">No rover selected.</b> Nothing is
          subscribed, so there is no {what} to show. Pick a rover above.
        </>
      ) : (
        <>
          <b className="text-slate-300">
            Nothing has ever arrived on{" "}
            <span className="font-mono">{feed.channel}</span>.
          </b>{" "}
          No {what} is being reported. This is not a reading of zero — nothing
          has been read at all.
        </>
      )}
    </div>
  );
}

/* =========================================================== the page ====== */

export default function Telemetry() {
  const things = useThings();
  /**
   * The rovers offered in the picker.
   *
   * `things` is the backend's `things_seen`, which only ever GROWS — nothing
   * leaves that roster for the life of the process — so it means "has reported
   * at some point this session", not "is reporting now". The picker says so
   * rather than printing a confident "reporting", and the per-feed pills are
   * where "now" is answered.
   *
   * The two default bays are offered as well, but they are NOT selected by
   * default: an unpicked page with nothing reporting subscribes to nothing at
   * all, which is the honest state. The operator can opt in to watching a bay
   * that has said nothing, and every pill will then say NO DATA, which is a
   * useful thing to be able to establish deliberately.
   */
  const bays = Array.from(new Set([...things, "rover1", "rover2"])).sort();
  const [picked, setPicked] = useState<string | null>(null);
  const thing = picked ?? things[0] ?? null;

  const now = useNow();
  const odom = useFeed("odom", "Odometry", thing, "odom", STALE_MS.odom, now);
  const imu = useFeed("imu", "IMU", thing, "imu", STALE_MS.imu, now);
  const scan = useFeed("scan", "LiDAR", thing, "scan", STALE_MS.scan, now);
  const camera = useFeed("camera", "Camera", thing, "camera", STALE_MS.camera, now);
  const drive = useFeed("drive", "Firmware", thing, "drive", STALE_MS.drive, now);
  const board = useFeed("board", "Board", thing, "board", STALE_MS.board, now);

  const feeds = [odom, imu, scan, camera, drive, board];
  const anyLive = feeds.some((f) => f.health === "live");
  const anyEver = feeds.some((f) => f.messages > 0);

  return (
    <div className="space-y-6">
      <div className="flex flex-wrap items-end justify-between gap-3">
        <div>
          <div className="lbl">Live telemetry</div>
          <h1 className="h-page mt-1">
            Telemetry — odometry, IMU, LiDAR, camera and board, one rover at a time
          </h1>
        </div>
        <div className="text-xs text-slate-500">
          {things.length
            ? `${things.length} rover${things.length > 1 ? "s" : ""} has reported this session: ${things.join(", ")}`
            : "no rover has reported this session"}
        </div>
      </div>

      {/* ------------------------------------------------- rover + roster --- */}
      <Card>
        <CardHeader
          title={thing ? thing.toUpperCase() : "No rover selected"}
          subtitle="Source"
          right={
            <span
              className={anyLive ? "chip-ok font-mono" : anyEver ? "chip-warn font-mono" : "chip font-mono"}
              title={
                anyLive
                  ? "At least one of the six channels delivered a frame within its own staleness budget."
                  : anyEver
                    ? "Every channel has gone quiet. Frames did arrive earlier in this session."
                    : "Not one of the six channels has ever delivered a frame."
              }
            >
              {anyLive ? "receiving" : anyEver ? "ALL QUIET" : "NOTHING RECEIVED"}
            </span>
          }
        />

        <div className="flex flex-wrap items-center gap-2">
          {bays.map((b) => {
            const reported = things.includes(b);
            return (
              <button
                key={b}
                type="button"
                onClick={() => setPicked(picked === b ? null : b)}
                className={`btn ${b === thing ? "btn-primary" : ""}`}
                title={
                  reported
                    ? `${b} has published to this dashboard at some point since the backend started. That is not a claim that it is publishing now — read the pills below.`
                    : `${b} has never published to this dashboard this session. Selecting it opens the sockets anyway, which is a deliberate way to establish that nothing is coming.`
                }
              >
                {b}
                <span className={`ml-2 text-2xs ${reported ? "text-emerald-300" : "text-slate-500"}`}>
                  {reported ? "has reported" : "silent"}
                </span>
              </button>
            );
          })}
        </div>

        {thing === null && (
          <div className="mt-3 rounded-lg border border-amber-500/30 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
            <b>No rover has reported to this dashboard since it started</b>, so
            nothing is selected and <b>no websocket has been opened</b>. Every
            panel below is showing what it genuinely knows, which is nothing.
            Pick a bay above to subscribe to it anyway.
          </div>
        )}

        {/* The six feeds, in one row, at a glance. This is the answer to "is
            the rover there at all" and it deliberately precedes every number on
            the page — a reading with no freshness beside it is a reading that
            will be believed for as long as it stays on screen. */}
        <div className="mt-4 grid gap-2 sm:grid-cols-2 lg:grid-cols-6">
          {feeds.map((f) => (
            <div key={f.key} className="rounded-lg border border-white/5 bg-black/30 px-3 py-2">
              <div className="flex items-center justify-between gap-2">
                <span className="lbl text-[10px]">{f.label}</span>
              </div>
              <div className="mt-1.5">
                <FeedPill feed={f} />
              </div>
              {/* The channel this tile is about, named even when nothing is
                  selected — "which topic is silent" is the first thing anyone
                  debugging this needs, and it is not knowable from the label. */}
              <div className="mt-1 truncate font-mono text-[10px] text-slate-500">
                {f.channel ?? `${f.subtype}:${DASH}`}
              </div>
            </div>
          ))}
        </div>
      </Card>

      {/* Every card is wrapped INDIVIDUALLY. A partial envelope on one channel
          costs that channel's panel and nothing else. */}
      <div className="grid gap-6 lg:grid-cols-2">
        <ErrorBoundary label="odometry">
          <OdomCard feed={odom} />
        </ErrorBoundary>
        <ErrorBoundary label="IMU">
          <ImuCard feed={imu} drive={drive} thing={thing} />
        </ErrorBoundary>
        <ErrorBoundary label="LiDAR scan">
          <ScanCard feed={scan} />
        </ErrorBoundary>
        <ErrorBoundary label="camera">
          <CameraCard feed={camera} thing={thing} />
        </ErrorBoundary>
      </div>

      <ErrorBoundary label="firmware health">
        <FirmwareCard feed={drive} />
      </ErrorBoundary>

      <ErrorBoundary label="board telemetry">
        <BoardCard feed={board} />
      </ErrorBoundary>

      <Card>
        <CardHeader title="What this page does and does not claim" subtitle="Read this once" />
        <ul className="space-y-2 text-sm leading-relaxed text-slate-400">
          <li>
            <b className="text-slate-300">A dash is not a zero.</b> Every number
            here renders <span className="font-mono">{DASH}</span> when the rover
            did not send the field. <span className="font-mono">0.000</span> means
            the rover sent a zero. On this page in particular that difference is
            the whole point: a velocity of zero and a velocity nobody reported
            look identical on a gauge that defaults to 0.
          </li>
          <li>
            <b className="text-slate-300">A green badge needs a timestamp.</b> An
            open websocket proves the dashboard is running, not that a rover
            exists — the backend accepts any channel name. Each feed above is
            judged against its own staleness budget on a 1 s clock tick, so a
            stream that simply stops goes amber on its own without any new data
            arriving to trigger it.
          </li>
          <li>
            <b className="text-slate-300">Nothing here is an arena position.</b>{" "}
            The odometry card is in the odometry frame. There is no map on this
            page on purpose — see the note on that card.
          </li>
          <li>
            <b className="text-slate-300">This page sends nothing.</b> No
            control, no connect, no publish. It only subscribes.
          </li>
        </ul>
      </Card>
    </div>
  );
}

/* ========================================================== odometry ======= */

/**
 * WHY THERE IS NO MAP ON THIS CARD.
 *
 * `odom:` is the ODOMETRY frame. Its origin is wherever the integrator was last
 * zeroed — a boot, a `set_coordinate`, a micro-ROS reconnect — and its axes are
 * the rover's, not the arena's. The arena frame used by every map on this
 * dashboard has its origin at the bottom-left corner of the floor and is in
 * millimetres. The two coincide only by accident.
 *
 * Plotting an odometry pose on an arena map is how a dashboard states, with a
 * picture and no hedging, that the rover is somewhere it has never claimed to
 * be. Worse, the failure is stable and silent: pointing at the wrong odometry
 * topic (`/odom` versus `/odom_raw`) offsets the whole frame by metres while
 * every number keeps updating smoothly and every velocity stays correct. That is
 * exactly why `frame_source` is printed at the top of this card, in full, and
 * why an absent `frame_source` is called out rather than shrugged off: an
 * odometry position whose frame is unstated is not usable for anything.
 *
 * The numbers are still worth having. Velocities and the yaw RATE are
 * frame-independent, and the position is a perfectly good relative measurement —
 * "it has moved 40 cm since the last zero" — as long as nothing turns it into a
 * coordinate on a floor.
 */
function OdomCard({ feed }: { feed: Feed }) {
  const d = feed.data;
  const frameSource = str(d?.frame_source);
  const yaw = num(d?.yaw_deg);
  const vx = num(d?.vx);
  const vyaw = num(d?.vyaw);
  const xM = num(d?.x_m);
  const yM = num(d?.y_m);

  return (
    <Card>
      <CardHeader
        title="Odometry"
        subtitle="odom: · dead-reckoned · NOT the arena frame"
        right={<FeedPill feed={feed} />}
      />

      {/* Above everything, including the staleness state, because it is true
          whether or not the feed is live. */}
      <div className="mb-4 rounded-xl border-2 border-sky-500/40 bg-sky-500/5 p-3">
        <div className="flex flex-wrap items-center justify-between gap-2">
          <span className="text-sm font-semibold text-sky-100">
            These coordinates are in the ODOMETRY frame.
          </span>
          <span
            // Amber only when a payload ARRIVED without naming its frame —
            // that is a real gap. Nothing arriving at all is not a fault of the
            // publisher's and does not get a warning colour.
            className={
              frameSource || feed.messages === 0 ? "chip font-mono" : "chip-warn font-mono"
            }
            title={
              frameSource
                ? "The topic or frame the publisher says these numbers came from."
                : feed.messages === 0
                  ? "No odometry payload has arrived, so nothing has named a frame yet."
                  : "A payload arrived but did not say which frame this is. An odometry position with an unstated frame cannot be compared with anything."
            }
          >
            frame_source · {frameSource ?? DASH}
          </span>
        </div>
        <p className="mt-2 text-xs leading-relaxed text-sky-100/80">
          The origin is wherever the integrator was last zeroed — a boot, a
          re-zero, a micro-ROS reconnect — and the axes are the rover's, not the
          arena's. <b>This is not a position on the arena floor</b>, it is not
          plotted on a map here, and it must not be copied into anything that
          expects arena millimetres. Treat it as displacement since the last
          zero. If <span className="font-mono">frame_source</span> ever changes
          between runs, every number below shifted with it and nothing on screen
          would otherwise have said so.
        </p>
      </div>

      {feed.messages === 0 ? (
        <Silent feed={feed} what="odometry" />
      ) : (
        <>
          <div className="grid grid-cols-2 gap-3 sm:grid-cols-3">
            <Field
              label="x (odom frame)"
              value={fmt(xM, 3, " m")}
              hint={xM === null ? undefined : `${Math.round(xM * 1000)} mm`}
              big
            />
            <Field
              label="y (odom frame)"
              value={fmt(yM, 3, " m")}
              hint={yM === null ? undefined : `${Math.round(yM * 1000)} mm`}
              big
            />
            <Field
              label="Yaw"
              value={fmt(yaw, 1, "°")}
              hint="integrated, drifts"
              big
            />
            <Field
              label="vx — forward"
              value={fmt(vx, 3, " m/s")}
              tone={vx !== null && Math.abs(vx) > 0.005 ? "text-ember-300" : undefined}
              hint={vx === null ? undefined : vx === 0 ? "a reported zero" : undefined}
            />
            <Field
              label="vyaw — turn rate"
              value={fmt(vyaw, 3, " rad/s")}
              hint={vyaw === null ? undefined : vyaw === 0 ? "a reported zero" : undefined}
              tone={vyaw !== null && Math.abs(vyaw) > 0.01 ? "text-ember-300" : undefined}
            />
            <Field
              label="vyaw in degrees"
              value={vyaw === null ? DASH : `${((vyaw * 180) / Math.PI).toFixed(1)} °/s`}
              hint="derived from vyaw"
            />
          </div>

          <p className="mt-3 text-xs leading-relaxed text-slate-500">
            Position is integrated wheel odometry. Nothing on this rover
            localises against the world, so the error only ever grows — a
            position that has been driving for a minute is worth less than one
            that was zeroed ten seconds ago. The velocities are the honest half
            of this feed: they are instantaneous and frame-independent, so{" "}
            <span className="font-mono">vx</span> and{" "}
            <span className="font-mono">vyaw</span> are the fields to watch when
            asking whether the machine is moving.
          </p>
        </>
      )}
    </Card>
  );
}

/* =============================================================== IMU ======= */

type RawTrack = { n: number; min: number; max: number };

const EMPTY_TRACK: RawTrack = { n: 0, min: NaN, max: NaN };

/**
 * Running min/max of `diag_raw_gyro_z` for this session.
 *
 * This is the evidence behind the claim on this card. Saying "the raw LSB does
 * vary" in prose is worth nothing to an operator looking at a yaw rate pinned
 * at 0.000; showing that the raw count has taken 41 distinct-enough values
 * between −38 and +36 since the page opened is a measurement, and it settles
 * the question the deadband raises — is the chip alive? — without anybody
 * having to pick the rover up.
 *
 * Keyed off the message counter rather than the value, so a repeated identical
 * reading still counts as a sample. That matters: a raw count that does NOT
 * move across a hundred samples is itself the interesting answer, and a
 * value-keyed effect would never have recorded the hundred.
 */
function useRawTrack(value: number | null, messages: number, thing: string | null): RawTrack {
  const [track, setTrack] = useState<RawTrack>(EMPTY_TRACK);
  const seenRef = useRef(0);

  // A different rover's gyro is not this rover's. Carrying the spread across a
  // switch would attribute one machine's liveness to another.
  useEffect(() => {
    seenRef.current = 0;
    setTrack(EMPTY_TRACK);
  }, [thing]);

  useEffect(() => {
    if (messages === seenRef.current) return;
    seenRef.current = messages;
    if (value === null) return;
    setTrack((t) => ({
      n: t.n + 1,
      min: Number.isFinite(t.min) ? Math.min(t.min, value) : value,
      max: Number.isFinite(t.max) ? Math.max(t.max, value) : value,
    }));
  }, [messages, value]);

  return track;
}

function ImuCard({
  feed,
  drive,
  thing,
}: {
  feed: Feed;
  drive: Feed;
  thing: string | null;
}) {
  const d = feed.data;
  const av = (d?.angular_velocity ?? null) as Record<string, unknown> | null;
  const la = (d?.linear_acceleration ?? null) as Record<string, unknown> | null;

  const gz = num(av?.z);
  const raw = num(d?.diag_raw_gyro_z);
  const track = useRawTrack(raw, feed.messages, thing);

  // The firmware's own IMU verdicts, off the drive channel. Cross-referenced
  // here rather than left on the Drive tab, because "is the chip initialised"
  // is the exact question a gyro reading 0.000 provokes.
  const imuInit = drive.data?.imu_init_ok;
  const whoOk = drive.data?.imu_who_am_i_readok;

  return (
    <Card>
      <CardHeader title="IMU" subtitle="imu: · gyro + accelerometer" right={<FeedPill feed={feed} />} />

      {feed.messages === 0 ? (
        <>
          <Silent feed={feed} what="IMU data" />
          <DeadbandNote />
        </>
      ) : (
        <>
          <GyroPanel gz={gz} raw={raw} track={track} imuInit={imuInit} whoOk={whoOk} />

          <div className="mt-4 grid grid-cols-3 gap-3">
            <Field label="ω x — roll rate" value={fmt(av?.x, 4, " rad/s")} />
            <Field label="ω y — pitch rate" value={fmt(av?.y, 4, " rad/s")} />
            <Field
              label="ω z — yaw rate"
              value={fmt(av?.z, 4, " rad/s")}
              tone={gz === 0 ? "text-amber-300" : undefined}
              hint={gz === 0 ? "exactly zero" : undefined}
            />
            <Field label="a x" value={fmt(la?.x, 3, " m/s²")} />
            <Field label="a y" value={fmt(la?.y, 3, " m/s²")} />
            <Field
              label="a z"
              value={fmt(la?.z, 3, " m/s²")}
              hint="≈9.81 at rest, upright"
            />
          </div>

          <p className="mt-3 text-xs leading-relaxed text-slate-500">
            The accelerometer is the sanity check on the whole chip: with the
            rover sitting level and still, <span className="font-mono">a z</span>{" "}
            should read about 9.81 m/s² and the other two about 0. If it does and
            the yaw rate does not move, the chip is alive and the yaw rate is
            being filtered, not lost.
          </p>
        </>
      )}
    </Card>
  );
}

/**
 * THE GYRO READS EXACTLY 0.0 AT REST. THIS IS NOT A DEAD SENSOR.
 *
 * The firmware applies a deadband before publishing `angular_velocity.z`, so a
 * stationary rover reports a hard 0.0 rather than the small noisy value the
 * chip is actually producing. Read cold, that is indistinguishable from a
 * sensor that has fallen off the bus — and this project has spent real time
 * chasing it as exactly that.
 *
 * `diag_raw_gyro_z` is the raw LSB count from the same axis, published
 * unfiltered. It moves. Showing the two side by side, with the raw's spread
 * since the page opened, turns "trust me, it's fine" into something an operator
 * can check in three seconds.
 *
 * Note what this panel does NOT do: it never rewrites the 0.0, never
 * substitutes the raw for it, and never hides it. The deadbanded value is what
 * anything downstream — heading hold, the mission executor's turn logic —
 * actually receives, so it stays on screen at full size. The point is to
 * annotate it, not to correct it.
 */
function GyroPanel({
  gz,
  raw,
  track,
  imuInit,
  whoOk,
}: {
  gz: number | null;
  raw: number | null;
  track: RawTrack;
  imuInit: unknown;
  whoOk: unknown;
}) {
  const deadbanded = gz === 0;
  const varies = track.n >= 2 && Number.isFinite(track.min) && track.max > track.min;
  const spread = varies ? track.max - track.min : null;

  return (
    <div className="rounded-xl border-2 border-amber-500/40 bg-amber-500/5 p-3">
      <div className="flex flex-wrap items-center justify-between gap-2">
        <span className="lbl">Yaw rate — the one that reads zero</span>
        <span
          className={raw === null ? "chip-warn" : varies ? "chip-ok" : "chip"}
          title={
            raw === null
              ? "This payload carries no diag_raw_gyro_z, so there is no independent evidence about the chip on this page."
              : varies
                ? "The raw LSB count has taken more than one value since this page opened — the chip is producing data."
                : "The raw LSB count has not changed yet. That is not proof of anything until a few samples have been seen."
          }
        >
          {raw === null ? "NO RAW DIAGNOSTIC" : varies ? "raw is varying" : "raw not varied yet"}
        </span>
      </div>

      <div className="mt-2 grid gap-3 sm:grid-cols-2">
        <div className="rounded-lg border border-white/5 bg-black/40 px-3 py-2">
          <div className="lbl text-[10px]">angular_velocity.z — published</div>
          <div
            className={`mt-1 font-mono text-3xl font-semibold tabular-nums ${
              gz === null ? "text-slate-600" : deadbanded ? "text-amber-300" : "text-slate-100"
            }`}
          >
            {gz === null ? DASH : gz.toFixed(4)}
            <span className="ml-1 text-sm text-slate-500">rad/s</span>
          </div>
          <div className="mt-0.5 font-mono text-[10px] text-slate-500">
            {gz === null
              ? "the payload did not carry it"
              : deadbanded
                ? "EXACTLY 0.0 — deadbanded, see below"
                : "moving"}
          </div>
        </div>

        <div className="rounded-lg border border-white/5 bg-black/40 px-3 py-2">
          <div className="lbl text-[10px]">diag_raw_gyro_z — raw LSB</div>
          <div
            className={`mt-1 font-mono text-3xl font-semibold tabular-nums ${
              raw === null ? "text-slate-600" : "text-slate-100"
            }`}
          >
            {raw === null ? DASH : raw}
          </div>
          <div className="mt-0.5 font-mono text-[10px] text-slate-500">
            {raw === null
              ? "not in this payload"
              : track.n === 0
                ? "first sample"
                : `${track.n} samples · min ${track.min} · max ${track.max}${
                    spread === null ? "" : ` · spread ${spread}`
                  }`}
          </div>
        </div>
      </div>

      <DeadbandNote gz={gz} raw={raw} varies={varies} imuInit={imuInit} whoOk={whoOk} />
    </div>
  );
}

function DeadbandNote({
  gz,
  raw,
  varies,
  imuInit,
  whoOk,
}: {
  gz?: number | null;
  raw?: number | null;
  varies?: boolean;
  imuInit?: unknown;
  whoOk?: unknown;
} = {}) {
  return (
    <div className="mt-3 text-xs leading-relaxed text-amber-100/90">
      <b>
        A yaw rate of exactly 0.0 at rest is the firmware's deadband, not a dead
        sensor.
      </b>{" "}
      The published{" "}
      <span className="font-mono">angular_velocity.z</span> is filtered before it
      leaves the board, so a stationary rover reports a hard zero rather than the
      small noisy value the chip is producing.{" "}
      <span className="font-mono">diag_raw_gyro_z</span> is the same axis
      unfiltered, and it is shown beside it precisely so nobody has to take that
      on trust.
      {gz === 0 && varies === true && (
        <>
          {" "}
          <b className="text-emerald-200">
            Right now that is confirmed on screen: the published value is pinned
            at 0.0 while the raw count is moving.
          </b>
        </>
      )}
      {gz === 0 && raw !== null && raw !== undefined && varies === false && (
        <>
          {" "}
          <b>
            Right now the raw count has not moved either — that is not yet
            evidence of a fault, only an absence of evidence. Give it a few
            seconds, or nudge the rover.
          </b>
        </>
      )}
      {gz === 0 && (raw === null || raw === undefined) && (
        <>
          {" "}
          <b>
            No raw diagnostic is arriving, so this page cannot tell you whether
            the chip is alive — only that the published value is zero.
          </b>
        </>
      )}
      {(imuInit !== undefined || whoOk !== undefined) && (
        <div className="mt-2 font-mono text-[10px] text-slate-400">
          firmware says · imu_init_ok {flag(imuInit)} · who_am_i read {flag(whoOk)}{" "}
          <span className="text-slate-500">(from drive:, not from this feed)</span>
        </div>
      )}
      <div className="mt-2 text-amber-100/70">
        The consequence is operational, not cosmetic: anything that integrates
        this value for heading gets nothing while the rover is turning slowly.
        Do not read a zero here as "the rover is not rotating".
      </div>
    </div>
  );
}

/* ============================================================== LiDAR ====== */

/**
 * A range bin at or fractionally above the driver's maximum is its saturation
 * value — "nothing out there" — and 0 or less is "no return". Neither is a
 * surface, and drawing either paints a phantom ring or a cluster on the sensor.
 */
function usableRange(v: unknown, maxM: number | null): number | null {
  const n = num(v);
  if (n === null || n <= 0) return null;
  if (maxM !== null && n >= maxM * 0.995) return null;
  return n;
}

function ScanCard({ feed }: { feed: Feed }) {
  const d = feed.data;
  const ranges = Array.isArray(d?.ranges) ? (d!.ranges as unknown[]) : null;
  const angleMin = num(d?.angle_min);
  const angleInc = num(d?.angle_increment);
  const validCount = num(d?.valid_count);
  const minRangeM = num(d?.min_range_m);
  const rangeMaxM = num(d?.range_max_m);

  // Derived from the array we were actually given, and labelled as derived.
  // Printed NEXT TO the publisher's own numbers rather than instead of them: if
  // the two disagree, that disagreement is the most interesting thing on the
  // card, and picking one silently would throw it away.
  let derivedValid: number | null = null;
  let derivedMin: number | null = null;
  if (ranges) {
    let count = 0;
    let nearest = Infinity;
    for (const v of ranges) {
      const r = usableRange(v, rangeMaxM);
      if (r === null) continue;
      count += 1;
      if (r < nearest) nearest = r;
    }
    derivedValid = count;
    derivedMin = nearest === Infinity ? null : nearest;
  }

  // The plot needs the angle mapping, not just the ranges. Without
  // angle_increment, placing bin i at i/n of a full circle is an ASSUMPTION —
  // and a scanner publishing a 270° sector would be drawn wrong by a quarter
  // turn with nothing on screen to say so. So the plot is refused and the
  // numbers stand alone. Requirement: degrade to a numeric summary.
  const plottable =
    ranges !== null && ranges.length > 1 && angleInc !== null && angleInc !== 0;

  const spanDeg =
    ranges && angleInc !== null && ranges.length > 1
      ? (Math.abs(angleInc) * (ranges.length - 1) * 180) / Math.PI
      : null;

  return (
    <Card>
      <CardHeader
        title="LiDAR scan"
        subtitle="scan: · sensor frame · top-down"
        right={<FeedPill feed={feed} />}
      />

      {feed.messages === 0 ? (
        <Silent feed={feed} what="scan" />
      ) : (
        <>
          {plottable ? (
            <div className="flex flex-wrap items-start gap-4">
              <ErrorBoundary label="polar plot">
                <PolarScan
                  ranges={ranges!}
                  angleMin={angleMin}
                  angleInc={angleInc!}
                  rangeMaxM={rangeMaxM}
                />
              </ErrorBoundary>
              <div className="min-w-[12rem] flex-1 space-y-2 text-xs text-slate-500">
                <p className="leading-relaxed">
                  Top-down, in the <b>sensor</b> frame, nose up. Bearings are
                  taken from{" "}
                  <span className="font-mono">angle_min</span> and{" "}
                  <span className="font-mono">angle_increment</span> as
                  published — nothing here assumes a full circle or a bin
                  ordering.
                </p>
                <p className="leading-relaxed">
                  Bins at or above the driver's maximum are its saturation value
                  — "nothing out there" — and are not drawn. Neither are zero or
                  negative bins, which mean "no return". A dot on this plot is a
                  surface something actually reflected off.
                </p>
              </div>
            </div>
          ) : (
            <div className="rounded-lg border border-amber-500/30 bg-amber-500/5 px-3 py-2 text-xs leading-relaxed text-amber-100/90">
              <b>No plot — the numbers below are all there is.</b>{" "}
              {ranges === null
                ? "This payload carries no `ranges` array."
                : ranges.length < 2
                  ? `This payload's \`ranges\` has ${ranges.length} entr${ranges.length === 1 ? "y" : "ies"}, which is not a scan.`
                  : "This payload carries `ranges` but no usable `angle_increment`."}{" "}
              Placing the bins around a circle without the angle mapping would be
              a guess, and a 270° sector drawn as a full circle is wrong by a
              quarter turn with nothing on screen to say so. The summary is
              reported instead.
            </div>
          )}

          <div className="mt-4 grid grid-cols-2 gap-3 sm:grid-cols-3">
            <Field
              label="Beams in payload"
              value={ranges === null ? DASH : String(ranges.length)}
              hint={ranges === null ? "no ranges array" : undefined}
            />
            <Field
              label="valid_count"
              value={validCount === null ? DASH : String(validCount)}
              hint="as published"
            />
            <Field
              label="Usable, derived"
              value={derivedValid === null ? DASH : String(derivedValid)}
              hint="counted from ranges"
              tone={
                validCount !== null && derivedValid !== null && validCount !== derivedValid
                  ? "text-amber-300"
                  : undefined
              }
              title={
                validCount !== null && derivedValid !== null && validCount !== derivedValid
                  ? "The publisher's valid_count and the count derived from the ranges array disagree. One of the two definitions of 'valid' is not what you think it is."
                  : undefined
              }
            />
            <Field
              label="min_range_m"
              value={fmt(minRangeM, 3, " m")}
              hint="as published"
            />
            <Field
              label="Nearest, derived"
              value={fmt(derivedMin, 3, " m")}
              hint="from ranges"
            />
            <Field
              label="Angular span"
              value={spanDeg === null ? DASH : `${spanDeg.toFixed(1)}°`}
              hint={
                angleMin === null
                  ? "angle_min absent"
                  : `from ${((angleMin * 180) / Math.PI).toFixed(1)}°`
              }
            />
          </div>

          {validCount !== null && derivedValid !== null && validCount !== derivedValid && (
            <div className="mt-3 rounded-lg border border-amber-500/30 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
              <b>
                valid_count ({validCount}) and the count derived from{" "}
                <span className="font-mono">ranges</span> ({derivedValid}) do not
                agree.
              </b>{" "}
              Both are shown rather than one being picked, because the
              disagreement is the finding: the publisher and this page are
              applying different definitions of a usable return, and anything
              that trusts one of them is trusting a number it has not checked.
            </div>
          )}
        </>
      )}
    </Card>
  );
}

/**
 * A plain top-down polar plot. No clustering, no classification, no world
 * transform — this is the raw feed, which is what makes it useful when the
 * arena map on the LiDAR tab looks wrong: if this is clean and that is not, the
 * fault is in the projection, not the sensor.
 *
 * Nose UP. Bearing theta is measured CCW from the sensor's forward axis, so a
 * point at theta lands at (cx − R·sin θ, cy − R·cos θ): θ = 0 is straight up,
 * θ = +90° is to the left. Getting this backwards mirrors the world, which is
 * the one rendering bug that looks entirely plausible.
 */
function PolarScan({
  ranges,
  angleMin,
  angleInc,
  rangeMaxM,
  size = 240,
}: {
  ranges: unknown[];
  angleMin: number | null;
  angleInc: number;
  rangeMaxM: number | null;
  size?: number;
}) {
  const ref = useRef<HTMLCanvasElement>(null);

  // Autoscale to the furthest usable return when the driver did not publish a
  // maximum. A fixed scale would either clip a real return or draw every point
  // in a knot at the centre.
  let maxUsable = 0;
  for (const v of ranges) {
    const r = usableRange(v, rangeMaxM);
    if (r !== null && r > maxUsable) maxUsable = r;
  }
  const scaleM = rangeMaxM !== null && rangeMaxM > 0 ? rangeMaxM : maxUsable > 0 ? maxUsable : 1;

  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = size * dpr;
    canvas.height = size * dpr;
    canvas.style.width = `${size}px`;
    canvas.style.height = `${size}px`;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, size, size);

    const cx = size / 2;
    const cy = size / 2;
    const pad = 14;
    const rMax = size / 2 - pad;
    const pxPerM = scaleM > 0 ? rMax / scaleM : 0;

    // Rings and cross-hairs.
    ctx.strokeStyle = "rgba(148,163,184,0.18)";
    ctx.lineWidth = 1;
    for (let i = 1; i <= 4; i++) {
      ctx.beginPath();
      ctx.arc(cx, cy, (rMax * i) / 4, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.beginPath();
    ctx.moveTo(cx - rMax, cy);
    ctx.lineTo(cx + rMax, cy);
    ctx.moveTo(cx, cy - rMax);
    ctx.lineTo(cx, cy + rMax);
    ctx.stroke();

    // Scale label on the outer ring, so the plot is never a picture with no units.
    ctx.fillStyle = "rgba(148,163,184,0.7)";
    ctx.font = "10px ui-monospace, SFMono-Regular, Menlo, monospace";
    ctx.textAlign = "center";
    ctx.fillText("FWD", cx, cy - rMax - 4);
    ctx.textAlign = "left";
    ctx.fillText(`${scaleM.toFixed(1)} m`, cx + 4, cy - rMax + 11);

    // Returns.
    const a0 = angleMin ?? 0;
    ctx.fillStyle = "#f97316";
    for (let i = 0; i < ranges.length; i++) {
      const r = usableRange(ranges[i], rangeMaxM);
      if (r === null) continue;
      const theta = a0 + i * angleInc;
      if (!Number.isFinite(theta)) continue;
      const R = Math.min(r * pxPerM, rMax);
      const px = cx - R * Math.sin(theta);
      const py = cy - R * Math.cos(theta);
      ctx.fillRect(px - 1, py - 1, 2, 2);
    }

    // The sensor itself.
    ctx.fillStyle = "#e2e8f0";
    ctx.beginPath();
    ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
    ctx.fill();
  }, [ranges, angleMin, angleInc, rangeMaxM, scaleM, size]);

  return (
    <div className="shrink-0">
      <canvas
        ref={ref}
        className="rounded-xl bg-black/50 ring-1 ring-white/5"
        aria-label="Top-down LiDAR returns in the sensor frame"
      />
      <div className="mt-1 text-center font-mono text-[10px] text-slate-500">
        sensor frame · nose up · {scaleM.toFixed(1)} m outer ring
      </div>
    </div>
  );
}

/* ============================================================= camera ====== */

function CameraCard({ feed, thing }: { feed: Feed; thing: string | null }) {
  const d = feed.data;
  const frame = typeof d?.frame === "string" ? (d.frame as string) : null;
  const detections = Array.isArray(d?.detections) ? (d!.detections as any[]) : [];

  /**
   * Has a frame ever actually arrived?
   *
   * The same gate the Camera page has, for the same reason. Printing
   * "Detections 0" with no frame received reads as "the camera looked and saw
   * no fire", which on this dashboard is the worst available sentence. And an
   * envelope carrying detections but no image made CameraView emit
   * `<img src="data:image/undefined;base64,undefined">` and size its overlay
   * canvas to NaN — so the component is only ever handed an envelope that
   * carries a frame AND the two dimensions it sizes that canvas from.
   */
  const gotFrame =
    feed.messages > 0 && frame !== null && num(d?.width) !== null && num(d?.height) !== null;
  const fires = detections.filter((x) => x?.kind === "fire" || x?.cls === "fire").length;

  return (
    <Card>
      <CardHeader title="Camera" subtitle="camera: · RGB + YOLO boxes" right={<FeedPill feed={feed} />} />

      <CameraView envelope={gotFrame ? (feed.envelope as any) : null} />

      <div className="mt-3 flex flex-wrap items-center gap-3">
        <span className="lbl">Detections</span>
        <span className={gotFrame ? "chip" : "chip-warn"}>{gotFrame ? detections.length : DASH}</span>
        {gotFrame && fires > 0 && <span className="chip-hot">{fires} fire</span>}
        {feed.health === "stale" && gotFrame && (
          <span className="chip-warn font-mono">
            frame is {ageWord(feed.ageMs)} old
          </span>
        )}
      </div>

      {!gotFrame ? (
        <div className="mt-2 text-sm text-amber-200/90">
          <b>No frame has arrived{thing ? ` from ${thing}` : ""}</b> — so there is
          nothing to report on. This is <i>not</i> "nothing detected": the camera
          has not been seen looking. The detection count is{" "}
          <span className="font-mono">{DASH}</span> for exactly that reason.
        </div>
      ) : detections.length === 0 ? (
        <div className="mt-2 text-sm text-slate-500">
          Nothing detected in this frame — a real frame arrived and the detector
          returned no boxes.
        </div>
      ) : (
        <ul className="mt-2 space-y-1.5">
          {detections.slice(0, 5).map((x, i) => (
            <li
              key={i}
              className="flex items-center justify-between rounded-md border border-white/5 bg-black/30 px-3 py-1.5 text-sm"
            >
              <span className={x?.kind === "fire" || x?.cls === "fire" ? "chip-hot" : "chip"}>
                {str(x?.label) ?? str(x?.cls) ?? "?"}
              </span>
              <span className="font-mono text-xs text-slate-400">conf {fmt(x?.conf, 2)}</span>
            </li>
          ))}
        </ul>
      )}
    </Card>
  );
}

/* =========================================================== firmware ====== */

/**
 * The board's own account of itself, off `drive:`.
 *
 * It is on this page because it is the context every other card is read in: a
 * yaw rate of zero means something different when `imu_init_ok` is false, and
 * an odometry position means nothing at all when `ros_ok` is false. The Drive
 * tab shows more of this and shows it next to controls; here it is read-only
 * and cut down to the fields that explain the four feeds above.
 */
function FirmwareCard({ feed }: { feed: Feed }) {
  const d = feed.data;
  const rosDown = d?.ros_ok === false;
  const estop = d?.estop_latched === true;

  return (
    <Card>
      <CardHeader
        title="Firmware and link health"
        subtitle="drive: · the context the readings above are true in"
        right={<FeedPill feed={feed} />}
      />

      {feed.messages === 0 ? (
        <Silent feed={feed} what="firmware health" />
      ) : (
        <>
          {rosDown && (
            <div className="mb-3 rounded-lg border-2 border-rose-500/50 bg-rose-950/40 px-3 py-2 text-sm text-rose-100">
              <b>ros_ok is false.</b> The teleop bridge cannot see the board.
              Every odometry and IMU number on this page is at best the last
              thing that got through before the link dropped.
            </div>
          )}
          {estop && (
            <div className="mb-3 rounded-lg border-2 border-rose-500/50 bg-rose-950/40 px-3 py-2 text-sm text-rose-100">
              <b>E-stop is latched.</b> All motor paths are zeroed at the board.
              The rover will not move whatever is commanded until it is cleared.
            </div>
          )}

          <div className="grid grid-cols-2 gap-3 sm:grid-cols-4">
            <Field label="ros_ok" value={flag(d?.ros_ok)} tone={rosDown ? "text-rose-300" : undefined} />
            <Field label="µROS session" value={flag(d?.uros_session)} />
            <Field label="Board deadman" value={flag(d?.deadman_ok)} />
            <Field label="Mode" value={str(d?.mode) ?? DASH} />

            <Field label="odom rate" value={fmt(d?.odom_hz, 1, " Hz")} />
            <Field
              label="imu rate"
              value={fmt(d?.imu_hz, 1, " Hz")}
              hint="a rate is not a reading"
            />
            <Field label="control loop" value={fmt(d?.control_hz, 1, " Hz")} />
            <Field label="battery rate" value={fmt(d?.batt_hz, 1, " Hz")} />

            <Field label="Firmware" value={str(d?.fw_version_str) ?? DASH} />
            <Field label="Active path" value={str(d?.active_path_name) ?? DASH} />
            <Field label="Battery" value={fmt(d?.battery_v, 2, " V")} />
            <Field label="Board uptime" value={fmt(d?.board_uptime_s, 0, " s")} />
          </div>

          {/* The bitfield, expanded. Named flags are shown even when the
              bitfield itself is absent, because a firmware that publishes the
              booleans without the raw word is a shape this has to survive. */}
          <div className="mt-3 flex flex-wrap gap-2">
            {[
              ["estop latched", d?.estop_latched],
              ["vel armed", d?.vel_armed],
              ["deadman firing", d?.board_deadman_firing],
              ["imu init ok", d?.imu_init_ok],
              ["agent connected", d?.agent_connected],
              ["cmd_vel compiled", d?.cmd_vel_compiled],
              ["servos compiled", d?.servos_compiled],
            ].map(([label, v]) => (
              <span
                key={String(label)}
                className={
                  v === undefined
                    ? "chip font-mono text-slate-600"
                    : v === true
                      ? "chip-ok font-mono"
                      : "chip font-mono"
                }
                title={
                  v === undefined
                    ? "This flag is not in the payload. Absent is not false."
                    : undefined
                }
              >
                {String(label)} · {flag(v)}
              </span>
            ))}
          </div>

          {str(d?.last_error) && (
            <div className="mt-3 rounded-lg border border-amber-500/30 bg-amber-500/5 px-3 py-2 font-mono text-xs text-amber-100/90">
              last_error · {str(d?.last_error)}
            </div>
          )}

          <p className="mt-3 text-xs leading-relaxed text-slate-500">
            A publish RATE is not a reading. <span className="font-mono">imu_hz</span>{" "}
            counts messages leaving the board; it stays healthy while the yaw
            rate inside those messages is a deadbanded zero, which is precisely
            why the IMU card above looks at the values and not at the rate.
          </p>
        </>
      )}
    </Card>
  );
}

/* ============================================================== board ====== */

/**
 * THE BOARD FEED — the only place raw hardware appears on this dashboard.
 *
 * `board:<thing>` is published at 5 Hz by rover/fpms_stm32_bridge.py straight
 * off the Rosmaster serial link: the IMU, all four wheel counters, the pack
 * voltage and the bridge's own health, in one consolidated sample. Everything
 * else on this page has been through ROS and arrives already integrated or
 * already filtered. This is the closest this dashboard gets to reading the
 * board with a meter, and it is what makes "IMU, encoders, voltage, health"
 * answerable without SSH-ing to the Pi.
 *
 * ---------------------------------------------------------------------------
 * READ `fresh` AND `link_dead` BEFORE ANY NUMBER UNDER THEM
 * ---------------------------------------------------------------------------
 * That is the publisher's own instruction and it is not a general caution. The
 * Rosmaster receive thread can exit silently, and every getter on that side is
 * a cached-field read — so when it dies the values stay perfectly plausible
 * FOREVER, at the full 5 Hz, and nothing about the shape of the feed changes.
 * A frozen board is indistinguishable from a parked one by inspection.
 *
 * So the publisher ships `stale_reason` the moment either flag turns, and this
 * card paints it as a banner ABOVE the readouts rather than as a note beside
 * them. It never blanks the numbers: the bridge deliberately keeps sending
 * them ("a blank panel is indistinguishable from a parked rover") and the
 * operator's question in that state is "what was the last thing it saw".
 *
 * ---------------------------------------------------------------------------
 * WHY THERE IS NO LOW-VOLTAGE WARNING
 * ---------------------------------------------------------------------------
 * Nobody has established this pack's cutoff. A threshold typed in here would
 * be an invented number on the one reading an operator would plan a run
 * around, which is the first rule at the top of this file. The voltage is
 * shown large and honestly, with its own age, and it is left to the operator
 * to know their own battery.
 */
function BoardCard({ feed }: { feed: Feed }) {
  const d = feed.data;

  const imu = (d?.imu ?? null) as Record<string, unknown> | null;
  const units = (imu?.units ?? null) as Record<string, unknown> | null;
  const accel = (imu?.accel ?? null) as Record<string, unknown> | null;
  const gyro = (imu?.gyro ?? null) as Record<string, unknown> | null;
  const enc = (d?.encoders ?? null) as Record<string, unknown> | null;
  const health = (d?.health ?? null) as Record<string, unknown> | null;
  const rates = (health?.rates_hz ?? null) as Record<string, unknown> | null;
  const targets = (health?.target_rates_hz ?? null) as Record<string, unknown> | null;

  const ticks = Array.isArray(enc?.ticks) ? (enc?.ticks as unknown[]) : null;
  const mm = Array.isArray(enc?.mm) ? (enc?.mm as unknown[]) : null;
  const order = Array.isArray(enc?.order) ? (enc?.order as unknown[]) : null;

  const linkDead = d?.link_dead === true;
  const notFresh = d?.fresh === false;
  const reason = str(d?.stale_reason);
  const estop = health?.estop === true;
  const armed = health?.armed === true;

  /** Seconds (as the payload sends them) to the millisecond ageWord() wants. */
  const ageMsOf = (v: unknown): number | null => {
    const s = num(v);
    return s === null ? null : s * 1000;
  };

  /**
   * A measured rate of exactly zero against a target that is not zero.
   *
   * This comparison is one the publisher set up on purpose — it sends
   * `rates_hz` and `target_rates_hz` side by side — and zero is the one
   * verdict needing no invented threshold: that sub-feed has stopped. A merely
   * SLOW rate is left uncoloured, because "how slow is broken" is a number
   * nobody here has measured.
   */
  const rateDead = (k: string): boolean => {
    const got = num(rates?.[k]);
    const want = num(targets?.[k]);
    return got === 0 && want !== null && want > 0;
  };

  const rateRow = (k: string, label: string) => (
    <Field
      key={k}
      label={label}
      value={fmt(rates?.[k], 2, " Hz")}
      hint={`target ${fmt(targets?.[k], 1, " Hz")}`}
      tone={rateDead(k) ? "text-rose-300" : undefined}
      title="Measured by the bridge over a 4 s window. A rate is not a reading: it counts messages, not sane values."
    />
  );

  return (
    <Card>
      <CardHeader
        title="Board — IMU, encoders, voltage, health"
        subtitle="board: · read straight off the Rosmaster serial link at 5 Hz"
        right={<FeedPill feed={feed} />}
      />

      {feed.messages === 0 ? (
        <Silent feed={feed} what="board telemetry" />
      ) : (
        <>
          {/* The publisher's own verdict on its own numbers, first and loudest.
              `stale_reason` is only ever present when something is wrong. */}
          {reason && (
            <div
              className={`mb-3 rounded-lg px-3 py-2 text-sm ${
                linkDead
                  ? "border-2 border-rose-500/60 bg-rose-950/40 text-rose-100"
                  : "border border-amber-500/40 bg-amber-500/5 text-amber-100/90"
              }`}
            >
              <b>{linkDead ? "BOARD LINK DEAD." : "Board sample is stale."}</b>{" "}
              {reason}
            </div>
          )}
          {estop && (
            <div className="mb-3 rounded-lg border-2 border-rose-500/60 bg-rose-950/40 px-3 py-2 text-sm text-rose-100">
              <b>E-stop is set at the board.</b> Motor paths are zeroed in
              firmware. Nothing commanded will move the rover until it clears.
            </div>
          )}

          {/* ---- the headline four: voltage, age, and the two trust flags --- */}
          <div className="grid grid-cols-2 gap-3 sm:grid-cols-4">
            <Field
              label="Pack voltage"
              value={fmt(d?.voltage, 2, " V")}
              hint={`read ${ageWord(ageMsOf(d?.voltage_age_s))} ago`}
              big
              title="Raw pack voltage as the board reports it. No warning threshold is applied — see the note on this card."
            />
            <Field
              label="Sample age"
              value={fmt(d?.data_age_s, 3, " s")}
              hint="freshest of imu / enc / volt"
              tone={notFresh || linkDead ? "text-rose-300" : undefined}
            />
            <Field
              label="fresh"
              value={flag(d?.fresh)}
              tone={notFresh ? "text-rose-300" : undefined}
              title="The publisher's own flag: NOT link_dead AND a board sample newer than 3 s. Read it before any number on this card."
            />
            <Field
              label="link dead"
              value={flag(d?.link_dead)}
              tone={linkDead ? "text-rose-300" : undefined}
              title="The Rosmaster receive thread has exited. Every value here is frozen at its last-read figure."
            />
          </div>

          {/* ---- IMU ---- */}
          <div className="lbl mt-5 text-[10px]">
            IMU · {str(units?.accel) ?? "m/s²"} / {str(units?.gyro) ?? "rad/s"} ·
            read {ageWord(ageMsOf(imu?.age_s))} ago
          </div>
          <div className="mt-2 grid grid-cols-3 gap-3 sm:grid-cols-6">
            <Field label="a x" value={fmt(accel?.x, 3)} />
            <Field label="a y" value={fmt(accel?.y, 3)} />
            <Field
              label="a z"
              value={fmt(accel?.z, 3)}
              hint="≈ -9.81 parked"
              title="Gravity on z is the cheapest proof the accelerometer is alive: a level board reading 0.000 here is not a still rover, it is a dead sensor."
            />
            <Field label="ω x" value={fmt(gyro?.x, 4)} />
            <Field label="ω y" value={fmt(gyro?.y, 4)} />
            <Field
              label="ω z"
              value={fmt(gyro?.z, 4)}
              hint={num(gyro?.z) === 0 ? "exactly zero" : undefined}
              title="A HEALTHY gyro on this rover still reads exactly 0.0000 while parked — the firmware applies a ±0.01 rad/s deadband. Zero here is not evidence of a fault on its own."
            />
          </div>

          {/* ---- encoders ---- */}
          <div className="lbl mt-5 text-[10px]">
            Wheel counters · {fmt(enc?.counts_per_mm, 2, " counts/mm")} · read{" "}
            {ageWord(ageMsOf(enc?.age_s))} ago
          </div>
          {ticks === null ? (
            <div className="mt-2 rounded-lg border border-white/10 bg-black/30 px-3 py-2 text-sm text-slate-400">
              The board has not returned a counter sample yet. This is not four
              zeroes — nothing has been read at all.
            </div>
          ) : (
            <div className="mt-2 grid grid-cols-2 gap-3 sm:grid-cols-4">
              {ticks.map((t, i) => (
                <Field
                  key={i}
                  label={str(order?.[i]) ?? `wheel ${i + 1}`}
                  value={fmt(t, 0, " ticks")}
                  hint={mm ? `${fmt(mm[i], 2)} mm` : undefined}
                />
              ))}
            </div>
          )}
          <p className="mt-2 text-xs leading-relaxed text-slate-500">
            Counts become millimetres at{" "}
            <span className="font-mono">{fmt(enc?.mm_per_tick, 5)}</span> mm/tick
            — 6.00 counts/mm, the figure the program that actually completed M1
            and M2 used, and which matches the 6.12 measured on this chassis.{" "}
            <b className="text-slate-400">It is not 14.8.</b> That value is 2.7x
            wrong and is the single cause behind this project's history of
            distance overshoot. Publishing the raw counts beside the millimetres
            is what stops an error like that hiding again.
          </p>

          {/* ---- bridge + firmware health ---- */}
          <div className="lbl mt-5 text-[10px]">Board and bridge health</div>
          <div className="mt-2 grid grid-cols-2 gap-3 sm:grid-cols-4">
            <Field label="Firmware" value={fmt(health?.fw, 1)} />
            <Field label="Serial port" value={str(health?.port) ?? DASH} />
            <Field label="Bridge uptime" value={fmt(health?.uptime_s, 0, " s")} />
            <Field
              label="MQTT errors"
              value={fmt(health?.mqtt_errors, 0)}
              hint={`${fmt(health?.mqtt_published, 0)} published`}
              tone={(num(health?.mqtt_errors) ?? 0) > 0 ? "text-amber-300" : undefined}
            />
            {rateRow("odom", "odom rate")}
            {rateRow("imu", "imu rate")}
            {rateRow("ticks", "ticks rate")}
            {rateRow("mqtt", "publish rate")}
          </div>

          <div className="mt-3 flex flex-wrap gap-2">
            {[
              ["armed", health?.armed],
              ["estop", health?.estop],
              ["link dead", health?.link_dead],
            ].map(([label, v]) => (
              <span
                key={String(label)}
                className={
                  v === undefined
                    ? "chip font-mono text-slate-600"
                    : v === true
                      ? "chip-warn font-mono"
                      : "chip font-mono"
                }
                title={
                  v === undefined
                    ? "This flag is not in the payload. Absent is not false."
                    : undefined
                }
              >
                {String(label)} · {flag(v)}
              </span>
            ))}
            {armed && (
              <span
                className="chip-warn font-mono"
                title="The board will act on velocity commands."
              >
                motors live
              </span>
            )}
          </div>

          <p className="mt-3 text-xs leading-relaxed text-slate-500">
            This feed is published ON A TIMER, not on change — a parked rover
            produces identical samples for minutes, and a feed that only speaks
            when something moves cannot be told apart from a dead one. So the
            pill above going amber always means the PUBLISHER stopped; it never
            means the rover is merely sitting still.
          </p>
        </>
      )}
    </Card>
  );
}
