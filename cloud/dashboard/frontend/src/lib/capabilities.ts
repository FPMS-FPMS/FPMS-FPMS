import { useEffect, useRef, useState } from "react";
import { useChannel } from "./ws";

/**
 * WHAT THE ROVER SAYS IT CAN DO — learned from the rover, not hardcoded.
 *
 * THE PROBLEM THIS SOLVES. The command set lives in four places that can drift
 * apart independently:
 *
 *   fpms_teleop.py      TELEOP_ACTIONS   the motion/actuator verbs, on ROS
 *   fpms_missions.py    "mission"        the mission executor
 *   fpms_rover_agent.py handle_command   ping/status/connect/disconnect/restart
 *   backend/main.py     CONTROL_ACTIONS  the allowlist the HTTP API will publish
 *   Control.tsx                          the buttons an operator can actually press
 *
 * A verb can be added to the rover and never reach a button; a button can
 * survive a verb being renamed and simply time out forever. Both have happened.
 *
 * The rovers already announce the truth: fpms-teleop publishes its full
 * `TELEOP_ACTIONS` on `events/online` and again in every `drive_status` reply,
 * with the explicit comment that "the dashboard should build its button set
 * from this rather than from a hardcoded list that can drift". fpms-missions
 * announces its own verb, its mission names and its backends. This module
 * listens for those announcements and accumulates them per rover.
 *
 * TWO RULES, both about not lying:
 *
 *   1. UNKNOWN IS NOT UNSUPPORTED. `events/online` is published once, at
 *      connect, and is NOT retained (see Bus.publish in fpms_teleop.py — no
 *      retain flag). A dashboard opened after the rover booted will therefore
 *      have heard nothing, and that must never be rendered as "this rover
 *      supports no commands". `announced` says whether we have heard anything
 *      at all, and every consumer is required to branch on it. Buttons are
 *      never disabled on the strength of a list we do not have.
 *
 *   2. DRIFT IS REPORTED IN BOTH DIRECTIONS. A verb the rover advertises that
 *      the dashboard has no control for is a missing feature. A control the
 *      dashboard offers that the rover never advertised is a button that will
 *      time out. Neither is discoverable by pressing things one at a time.
 *
 * The refresh path is `drive_status`, which is a pure read — it commands no
 * motion — so an operator can re-ask at any time without touching the rover.
 */

export type ServiceAnnouncement = {
  /** "teleop", "missions", or whatever the service called itself. */
  svc: string;
  /** Verbs this service says it subscribes and answers. */
  actions: string[];
  /** Where each verb's reply lands, from teleop's reply_topics. */
  replyTopics: Record<string, string>;
  /** Verbs it acts on but deliberately does not answer (missions: stop/estop). */
  actsSilentlyOn: string[];
  /** Non-command capabilities, e.g. rover-agent's camera/lidar/yolo. */
  capabilities: string[];
  /** The envelope subtype it came from — "online" or "drive_status". */
  via: string;
  at: number;
};

export type RoverCapabilities = {
  /** True once ANY service on this rover has announced a verb list. */
  announced: boolean;
  /** Union of every advertised verb, across services. */
  actions: ReadonlySet<string>;
  /** verb -> the service that claims it. */
  owners: Readonly<Record<string, string>>;
  /** verb -> reply topic, where the rover told us. */
  replyTopics: Readonly<Record<string, string>>;
  /**
   * verb -> why nothing on the rover owns it. Straight from teleop's
   * `not_owned_here` in its drive_status snapshot; the rover is the authority
   * on which of its own commands land nowhere.
   */
  notOwned: Readonly<Record<string, string>>;
  services: readonly ServiceAnnouncement[];
  /** Mission names the executor offers, if it has announced. */
  missions: string[] | null;
  backends: string[] | null;
  defaultBackend: string | null;
  /** Numeric envelope, merged across announcements. Display only. */
  limits: Readonly<Record<string, unknown>>;
  lastAt: number | null;
};

const EMPTY: RoverCapabilities = {
  announced: false,
  actions: new Set<string>(),
  owners: {},
  replyTopics: {},
  notOwned: {},
  services: [],
  missions: null,
  backends: null,
  defaultBackend: null,
  limits: {},
  lastAt: null,
};

export function emptyCapabilities(): RoverCapabilities {
  return EMPTY;
}

function strList(v: unknown): string[] {
  return Array.isArray(v) ? v.filter((x): x is string => typeof x === "string") : [];
}

function strMap(v: unknown): Record<string, string> {
  if (!v || typeof v !== "object" || Array.isArray(v)) return {};
  const out: Record<string, string> = {};
  for (const [k, val] of Object.entries(v as Record<string, unknown>)) {
    if (typeof val === "string") out[k] = val;
  }
  return out;
}

/**
 * Fold one announcement into a rover's capability record.
 *
 * ADDITIVE, never subtractive. A `drive_status` from teleop describes teleop
 * only; treating it as the whole truth would delete the mission executor's verb
 * from the set every time an operator pressed refresh. Services are replaced
 * individually, keyed by name, and the union is recomputed — so a service that
 * genuinely drops a verb loses it on its next announcement, while a service
 * that has simply not spoken keeps what it last said.
 */
function fold(prev: RoverCapabilities, ann: ServiceAnnouncement): RoverCapabilities {
  const services = [...prev.services.filter((s) => s.svc !== ann.svc), ann].sort((a, b) =>
    a.svc.localeCompare(b.svc),
  );

  const actions = new Set<string>();
  const owners: Record<string, string> = {};
  const replyTopics: Record<string, string> = { ...prev.replyTopics };
  for (const s of services) {
    for (const a of s.actions) {
      actions.add(a);
      // First announcer wins the "owner" label, and services are sorted by
      // name, so the attribution is stable rather than depending on which
      // service happened to reconnect most recently.
      if (!(a in owners)) owners[a] = s.svc;
    }
    Object.assign(replyTopics, s.replyTopics);
  }

  return {
    announced: true,
    actions,
    owners,
    replyTopics,
    notOwned: prev.notOwned,
    services,
    missions: prev.missions,
    backends: prev.backends,
    defaultBackend: prev.defaultBackend,
    limits: prev.limits,
    lastAt: ann.at,
  };
}

/**
 * Accumulate capability announcements per rover off the shared `events` stream.
 *
 * `useChannel` keeps only the newest envelope, so — exactly as Control.tsx and
 * Drive.tsx already do for their command logs — the message COUNTER is what
 * says a new envelope landed, and the effect reads `.data` when it changes.
 *
 * Replayed announcements are welcome here, unlike in the command logs. The hub
 * re-broadcasts its last message to every new subscriber, and an `online` from
 * before this page loaded is still a true statement about what the rover
 * supports. A capability list does not go stale the way an ack does.
 */
export function useRoverCapabilities(): Record<string, RoverCapabilities> {
  const ev = useChannel<any>("events");
  const [byThing, setByThing] = useState<Record<string, RoverCapabilities>>({});
  const seenRef = useRef(0);

  useEffect(() => {
    if (ev.messages === seenRef.current) return;
    seenRef.current = ev.messages;

    const env = ev.data;
    if (!env || typeof env !== "object") return;
    const thing = typeof env.thing === "string" ? env.thing : null;
    const subtype = typeof env.subtype === "string" ? env.subtype : "";
    const data = env.data && typeof env.data === "object" ? (env.data as Record<string, any>) : null;
    if (!thing || !data) return;
    if (subtype !== "online" && subtype !== "drive_status") return;

    const actions = strList(data.actions).concat(strList(data.owns));
    const capabilities = strList(data.capabilities);
    // rover-agent's `online` carries capabilities but no verb list. It is worth
    // recording (it is how the operator learns the camera/LiDAR side is up) but
    // it must NOT set `announced` on its own, or the command drift report would
    // claim every teleop verb is unsupported on the strength of a message that
    // never mentioned commands.
    if (actions.length === 0 && capabilities.length === 0) return;

    const ann: ServiceAnnouncement = {
      svc: typeof data.svc === "string" ? data.svc : "rover-agent",
      actions: Array.from(new Set(actions)).sort(),
      replyTopics: strMap(data.reply_topics),
      actsSilentlyOn: strList(data.acts_silently_on),
      capabilities,
      via: subtype,
      at: Date.now(),
    };

    const notOwned = strMap(data.not_owned_here);
    const missions = strList(data.missions);
    const backends = strList(data.backends);
    const defaultBackend =
      typeof data.default_backend === "string" ? data.default_backend : null;
    const limits =
      data.limits && typeof data.limits === "object" && !Array.isArray(data.limits)
        ? (data.limits as Record<string, unknown>)
        : null;

    setByThing((prevAll) => {
      const prev = prevAll[thing] ?? EMPTY;
      let next = actions.length ? fold(prev, ann) : { ...prev, services: prev.services };
      if (actions.length === 0) {
        // Capability-only announcement: record the service, leave the verb
        // union and `announced` untouched.
        next = {
          ...prev,
          services: [...prev.services.filter((s) => s.svc !== ann.svc), ann].sort((a, b) =>
            a.svc.localeCompare(b.svc),
          ),
          lastAt: ann.at,
        };
      }
      return {
        ...prevAll,
        [thing]: {
          ...next,
          notOwned: Object.keys(notOwned).length ? { ...next.notOwned, ...notOwned } : next.notOwned,
          missions: missions.length ? missions : next.missions,
          backends: backends.length ? backends : next.backends,
          defaultBackend: defaultBackend ?? next.defaultBackend,
          limits: limits ? { ...next.limits, ...limits } : next.limits,
        },
      };
    });
  }, [ev.messages, ev.data]);

  return byThing;
}

/**
 * Merge what several rovers have announced into one view.
 *
 * The Control tab can target BOTH bays at once, and in that mode the honest
 * answer to "is this verb supported" is the UNION: a button that works on
 * rover1 must not be greyed out because rover2 has never spoken. Refusals are
 * per-rover and land in the log with the rover's name on them, which is the
 * right place for a disagreement between two machines to surface.
 */
export function mergeCapabilities(
  all: Record<string, RoverCapabilities>,
  things: readonly string[],
): RoverCapabilities {
  const picked = things.map((t) => all[t]).filter((c): c is RoverCapabilities => !!c);
  if (picked.length === 0) return EMPTY;
  if (picked.length === 1) return picked[0];

  const actions = new Set<string>();
  const owners: Record<string, string> = {};
  const replyTopics: Record<string, string> = {};
  const notOwned: Record<string, string> = {};
  const limits: Record<string, unknown> = {};
  const services: ServiceAnnouncement[] = [];
  let announced = false;
  let missions: string[] | null = null;
  let backends: string[] | null = null;
  let defaultBackend: string | null = null;
  let lastAt: number | null = null;

  for (const c of picked) {
    announced = announced || c.announced;
    c.actions.forEach((a) => actions.add(a));
    Object.assign(owners, c.owners);
    Object.assign(replyTopics, c.replyTopics);
    Object.assign(notOwned, c.notOwned);
    Object.assign(limits, c.limits);
    services.push(...c.services);
    missions = missions ?? c.missions;
    backends = backends ?? c.backends;
    defaultBackend = defaultBackend ?? c.defaultBackend;
    if (c.lastAt !== null) lastAt = lastAt === null ? c.lastAt : Math.max(lastAt, c.lastAt);
  }

  return { announced, actions, owners, replyTopics, notOwned, services, missions, backends, defaultBackend, limits, lastAt };
}
