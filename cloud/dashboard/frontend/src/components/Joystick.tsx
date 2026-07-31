import { useEffect, useRef, useState } from "react";

/**
 * Analog stick for manual driving.
 *
 * Three properties matter more than the looks:
 *
 *  1. It emits on a fixed clock, not on pointer events. The rover has a deadman
 *     timer — if jog messages stop arriving it halts — so a stick held perfectly
 *     still must keep producing messages. A pointermove-driven emitter goes
 *     silent exactly when the operator holds a steady heading.
 *  2. It has a dead zone. A thumb resting on a tablet is never exactly centred,
 *     and a few percent of creep on a machine that weighs something is a fault,
 *     not a quirk.
 *  3. Release is authoritative: the knob springs home and {0,0} goes out in the
 *     same tick, before the deadman has to do it for us.
 */

/** Normalised stick output. +vx is forward (stick up), +wz is right. */
export type StickValue = { vx: number; wz: number };

type Props = {
  /** Called at ~`hz` while held, and once with {0,0} the instant it is released. */
  onChange: (v: StickValue) => void;
  /** Fired when the stick is grabbed and when it is let go. */
  onActiveChange?: (active: boolean) => void;
  /** Pad diameter in px. */
  size?: number;
  /** Fraction of travel (0..1) treated as centre. */
  deadZone?: number;
  /** Emit rate while held. */
  hz?: number;
  disabled?: boolean;
  /** Shown across the pad when disabled. */
  disabledHint?: string;
};

const clamp1 = (n: number) => (n > 1 ? 1 : n < -1 ? -1 : n);

export default function Joystick({
  onChange,
  onActiveChange,
  size = 220,
  deadZone = 0.08,
  hz = 10,
  disabled = false,
  disabledHint = "unavailable",
}: Props) {
  const padRef = useRef<HTMLDivElement | null>(null);
  /** Raw stick position inside the unit circle, y positive upwards. */
  const rawRef = useRef<{ x: number; y: number }>({ x: 0, y: 0 });
  const pointerIdRef = useRef<number | null>(null);
  const [knob, setKnob] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [active, setActive] = useState(false);

  // Callbacks live in refs so the emit interval is never torn down and rebuilt
  // by a parent re-render — a restart mid-hold would put a gap in the stream
  // the deadman could notice.
  const changeRef = useRef(onChange);
  changeRef.current = onChange;
  const activeRef = useRef(onActiveChange);
  activeRef.current = onActiveChange;

  /** Apply the dead zone, then rescale so travel past it starts from zero. */
  const shape = (raw: { x: number; y: number }): StickValue => {
    const mag = Math.hypot(raw.x, raw.y);
    if (!Number.isFinite(mag) || mag <= deadZone) return { vx: 0, wz: 0 };
    const k = Math.min(1, (mag - deadZone) / (1 - deadZone)) / mag;
    return { vx: clamp1(raw.y * k), wz: clamp1(raw.x * k) };
  };

  // The heartbeat. Sends once immediately on grab so the first movement is not
  // delayed by a whole period, then keeps repeating the current position —
  // including an unchanged one — for as long as the stick is held.
  useEffect(() => {
    if (!active) return;
    const period = Math.max(20, Math.round(1000 / (hz > 0 ? hz : 10)));
    changeRef.current(shape(rawRef.current));
    const id = window.setInterval(
      () => changeRef.current(shape(rawRef.current)),
      period,
    );
    return () => window.clearInterval(id);
  }, [active, hz, deadZone]); // eslint-disable-line react-hooks/exhaustive-deps

  // Navigating away mid-hold must not leave the rover coasting on the deadman.
  useEffect(
    () => () => {
      if (pointerIdRef.current !== null) changeRef.current({ vx: 0, wz: 0 });
    },
    [],
  );

  const track = (e: { clientX: number; clientY: number }) => {
    const el = padRef.current;
    if (!el) return;
    const r = el.getBoundingClientRect();
    const radius = Math.min(r.width, r.height) / 2;
    if (!(radius > 0)) return;
    let x = (e.clientX - (r.left + r.width / 2)) / radius;
    // Screen y grows downwards; the operator's "up" has to mean forward.
    let y = ((r.top + r.height / 2) - e.clientY) / radius;
    const mag = Math.hypot(x, y);
    if (mag > 1) {
      x /= mag;
      y /= mag;
    }
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    rawRef.current = { x, y };
    setKnob({ x, y });
  };

  const grab = (e: React.PointerEvent<HTMLDivElement>) => {
    if (disabled) return;
    e.preventDefault();
    pointerIdRef.current = e.pointerId;
    // Capture keeps the drag alive when the thumb slides off the pad, which on
    // a tablet is most drags.
    try {
      e.currentTarget.setPointerCapture(e.pointerId);
    } catch {
      /* capture is a nicety, not a requirement */
    }
    track(e);
    setActive(true);
    activeRef.current?.(true);
  };

  const move = (e: React.PointerEvent<HTMLDivElement>) => {
    if (pointerIdRef.current !== e.pointerId) return;
    e.preventDefault();
    track(e);
  };

  /** Idempotent — pointerup and the lostpointercapture it triggers both land here. */
  const release = (e: React.PointerEvent<HTMLDivElement>) => {
    if (pointerIdRef.current === null || pointerIdRef.current !== e.pointerId) return;
    pointerIdRef.current = null;
    try {
      if (e.currentTarget.hasPointerCapture(e.pointerId)) {
        e.currentTarget.releasePointerCapture(e.pointerId);
      }
    } catch {
      /* already gone */
    }
    rawRef.current = { x: 0, y: 0 };
    setKnob({ x: 0, y: 0 });
    setActive(false);
    activeRef.current?.(false);
    // Explicit zero now, rather than waiting for the deadman to infer one.
    changeRef.current({ vx: 0, wz: 0 });
  };

  const out = shape(knob);
  const knobPx = Math.round(size * 0.28);
  const travel = size / 2 - knobPx / 2 - 4;
  const live = Math.hypot(out.vx, out.wz) > 0;

  return (
    <div className="flex flex-col items-center gap-3">
      <div
        ref={padRef}
        onPointerDown={grab}
        onPointerMove={move}
        onPointerUp={release}
        onPointerCancel={release}
        onLostPointerCapture={release}
        style={{ width: size, height: size, touchAction: "none" }}
        className={`relative select-none rounded-full border transition-colors ${
          disabled
            ? "cursor-not-allowed border-white/5 bg-black/40"
            : active
              ? "cursor-grabbing border-ember-500/40 bg-ember-500/[0.06] shadow-glow"
              : "cursor-grab border-white/10 bg-black/40"
        }`}
        role="application"
        aria-label="Drive joystick"
        title={disabled ? disabledHint : "Drag to drive. Release to stop."}
      >
        {/* crosshair + outer ring */}
        <div className="pointer-events-none absolute inset-0 rounded-full">
          <div className="absolute left-1/2 top-0 h-full w-px -translate-x-1/2 bg-white/5" />
          <div className="absolute left-0 top-1/2 h-px w-full -translate-y-1/2 bg-white/5" />
          <div className="absolute inset-[14%] rounded-full border border-dashed border-white/5" />
        </div>

        {/* dead zone — drawn so the operator can see why small movements do nothing */}
        <div
          className="pointer-events-none absolute left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 rounded-full border border-amber-400/25 bg-amber-400/[0.04]"
          style={{ width: size * deadZone * 2, height: size * deadZone * 2 }}
          title={`dead zone ${Math.round(deadZone * 100)}%`}
        />

        {/* axis hints */}
        <span className="pointer-events-none absolute left-1/2 top-2 -translate-x-1/2 font-mono text-[10px] text-slate-600">
          FWD
        </span>
        <span className="pointer-events-none absolute bottom-2 left-1/2 -translate-x-1/2 font-mono text-[10px] text-slate-600">
          REV
        </span>
        <span className="pointer-events-none absolute right-2 top-1/2 -translate-y-1/2 font-mono text-[10px] text-slate-600">
          R
        </span>
        <span className="pointer-events-none absolute left-2 top-1/2 -translate-y-1/2 font-mono text-[10px] text-slate-600">
          L
        </span>

        {/* knob */}
        <div
          className={`pointer-events-none absolute left-1/2 top-1/2 rounded-full border transition-transform ${
            active ? "duration-0" : "duration-150"
          } ${
            live
              ? "border-ember-400/60 bg-ember-500/30 shadow-glow"
              : "border-white/15 bg-white/10"
          }`}
          style={{
            width: knobPx,
            height: knobPx,
            transform: `translate(-50%, -50%) translate(${knob.x * travel}px, ${-knob.y * travel}px)`,
          }}
        />

        {disabled && (
          <div className="absolute inset-0 flex items-center justify-center rounded-full bg-black/50 px-6 text-center text-xs text-slate-500">
            {disabledHint}
          </div>
        )}
      </div>

      <div className="flex items-center gap-2 font-mono text-[11px] text-slate-400">
        <span className={live ? "chip-hot" : "chip"}>{live ? "driving" : "centred"}</span>
        <span>vx {out.vx.toFixed(2)}</span>
        <span>wz {out.wz.toFixed(2)}</span>
      </div>
    </div>
  );
}
