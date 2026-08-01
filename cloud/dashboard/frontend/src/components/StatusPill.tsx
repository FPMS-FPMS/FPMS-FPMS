/**
 * Per-stream connection state, used in card headers across Camera, Thermal,
 * LiDAR, Drive and Control.
 *
 * "No signal" is a warning, not a cosmetic state — it is how an operator finds
 * out that the picture they are looking at stopped updating. It stays amber,
 * keeps its pulsing dot, and is never rendered as merely "idle".
 */
export function StatusPill({
  connected,
  lastAt,
  messages,
}: {
  connected: boolean;
  lastAt: number | null;
  messages: number;
}) {
  // Unchanged: stale is evaluated at render, on the same 3s threshold.
  const stale = !connected || (lastAt !== null && Date.now() - lastAt > 3000);
  const cls = stale ? "chip-warn" : "chip-ok";
  const label = stale ? "no signal" : "live";

  const title = stale
    ? lastAt
      ? `No frames for over 3s — last at ${new Date(lastAt).toLocaleTimeString()}`
      : "Nothing has arrived on this channel yet"
    : lastAt
      ? `Live · last frame ${new Date(lastAt).toLocaleTimeString()}`
      : "Live";

  return (
    <span className={cls} title={title} role="status">
      <span
        className={`inline-block h-2 w-2 shrink-0 rounded-full pulse-dot ${
          stale ? "bg-amber-300 text-amber-300" : "bg-emerald-300 text-emerald-300"
        }`}
      />
      <span className={stale ? "font-semibold uppercase tracking-wide" : ""}>{label}</span>
      <span className="font-mono text-2xs opacity-70">· {messages} pkts</span>
    </span>
  );
}
