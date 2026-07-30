export function StatusPill({
  connected,
  lastAt,
  messages,
}: {
  connected: boolean;
  lastAt: number | null;
  messages: number;
}) {
  const stale = !connected || (lastAt !== null && Date.now() - lastAt > 3000);
  const cls = stale ? "chip-warn" : "chip-ok";
  const label = stale ? "no signal" : "live";
  return (
    <span className={cls} title={lastAt ? `last: ${new Date(lastAt).toLocaleTimeString()}` : ""}>
      <span
        className={`inline-block h-2 w-2 rounded-full pulse-dot ${
          stale ? "bg-amber-400 text-amber-400" : "bg-emerald-400 text-emerald-400"
        }`}
      />
      {label}
      <span className="text-[10px] opacity-70">· {messages} pkts</span>
    </span>
  );
}
