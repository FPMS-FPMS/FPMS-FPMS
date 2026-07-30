type Analysis = {
  data: {
    stats: { min_c: number; max_c: number; mean_c: number; std_c: number };
    hotspots: {
      id: number;
      pixels: number;
      centroid: [number, number];
      peak_c: number;
      mean_c: number;
    }[];
    trend: { direction: string; delta_max_c: number; window_s: number };
    severity: "nominal" | "elevated" | "critical";
    report: string;
    thresholds: { hot_c: number; critical_c: number };
  };
};

export function AnalystPanel({ analysis }: { analysis: Analysis | null }) {
  if (!analysis) {
    return (
      <div className="card p-5">
        <div className="lbl">Analyst</div>
        <div className="mt-3 text-sm text-slate-500">
          Waiting for thermal frames to analyze…
        </div>
      </div>
    );
  }
  const d = analysis.data;
  const sev = d.severity;
  const sevChip =
    sev === "critical"
      ? "chip-hot"
      : sev === "elevated"
      ? "chip-warn"
      : "chip-ok";

  const trendArrow =
    d.trend.direction === "rising" ? "↑" : d.trend.direction === "falling" ? "↓" : "→";

  return (
    <div className="card-glow p-5">
      <div className="flex items-start justify-between">
        <div>
          <div className="lbl">Mini analyst · onboard</div>
          <div className="mt-1 text-lg font-semibold text-slate-100">
            Scene assessment
          </div>
        </div>
        <span className={sevChip}>{sev.toUpperCase()}</span>
      </div>
      <p className="mt-4 text-sm leading-relaxed text-slate-200">{d.report}</p>

      <div className="mt-5 grid grid-cols-2 gap-3 sm:grid-cols-4">
        <Stat label="Peak" value={`${d.stats.max_c.toFixed(1)}°C`} accent={sev === "critical"} />
        <Stat label="Mean" value={`${d.stats.mean_c.toFixed(1)}°C`} />
        <Stat label="σ" value={`${d.stats.std_c.toFixed(1)}°C`} />
        <Stat label={`Trend (${d.trend.window_s}s)`} value={`${trendArrow} ${d.trend.delta_max_c > 0 ? "+" : ""}${d.trend.delta_max_c}°C`} />
      </div>

      <div className="mt-5">
        <div className="lbl mb-2">Hotspots ({d.hotspots.length})</div>
        {d.hotspots.length === 0 ? (
          <div className="text-sm text-slate-500">None above {d.thresholds.hot_c}°C.</div>
        ) : (
          <ul className="space-y-1.5 text-sm">
            {d.hotspots.map((h) => (
              <li
                key={h.id}
                className="flex items-center justify-between rounded-md border border-white/5 bg-black/30 px-3 py-2"
              >
                <span className="font-mono text-xs text-slate-400">
                  #{h.id} · ({h.centroid[0]}, {h.centroid[1]}) · {h.pixels}px
                </span>
                <span className="font-semibold text-ember-300">
                  {h.peak_c.toFixed(1)}°C
                </span>
              </li>
            ))}
          </ul>
        )}
      </div>
    </div>
  );
}

function Stat({ label, value, accent }: { label: string; value: string; accent?: boolean }) {
  return (
    <div className="rounded-lg border border-white/5 bg-black/30 px-3 py-2">
      <div className="lbl text-[10px]">{label}</div>
      <div className={`mt-1 text-lg font-semibold ${accent ? "text-ember-300" : "text-slate-100"}`}>
        {value}
      </div>
    </div>
  );
}
