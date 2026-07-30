import { useEffect, useRef } from "react";

type Envelope = {
  thing: string;
  data: {
    rows: number;
    cols: number;
    grid: number[][];
    unit: string;
  };
};

type Analysis = {
  thing: string;
  data: {
    stats: { min_c: number; max_c: number; mean_c: number };
    hotspots: { id: number; centroid: [number, number]; peak_c: number; pixels: number }[];
    severity: "nominal" | "elevated" | "critical";
  };
};

export function ThermalView({
  envelope,
  analysis,
  height = 360,
}: {
  envelope: Envelope | null;
  analysis: Analysis | null;
  height?: number;
}) {
  const ref = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = ref.current;
    if (!canvas || !envelope) return;
    const { rows, cols, grid } = envelope.data;
    const cellW = 12;
    const cellH = 12;
    const w = cols * cellW;
    const h = rows * cellH;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    canvas.style.width = "100%";
    canvas.style.height = `${height}px`;
    const ctx = canvas.getContext("2d")!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    // Compute min/max for this frame so colours span the visible range.
    let vmin = Infinity, vmax = -Infinity;
    for (const row of grid) for (const v of row) {
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
    }
    // Bias so a fire really pops
    vmax = Math.max(vmax, 60);

    for (let y = 0; y < rows; y++) {
      for (let x = 0; x < cols; x++) {
        const v = grid[y][x];
        const t = Math.max(0, Math.min(1, (v - vmin) / (vmax - vmin || 1)));
        ctx.fillStyle = ironbow(t);
        ctx.fillRect(x * cellW, y * cellH, cellW + 1, cellH + 1);
      }
    }

    // Hotspot rings
    if (analysis?.data.hotspots) {
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1.5;
      ctx.font = "10px Inter, system-ui, sans-serif";
      ctx.fillStyle = "#ffffff";
      for (const hs of analysis.data.hotspots) {
        const [cx, cy] = hs.centroid;
        const rad = Math.max(8, Math.sqrt(hs.pixels) * 6);
        ctx.beginPath();
        ctx.arc(cx * cellW + cellW / 2, cy * cellH + cellH / 2, rad, 0, Math.PI * 2);
        ctx.stroke();
        ctx.fillText(
          `${hs.peak_c.toFixed(1)}°C`,
          cx * cellW + rad,
          cy * cellH - 2,
        );
      }
    }
  }, [envelope, analysis, height]);

  return (
    <div className="overflow-hidden rounded-xl bg-black/60 ring-1 ring-white/5">
      {envelope ? (
        <canvas ref={ref} className="block w-full" />
      ) : (
        <div className="flex items-center justify-center text-sm text-slate-500" style={{ height }}>
          waiting for thermal stream…
        </div>
      )}
    </div>
  );
}

// "Ironbow" colormap approximation — black → purple → red → orange → yellow → white
function ironbow(t: number): string {
  const stops: [number, [number, number, number]][] = [
    [0.0, [0, 0, 15]],
    [0.2, [60, 0, 90]],
    [0.4, [180, 30, 60]],
    [0.6, [235, 90, 30]],
    [0.8, [250, 190, 60]],
    [1.0, [255, 255, 220]],
  ];
  for (let i = 0; i < stops.length - 1; i++) {
    const [a, ca] = stops[i];
    const [b, cb] = stops[i + 1];
    if (t <= b) {
      const f = (t - a) / (b - a);
      const r = Math.round(ca[0] + (cb[0] - ca[0]) * f);
      const g = Math.round(ca[1] + (cb[1] - ca[1]) * f);
      const bl = Math.round(ca[2] + (cb[2] - ca[2]) * f);
      return `rgb(${r},${g},${bl})`;
    }
  }
  return "rgb(255,255,220)";
}
