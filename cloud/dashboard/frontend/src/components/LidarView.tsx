import { useEffect, useRef } from "react";

type Envelope = {
  thing: string;
  data: {
    ranges_m: number[];
    range_max_m: number;
    heading_deg?: number;
  };
};

export function LidarView({
  envelope,
  size = 380,
  accent = "#f97316",
}: {
  envelope: Envelope | null;
  size?: number;
  accent?: string;
}) {
  const ref = useRef<HTMLCanvasElement>(null);

  // Sizing is a separate effect from drawing. Setting canvas.width resets the
  // backing store and the transform, so doing it per frame both throws away
  // the previous buffer and stacks another ctx.scale(dpr) on the last one.
  // This effect only runs when `size` actually changes; setTransform (rather
  // than scale) makes it idempotent.
  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = size * dpr;
    canvas.height = size * dpr;
    canvas.style.width = `${size}px`;
    canvas.style.height = `${size}px`;
    canvas.getContext("2d")?.setTransform(dpr, 0, 0, dpr, 0, 0);
  }, [size]);

  useEffect(() => {
    const ctx = ref.current?.getContext("2d");
    if (!ctx) return;
    draw(ctx, size, envelope, accent);
  }, [envelope, size, accent]);

  return <canvas ref={ref} className="rounded-xl bg-black/50 ring-1 ring-white/5" />;
}

function draw(
  ctx: CanvasRenderingContext2D,
  size: number,
  env: Envelope | null,
  accent: string,
) {
  ctx.clearRect(0, 0, size, size);
  const cx = size / 2;
  const cy = size / 2;
  const rmax = Math.min(cx, cy) - 12;
  const maxRange = env?.data.range_max_m ?? 6.0;

  // radial grid
  ctx.strokeStyle = "rgba(148, 163, 184, 0.08)";
  ctx.lineWidth = 1;
  for (let i = 1; i <= 5; i++) {
    ctx.beginPath();
    ctx.arc(cx, cy, (rmax * i) / 5, 0, Math.PI * 2);
    ctx.stroke();
  }
  // axes
  ctx.beginPath();
  ctx.moveTo(cx - rmax, cy); ctx.lineTo(cx + rmax, cy);
  ctx.moveTo(cx, cy - rmax); ctx.lineTo(cx, cy + rmax);
  ctx.stroke();

  // range labels
  ctx.fillStyle = "rgba(148, 163, 184, 0.5)";
  ctx.font = "10px Inter, system-ui, sans-serif";
  for (let i = 1; i <= 5; i++) {
    const r = (rmax * i) / 5;
    const m = ((maxRange * i) / 5).toFixed(1);
    ctx.fillText(`${m}m`, cx + 4, cy - r + 10);
  }

  // rover triangle (heading up)
  const heading = env?.data.heading_deg ?? 0;
  const hRad = (heading * Math.PI) / 180;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(hRad);
  ctx.fillStyle = accent;
  ctx.beginPath();
  ctx.moveTo(0, -8);
  ctx.lineTo(6, 6);
  ctx.lineTo(-6, 6);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  if (!env) {
    ctx.fillStyle = "rgba(148,163,184,0.6)";
    ctx.font = "12px Inter, system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("waiting for LiDAR…", cx, cy + rmax + 24);
    return;
  }

  const ranges = env.data.ranges_m;
  // point cloud
  ctx.fillStyle = accent;
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i];
    if (!(r > 0)) continue; // 0.0 = no return, not a hit at zero range
    const norm = Math.min(1, r / maxRange);
    const px = cx + Math.cos(((i - 90) * Math.PI) / 180) * norm * rmax;
    const py = cy + Math.sin(((i - 90) * Math.PI) / 180) * norm * rmax;
    ctx.globalAlpha = 0.65 + 0.35 * (1 - norm);
    ctx.fillRect(px - 1.2, py - 1.2, 2.4, 2.4);
  }
  ctx.globalAlpha = 1;

  // Ghost outline. Empty bins are 0.0, and connecting through them dragged the
  // polygon into the centre once per gap — the "starburst" that made every
  // scan look like a sensor fault. Break the path at each gap instead of
  // closing one continuous loop.
  ctx.beginPath();
  let penDown = false;
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i];
    if (!(r > 0)) {
      penDown = false;
      continue;
    }
    const norm = Math.min(1, r / maxRange);
    const px = cx + Math.cos(((i - 90) * Math.PI) / 180) * norm * rmax;
    const py = cy + Math.sin(((i - 90) * Math.PI) / 180) * norm * rmax;
    if (penDown) ctx.lineTo(px, py);
    else ctx.moveTo(px, py);
    penDown = true;
  }
  ctx.strokeStyle = "rgba(249,115,22,0.35)";
  ctx.lineWidth = 1;
  ctx.stroke();

  ctx.textAlign = "start";
}
