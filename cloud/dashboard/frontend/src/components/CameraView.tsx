import { useEffect, useRef } from "react";

type Detection = { cls: string; conf: number; box: [number, number, number, number] };

type Envelope = {
  thing: string;
  data: {
    format: string;
    encoding: string;
    frame: string;
    width: number;
    height: number;
    detections: Detection[];
  };
};

export function CameraView({ envelope }: { envelope: Envelope | null }) {
  const overlayRef = useRef<HTMLCanvasElement>(null);
  const imgRef = useRef<HTMLImageElement>(null);

  useEffect(() => {
    if (!envelope) return;
    const overlay = overlayRef.current;
    if (!overlay) return;
    const w = envelope.data.width;
    const h = envelope.data.height;
    const dpr = window.devicePixelRatio || 1;
    overlay.width = w * dpr;
    overlay.height = h * dpr;
    overlay.style.width = "100%";
    overlay.style.height = "auto";
    overlay.style.aspectRatio = `${w} / ${h}`;
    const ctx = overlay.getContext("2d")!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    for (const d of envelope.data.detections ?? []) {
      const [x1, y1, x2, y2] = d.box;
      const color = d.cls === "fire" ? "#f97316" : "#38bdf8";
      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      const label = `${d.cls} ${d.conf.toFixed(2)}`;
      ctx.font = "13px Inter, system-ui, sans-serif";
      const tw = ctx.measureText(label).width + 10;
      ctx.fillStyle = color;
      ctx.fillRect(x1, y1 - 20, tw, 20);
      ctx.fillStyle = "#0b0f16";
      ctx.fillText(label, x1 + 5, y1 - 6);
    }
  }, [envelope]);

  return (
    <div className="relative overflow-hidden rounded-xl bg-black ring-1 ring-white/5">
      {envelope ? (
        <>
          <img
            ref={imgRef}
            alt="YOLO camera stream"
            src={`data:image/${envelope.data.format};base64,${envelope.data.frame}`}
            className="block w-full"
          />
          <canvas
            ref={overlayRef}
            className="pointer-events-none absolute inset-0"
          />
        </>
      ) : (
        <div className="flex aspect-[4/3] items-center justify-center text-sm text-slate-500">
          waiting for camera feed…
        </div>
      )}
    </div>
  );
}
