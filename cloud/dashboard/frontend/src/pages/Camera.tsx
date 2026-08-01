import { useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { StatusPill } from "../components/StatusPill";
import { CameraView } from "../components/CameraView";
import { useChannel } from "../lib/ws";
import { apiPost } from "../lib/api";
import { useThings } from "../lib/things";

/**
 * Both rovers side by side, mirroring the LiDAR page. Previously this page
 * showed a single hard-coded rover1 feed, so a fleet running rover2 saw
 * "waiting for camera feed" forever while frames streamed past.
 */
export default function Camera() {
  const things = useThings();
  // Always show both bays so a rover that has not reported yet is visibly
  // absent rather than silently missing from the page.
  const bays = Array.from(new Set([...things, "rover1", "rover2"])).sort().slice(0, 4);

  return (
    <div className="space-y-6">
      <div className="flex items-end justify-between">
        <div>
          <div className="lbl">Page 3</div>
          <h1 className="h-page mt-1">YOLO cameras — both Orange Pi 5B rovers</h1>
        </div>
        <div className="text-xs text-slate-500">
          {things.length
            ? `${things.length} rover${things.length > 1 ? "s" : ""} reporting: ${things.join(", ")}`
            : "no rovers reporting"}
        </div>
      </div>

      <div className="grid gap-6 lg:grid-cols-2">
        {bays.map((t) => (
          <RoverCamera key={t} thing={t} live={things.includes(t)} />
        ))}
      </div>

      <Card>
        <CardHeader title="How this feed works" subtitle="Behind the scenes" />
        <ul className="space-y-2 text-sm leading-relaxed text-slate-300">
          <li>
            Frames are JPEG-encoded on-rover, base64-wrapped and published to{" "}
            <code className="rounded bg-black/50 px-1 py-0.5 text-xs">
              fpms/&lt;rover&gt;/telemetry/camera
            </code>.
          </li>
          <li>
            YOLO inference runs on the RK3588 NPU (6 TOPS). Recorded in a past
            bench run on this hardware — not a live reading, and not a claim
            about any feed above: 20 ms inference, 11 ms decode/NMS, 35 ms
            end-to-end, about 28 fps of headroom. Only bounding boxes travel
            with the frame.
          </li>
          <li>
            A hotspot only triggers action when the thermal camera <em>also</em>{" "}
            agrees — cross-validation.
          </li>
        </ul>
      </Card>
    </div>
  );
}

function RoverCamera({ thing, live }: { thing: string; live: boolean }) {
  const stream = useChannel<any>(`camera:${thing}`);
  const [busy, setBusy] = useState(false);
  // A rejected POST used to be an unhandled promise rejection and nothing else:
  // the button un-greyed and the operator was left believing the command went.
  const [cmdErr, setCmdErr] = useState<string | null>(null);

  const cmd = async (action: "connect" | "disconnect") => {
    setBusy(true);
    setCmdErr(null);
    try { await apiPost(`/api/rover/${thing}/${action}`); }
    catch (e: unknown) { setCmdErr(`${action} was not sent — ${e instanceof Error ? e.message : String(e)}`); }
    finally { setBusy(false); }
  };

  const frame: string | null =
    typeof stream.data?.data?.frame === "string" ? stream.data.data.frame : null;
  const format: string =
    typeof stream.data?.data?.format === "string" ? stream.data.data.format : "jpeg";

  // Reading `.data.data.format` off an envelope that arrived without a payload
  // threw and took the page down. A frame we do not have simply disables the
  // button instead.
  const snapshot = () => {
    if (!frame) return;
    const a = document.createElement("a");
    a.href = `data:image/${format};base64,${frame}`;
    a.download = `${thing}-${Date.now()}.${format === "png" ? "png" : "jpg"}`;
    a.click();
  };

  const detections: any[] = Array.isArray(stream.data?.data?.detections)
    ? stream.data.data.detections
    : [];
  const npu = stream.data?.data?.npu;

  /**
   * Has a frame ever arrived on this channel?
   *
   * This gate exists because the page used to print "Detections 0" and
   * "Nothing detected in this frame." with zero frames received. On a
   * fire-detection dashboard that is the worst available sentence: it reads as
   * a camera that looked and saw no fire, when in fact nothing ever looked.
   * No frame, no claim — the count goes to "--" and the copy says why.
   */
  const gotFrame = stream.messages > 0 && frame !== null;

  return (
    <Card>
      <CardHeader
        title={thing.toUpperCase()}
        subtitle={live ? `RGB · YOLO on NPU${npu === false ? " (NPU off)" : ""}` : "not reporting"}
        right={
          <StatusPill
            connected={stream.connected}
            lastAt={stream.lastAt}
            messages={stream.messages}
          />
        }
      />
      {/* Only ever handed an envelope that actually carries a frame. An
          envelope with detections but no image made CameraView emit
          <img src="data:image/undefined;base64,undefined"> and size its overlay
          canvas to NaN. */}
      <CameraView envelope={gotFrame ? stream.data : null} />

      <div className="mt-3">
        <div className="lbl mb-1.5">
          Detections{" "}
          <span className={gotFrame ? "chip ml-1" : "chip-warn ml-1"}>
            {gotFrame ? detections.length : "--"}
          </span>
        </div>
        {!gotFrame ? (
          <div className="text-sm text-amber-200/90">
            <b>No frame has arrived from {thing}</b> — so there is nothing to
            report on. This is <i>not</i> "nothing detected": the camera has not
            been seen looking. Connect the rover, or check the camera publisher
            on it.
          </div>
        ) : detections.length === 0 ? (
          <div className="text-sm text-slate-500">Nothing detected in this frame.</div>
        ) : (
          <ul className="space-y-1.5">
            {detections.slice(0, 6).map((d, i) => (
              <li
                key={i}
                className="flex items-center justify-between rounded-md border border-white/5 bg-black/30 px-3 py-1.5 text-sm"
              >
                <span className={d.kind === "fire" ? "chip-hot" : "chip"}>
                  {d.label ?? d.cls}
                </span>
                <span className="font-mono text-xs text-slate-400">
                  conf {typeof d.conf === "number" ? d.conf.toFixed(2) : "—"}
                </span>
                <span className="font-mono text-xs text-slate-500">
                  [{Array.isArray(d.box) ? d.box.map((n: number) => Math.round(n)).join(", ") : "—"}]
                </span>
              </li>
            ))}
          </ul>
        )}
      </div>

      {cmdErr && (
        <div className="mt-3 rounded-lg border border-rose-500/40 bg-rose-950/30 px-3 py-2 text-xs text-rose-200">
          {cmdErr}
        </div>
      )}

      <div className="mt-4 flex flex-wrap justify-end gap-2">
        <button
          className="btn"
          onClick={snapshot}
          disabled={!frame}
          title={frame ? "Download this frame" : "No frame has arrived to save"}
        >
          📸 Snapshot
        </button>
        <button className="btn-primary" disabled={busy} onClick={() => cmd("connect")}>
          Connect {thing}
        </button>
        <button className="btn-danger" disabled={busy} onClick={() => cmd("disconnect")}>
          Disconnect
        </button>
      </div>
    </Card>
  );
}
