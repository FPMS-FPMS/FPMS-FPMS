import { useEffect, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import SharePanel from "../components/SharePanel";
import { apiGet } from "../lib/api";

type Platform = {
  id: "windows" | "macos" | "linux" | "ios" | "android";
  name: string;
  type: string;
  size_bytes: number | null;
  available: boolean;
  download_url: string | null;
  install_notes: string[];
};

type Catalog = { platforms: Platform[]; server_host: string; public_note: string };

const ICONS: Record<Platform["id"], string> = {
  windows: "🪟",
  macos: "🍎",
  linux: "🐧",
  ios: "📱",
  android: "🤖",
};

export default function Install() {
  const [cat, setCat] = useState<Catalog | null>(null);

  useEffect(() => {
    apiGet<Catalog>("/api/downloads").then(setCat).catch(() => {});
  }, []);

  return (
    <div className="space-y-6">
      <div>
        <div className="lbl">Install FPMS · every platform</div>
        <h1 className="h-page mt-1">Download or install as a Progressive Web App</h1>
        <p className="mt-2 max-w-3xl text-sm text-slate-400">
          FPMS runs on this laptop. Windows gets a signed-style native <code className="rounded bg-black/50 px-1 py-0.5 text-xs">.exe</code>;
          macOS, Linux, iOS, and Android use the Progressive Web App — same UI, same features,
          installs to your dock / launcher / home screen and opens fullscreen.
        </p>
      </div>

      <SharePanel />

      <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
        {cat?.platforms.map((p) => (
          <Card key={p.id}>
            <CardHeader
              title={p.name}
              subtitle={p.type}
              right={
                <span className="text-2xl" aria-hidden>{ICONS[p.id]}</span>
              }
            />
            {p.download_url && p.available && (
              <a
                href={p.download_url}
                className="btn-primary mb-4 w-full justify-center py-2 text-sm font-semibold"
                download
              >
                ⬇ Download {p.size_bytes ? `(${humanBytes(p.size_bytes)})` : ""}
              </a>
            )}
            {!p.download_url && (
              <div className="mb-4 rounded-md border border-white/5 bg-black/30 p-3 text-xs text-slate-400">
                No native installer for {p.name}. Follow the PWA steps below — same features.
              </div>
            )}
            <div className="lbl mb-1.5">Steps</div>
            <ol className="space-y-1.5 text-xs leading-relaxed text-slate-300">
              {p.install_notes.map((step, i) => (
                <li key={i} className="flex gap-2">
                  <span className="font-mono text-slate-500">{i + 1}.</span>
                  <span>{step}</span>
                </li>
              ))}
            </ol>
          </Card>
        ))}
      </div>

      <Card>
        <CardHeader title="What runs where" subtitle="Everything is on this laptop" />
        <div className="grid gap-4 md:grid-cols-3 text-sm">
          <div>
            <div className="font-semibold text-slate-100">The app</div>
            <p className="mt-1 text-slate-400">Native window on Windows via WebView2. Elsewhere it's a PWA — Chrome/Safari's install-as-app.</p>
          </div>
          <div>
            <div className="font-semibold text-slate-100">The backend</div>
            <p className="mt-1 text-slate-400">FastAPI on this laptop, bound to 0.0.0.0:8000. Serves the UI, the API, and every WebSocket stream.</p>
          </div>
          <div>
            <div className="font-semibold text-slate-100">Public access</div>
            <p className="mt-1 text-slate-400">Optional. Run <code className="rounded bg-black/50 px-1 py-0.5">Publish-Public.bat</code> to expose this laptop's app on a public HTTPS URL (Cloudflare Tunnel, free). Password required.</p>
          </div>
        </div>
      </Card>
    </div>
  );
}

function humanBytes(n: number): string {
  const units = ["B", "KB", "MB", "GB"];
  let i = 0, v = n;
  while (v >= 1024 && i < units.length - 1) { v /= 1024; i++; }
  return `${v.toFixed(v >= 100 ? 0 : 1)} ${units[i]}`;
}
