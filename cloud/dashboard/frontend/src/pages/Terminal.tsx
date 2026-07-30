import { useEffect, useMemo, useRef, useState } from "react";
import { Terminal as XTerm } from "xterm";
import { FitAddon } from "xterm-addon-fit";
import "xterm/css/xterm.css";
import { Card, CardHeader } from "../components/Card";
import { apiGet } from "../lib/api";

type Mode = "ssh" | "local";

type Thing = { name: string; arn: string; attributes: Record<string, string> };

export default function Terminal({ isLan }: { isLan: boolean }) {
  const [mode, setMode] = useState<Mode>("ssh");
  const [host, setHost] = useState("");
  const [username, setUsername] = useState("orangepi");
  const [password, setPassword] = useState("");
  const [port, setPort] = useState("22");
  const [connected, setConnected] = useState(false);
  const [things, setThings] = useState<Thing[]>([]);

  const containerRef = useRef<HTMLDivElement>(null);
  const termRef = useRef<XTerm | null>(null);
  const fitRef = useRef<FitAddon | null>(null);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    apiGet<{ things: Thing[] }>("/api/aws/things")
      .then((r) => setThings(r.things))
      .catch(() => {});
  }, []);

  // Set up xterm.js once
  useEffect(() => {
    if (!containerRef.current) return;
    const t = new XTerm({
      cursorBlink: true,
      fontFamily: "'JetBrains Mono', ui-monospace, monospace",
      fontSize: 13,
      theme: {
        background: "#07090d",
        foreground: "#e5e7eb",
        cursor: "#f97316",
        black: "#1a2233",
        brightBlack: "#334155",
      },
      convertEol: true,
      scrollback: 5000,
    });
    const fit = new FitAddon();
    t.loadAddon(fit);
    t.open(containerRef.current);
    fit.fit();
    t.writeln("\x1b[90mFPMS terminal ready. Fill in the fields above and click Connect.\x1b[0m");

    termRef.current = t;
    fitRef.current = fit;

    const onResize = () => {
      try { fit.fit(); sendResize(); } catch { /* no-op */ }
    };
    window.addEventListener("resize", onResize);

    return () => {
      window.removeEventListener("resize", onResize);
      wsRef.current?.close();
      t.dispose();
    };
  }, []);

  const sendResize = () => {
    const t = termRef.current;
    const ws = wsRef.current;
    if (!t || !ws || ws.readyState !== WebSocket.OPEN) return;
    const msg = "\x1b\x00" + JSON.stringify({ kind: "resize", cols: t.cols, rows: t.rows });
    ws.send(msg);
  };

  const canConnect = useMemo(() => {
    if (connected) return false;
    if (mode === "local") return isLan;
    return host.trim().length > 0 && username.trim().length > 0;
  }, [connected, mode, host, username, isLan]);

  const connect = () => {
    if (!termRef.current) return;
    const t = termRef.current;
    t.clear();

    const proto = window.location.protocol === "https:" ? "wss" : "ws";
    const base = `${proto}://${window.location.host}`;
    let url: string;
    if (mode === "local") {
      url = `${base}/ws/term/local`;
    } else {
      const q = new URLSearchParams({
        host, port, username,
        ...(password ? { password } : {}),
      });
      url = `${base}/ws/term/ssh?${q}`;
    }
    const ws = new WebSocket(url);
    wsRef.current = ws;

    ws.onopen = () => {
      setConnected(true);
      sendResize();
    };
    ws.onmessage = (ev) => t.write(ev.data);
    ws.onclose = () => {
      setConnected(false);
      t.writeln("\r\n\x1b[90msession ended.\x1b[0m");
    };
    ws.onerror = () => t.writeln("\r\n\x1b[31mconnection error.\x1b[0m");

    t.onData((d) => {
      if (ws.readyState === WebSocket.OPEN) ws.send(d);
    });
    t.onResize(() => sendResize());
  };

  const disconnect = () => {
    wsRef.current?.close();
  };

  return (
    <div className="space-y-6">
      <div>
        <div className="lbl">Terminal · SSH + local shell</div>
        <h1 className="h-page mt-1">Interactive shell</h1>
        <p className="mt-2 max-w-3xl text-sm text-slate-400">
          SSH into any device the app has provisioned, or run a local PowerShell if you're on the LAN.
        </p>
      </div>

      <Card>
        <CardHeader
          title={mode === "ssh" ? "SSH session" : "Local PowerShell"}
          subtitle="Session setup"
          right={
            <div className="flex items-center gap-2">
              <span className={connected ? "chip-ok" : "chip"}>
                <span className={`inline-block h-2 w-2 rounded-full ${connected ? "bg-emerald-400" : "bg-slate-500"}`} />
                {connected ? "connected" : "idle"}
              </span>
              {isLan
                ? <span className="chip-ok">LAN — local shell allowed</span>
                : <span className="chip-warn">public — SSH only</span>}
            </div>
          }
        />

        <div className="mb-3 flex gap-1 rounded-lg border border-white/5 bg-black/30 p-1 text-sm">
          <button
            onClick={() => setMode("ssh")}
            className={`flex-1 rounded-md px-3 py-1.5 font-medium transition ${mode === "ssh" ? "bg-ember-500/20 text-ember-200" : "text-slate-400"}`}
          >
            SSH · to any host
          </button>
          <button
            onClick={() => setMode("local")}
            disabled={!isLan}
            title={isLan ? "" : "Disabled — you are not on the LAN"}
            className={`flex-1 rounded-md px-3 py-1.5 font-medium transition ${mode === "local" ? "bg-ember-500/20 text-ember-200" : "text-slate-400"} disabled:opacity-40 disabled:cursor-not-allowed`}
          >
            Local · PowerShell
          </button>
        </div>

        {mode === "ssh" ? (
          <div className="grid gap-3 md:grid-cols-[1fr_100px_1fr_1fr_auto]">
            <Field label="Host / IP" value={host} onChange={setHost} placeholder="192.168.0.42" mono />
            <Field label="Port" value={port} onChange={setPort} mono />
            <Field label="Username" value={username} onChange={setUsername} mono />
            <Field label="Password" value={password} onChange={setPassword} type="password" />
            <div className="flex items-end">
              {connected
                ? <button className="btn-danger" onClick={disconnect}>Disconnect</button>
                : <button className="btn-primary" onClick={connect} disabled={!canConnect}>Connect</button>}
            </div>
          </div>
        ) : (
          <div className="flex items-end justify-between gap-4">
            <p className="text-sm text-slate-400">
              Runs <code className="rounded bg-black/50 px-1 py-0.5 text-xs">powershell.exe</code> on the laptop
              hosting this dashboard. Only reachable when you're on the same network.
            </p>
            {connected
              ? <button className="btn-danger" onClick={disconnect}>Disconnect</button>
              : <button className="btn-primary" onClick={connect} disabled={!canConnect}>Open PowerShell</button>}
          </div>
        )}

        {things.length > 0 && mode === "ssh" && (
          <div className="mt-3">
            <div className="lbl mb-1">Quick connect · from Devices</div>
            <div className="flex flex-wrap gap-1.5">
              {things.map((t) => {
                const ip = t.attributes?.ip;
                if (!ip) return null;
                return (
                  <button
                    key={t.name}
                    className="chip"
                    onClick={() => setHost(ip)}
                  >
                    {t.name} · {ip}
                  </button>
                );
              })}
            </div>
          </div>
        )}
      </Card>

      <div className="card overflow-hidden">
        <div ref={containerRef} style={{ height: "60vh", padding: 12 }} />
      </div>

      <div className="rounded-lg border border-amber-500/20 bg-amber-500/5 p-3 text-xs text-amber-200">
        Terminal traffic never leaves your device or the SSH target. All bytes are proxied
        through the FPMS backend over WebSocket. Sessions end when you close the tab or click Disconnect.
      </div>
    </div>
  );
}

function Field({
  label, value, onChange, type = "text", placeholder, mono,
}: {
  label: string; value: string; onChange: (v: string) => void;
  type?: string; placeholder?: string; mono?: boolean;
}) {
  return (
    <label className="block">
      <div className="lbl">{label}</div>
      <input
        type={type}
        value={value}
        placeholder={placeholder}
        onChange={(e) => onChange(e.target.value)}
        className={`mt-1 w-full rounded-lg border border-white/10 bg-black/40 px-3 py-2 text-sm text-slate-100 outline-none focus:border-ember-500/50 ${mono ? "font-mono" : ""}`}
      />
    </label>
  );
}
