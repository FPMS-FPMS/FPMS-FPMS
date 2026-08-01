import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Terminal as XTerm } from "xterm";
import type { IDisposable } from "xterm";
import { FitAddon } from "xterm-addon-fit";
import "xterm/css/xterm.css";
import { Card, CardHeader } from "../components/Card";
import { apiGet } from "../lib/api";

/**
 * THE CONSOLE.
 *
 * Wire protocol, verified against backend/terminal.py + the two WebSocket
 * routes in backend/main.py:
 *
 *   /ws/term/ssh?host=&port=&username=&key_path=
 *        accept() first, THEN paramiko connects. Failures are reported as
 *        ANSI text on the socket ("SSH connect failed: ..."), not as a close
 *        code — so a session that dies still has something to read.
 *
 * NO SECRET IS EVER PUT IN THAT URL. The backend also accepts `password=` and
 * this page used to send it: a URL is logged by every proxy on the path, kept
 * in browser history, and shipped in crash reports, so a masked on-screen echo
 * hid it from the operator while leaving it in a dozen logs. `key_path` is a
 * FILESYSTEM PATH on the machine running the backend, not a credential, so it
 * is safe to pass this way. Password auth stays off until the backend can take
 * a credential off the URL — see PASSWORD_AUTH_NOTE below for the exact change.
 *   /ws/term/local
 *        refused before accept unless auth.is_lan(client). When refused the
 *        server accepts, writes one red line, and closes.
 *
 *   browser -> server : UTF-8 keystrokes, or "\x1b\x00" + JSON for control
 *                       messages. The only control message is
 *                       {kind:"resize", cols, rows}.
 *   server -> browser : UTF-8 shell output.
 *
 * WHAT THIS PAGE MUST NEVER DO IS SHOW A BLANK BLACK BOX. Every way this can
 * fail produces either zero bytes or an immediate close, and both used to look
 * identical to "the rover has not typed anything yet":
 *
 *   - safe mode / cloud role refuses /ws/term/*  -> close 1008, no bytes
 *   - `import paramiko` raises in the backend    -> close, no bytes (the import
 *     sits OUTSIDE run_ssh_session's try, so nothing is ever sent)
 *   - the app is served by something that does not proxy Upgrade -> no open
 *   - local shell off-LAN                        -> one red line, then close
 *
 * So the socket is instrumented: did it open, did a single byte arrive, what
 * close code came back, and how long did it last. `diagnose()` turns that into
 * a sentence naming the cause and the fix.
 */

type Mode = "ssh" | "local";

type Thing = { name: string; arn: string; attributes: Record<string, string> };

/** Shape of /api/auth-status (backend/main.py::auth_status). */
type AuthStatus = {
  auth_required: boolean;
  authenticated: boolean;
  is_lan: boolean;
  client_host: string | null;
  is_remote?: boolean;
  safe_mode?: boolean;
  role?: string;
  controls_disabled?: boolean;
};

type Phase = "idle" | "connecting" | "open" | "closed";

type SessionStats = {
  url: string;
  mode: Mode;
  startedAt: number;
  openedAt: number | null;
  closedAt: number | null;
  bytesIn: number;
  bytesOut: number;
  lastDataAt: number | null;
  closeCode: number | null;
  closeReason: string;
  sawError: boolean;
  /** First slice of what the server said, kept so a diagnosis can quote it. */
  head: string;
};

const CLOSE_CODES: Record<number, string> = {
  1000: "normal close — the server ended the session",
  1001: "going away — the backend is restarting, or the tab navigated",
  1005: "no status code (what a shell that simply exited looks like)",
  1006: "abnormal close — no close frame arrived at all",
  1008: "policy violation — the backend refused /ws/term/* for this visitor",
  1011: "the backend hit an unhandled error while running the session",
};

/** How long to wait for the HTTP upgrade before calling it a failure. */
const OPEN_TIMEOUT_MS = 12_000;

/**
 * Why there is no password box, and what would bring it back.
 *
 * backend/terminal.py::run_ssh_session takes its credential as an argument that
 * backend/main.py reads straight off `ws.query_params`, and it calls
 * paramiko.connect() before reading a single frame from the socket. There is
 * therefore no point at which this page could hand over a password without
 * putting it in the URL — the handler has already decided whether it succeeded.
 *
 * The backend change that fixes it (one of):
 *   a) have /ws/term/ssh accept the socket, await ONE frame, and read
 *      {kind:"auth", password} from it before connecting; or
 *   b) add a POST /api/terminal/ticket that takes the credential in a JSON body
 *      and returns a single-use, short-TTL token; /ws/term/ssh then takes
 *      ?ticket=… and redeems it server-side.
 * Either keeps the secret out of every access log on the path. (b) also keeps
 * it out of the browser's own WebSocket URL bookkeeping.
 */
const PASSWORD_AUTH_NOTE =
  "Password auth is deliberately unavailable here: the backend only accepts an SSH " +
  "password as a URL query parameter, and URLs end up in proxy logs, browser history " +
  "and crash reports. Use a key instead, or change the backend to take the credential " +
  "off the URL.";

function emptyStats(url: string, mode: Mode): SessionStats {
  return {
    url,
    mode,
    startedAt: Date.now(),
    openedAt: null,
    closedAt: null,
    bytesIn: 0,
    bytesOut: 0,
    lastDataAt: null,
    closeCode: null,
    closeReason: "",
    sawError: false,
    head: "",
  };
}

export default function Terminal({ isLan }: { isLan: boolean }) {
  const [mode, setMode] = useState<Mode>("ssh");
  const [host, setHost] = useState("");
  const [username, setUsername] = useState("orangepi");
  // A path on the BACKEND host, not a secret. Never a password — see
  // PASSWORD_AUTH_NOTE.
  const [keyPath, setKeyPath] = useState("");
  const [port, setPort] = useState("22");
  const [phase, setPhase] = useState<Phase>("idle");
  const [things, setThings] = useState<Thing[]>([]);
  const [auth, setAuth] = useState<AuthStatus | null>(null);
  const [authErr, setAuthErr] = useState<string | null>(null);

  const containerRef = useRef<HTMLDivElement>(null);
  const termRef = useRef<XTerm | null>(null);
  const fitRef = useRef<FitAddon | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const statsRef = useRef<SessionStats | null>(null);
  const openTimerRef = useRef<number | null>(null);
  // Bumped once a second so the byte counters and the "last output" age move
  // without a re-render per keystroke on a fast SSH stream.
  const [, setTick] = useState(0);

  useEffect(() => {
    const id = window.setInterval(() => setTick((n) => n + 1), 1000);
    return () => window.clearInterval(id);
  }, []);

  const loadAuth = useCallback(() => {
    apiGet<AuthStatus>("/api/auth-status")
      .then((a) => { setAuth(a); setAuthErr(null); })
      .catch((e) => setAuthErr(String(e)));
  }, []);

  useEffect(() => {
    loadAuth();
    apiGet<{ things: Thing[] }>("/api/aws/things")
      .then((r) => setThings(r.things))
      .catch(() => { /* registry is optional; quick-connect just stays empty */ });
  }, [loadAuth]);

  const sendResize = useCallback(() => {
    const t = termRef.current;
    const ws = wsRef.current;
    if (!t || !ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send("\x1b\x00" + JSON.stringify({ kind: "resize", cols: t.cols, rows: t.rows }));
  }, []);

  // Set up xterm.js once.
  //
  // onData/onResize are wired HERE and not in connect(). They used to be
  // registered on every connect, and xterm never removes a handler you do not
  // dispose — so the second session sent every keystroke twice, the third three
  // times, and the shell saw "lls" for "ls".
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
    try { fit.fit(); } catch { /* zero-size container on first paint */ }
    t.writeln("\x1b[90mFPMS console. Nothing is connected yet — fill in the fields above and press Connect.\x1b[0m");

    termRef.current = t;
    fitRef.current = fit;

    const disposables: IDisposable[] = [
      t.onData((d) => {
        const ws = wsRef.current;
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send(d);
          if (statsRef.current) statsRef.current.bytesOut += d.length;
        }
      }),
      t.onResize(() => sendResize()),
    ];

    // The window is not the only thing that changes this element's size — the
    // nav collapses, the diagnosis panel appears and disappears. A ResizeObserver
    // catches all of it; the old window-only listener left the shell convinced
    // it had the old geometry, which is how a full-screen editor ends up drawing
    // over itself.
    const refit = () => {
      try { fit.fit(); sendResize(); } catch { /* not laid out yet */ }
    };
    const ro = new ResizeObserver(refit);
    ro.observe(containerRef.current);
    window.addEventListener("resize", refit);

    return () => {
      window.removeEventListener("resize", refit);
      ro.disconnect();
      for (const d of disposables) d.dispose();
      wsRef.current?.close();
      t.dispose();
    };
  }, [sendResize]);

  const quickConnect = useMemo(
    () =>
      things
        .map((t) => ({ name: t.name, ip: t.attributes?.ip }))
        .filter((t): t is { name: string; ip: string } => typeof t.ip === "string" && t.ip !== ""),
    [things],
  );

  const blockers = useMemo(
    () => preflight(mode, auth, authErr, isLan, keyPath),
    [mode, auth, authErr, isLan, keyPath],
  );
  const hardBlocked = blockers.some((b) => b.hard);

  const canConnect = useMemo(() => {
    if (phase === "connecting" || phase === "open") return false;
    if (hardBlocked) return false;
    if (mode === "local") return isLan;
    return host.trim().length > 0 && username.trim().length > 0;
  }, [phase, hardBlocked, mode, host, username, isLan]);

  const connect = () => {
    const t = termRef.current;
    if (!t) return;
    // Re-read the gate: safe mode can be flipped on the server after this tab
    // loaded, and a stale "allowed" is exactly how you get a mystery 1008.
    loadAuth();

    wsRef.current?.close();
    t.reset();

    const proto = window.location.protocol === "https:" ? "wss" : "ws";
    const base = `${proto}://${window.location.host}`;
    let url: string;
    let shown: string;
    if (mode === "local") {
      url = `${base}/ws/term/local`;
      shown = url;
    } else {
      // Every parameter here is non-secret by construction: a hostname, a port,
      // a login name, and a path on the backend's own filesystem. If you are
      // ever tempted to add `password` back, read PASSWORD_AUTH_NOTE first.
      const q = new URLSearchParams({
        host, port, username,
        ...(keyPath.trim() ? { key_path: keyPath.trim() } : {}),
      });
      url = `${base}/ws/term/ssh?${q}`;
      // Safe to echo in full — there is nothing secret in it.
      shown = url;
    }

    const stats = emptyStats(shown, mode);
    statsRef.current = stats;
    setPhase("connecting");
    t.writeln(`\x1b[90mopening ${shown}\x1b[0m`);
    t.writeln("\x1b[90mwaiting for the backend to accept the upgrade…\x1b[0m");

    const ws = new WebSocket(url);
    wsRef.current = ws;

    // A WebSocket that is refused at the HTTP layer can sit in CONNECTING for a
    // long time behind some proxies. Without this the page shows "connecting…"
    // forever and never says anything at all.
    openTimerRef.current = window.setTimeout(() => {
      if (ws.readyState === WebSocket.CONNECTING) {
        t.writeln(`\r\n\x1b[31mno response after ${OPEN_TIMEOUT_MS / 1000}s — giving up on the upgrade.\x1b[0m`);
        ws.close();
      }
    }, OPEN_TIMEOUT_MS);

    ws.onopen = () => {
      stats.openedAt = Date.now();
      setPhase("open");
      sendResize();
    };
    ws.onmessage = (ev) => {
      const s = typeof ev.data === "string" ? ev.data : "";
      stats.bytesIn += s.length;
      stats.lastDataAt = Date.now();
      if (stats.head.length < 600) stats.head = (stats.head + s).slice(0, 600);
      t.write(ev.data);
    };
    ws.onerror = () => {
      stats.sawError = true;
    };
    ws.onclose = (ev) => {
      if (openTimerRef.current) {
        window.clearTimeout(openTimerRef.current);
        openTimerRef.current = null;
      }
      stats.closedAt = Date.now();
      stats.closeCode = ev.code;
      stats.closeReason = ev.reason || "";
      setPhase("closed");
      const why = CLOSE_CODES[ev.code];
      t.writeln(
        `\r\n\x1b[90msession ended · close ${ev.code}${ev.reason ? ` "${ev.reason}"` : ""}` +
        `${why ? ` · ${why}` : ""}\x1b[0m`,
      );
      // The diagnosis panel below the terminal has the full story; point at it
      // rather than duplicating paragraphs into the scrollback.
      t.writeln("\x1b[90msee the panel under this window for what to do about it.\x1b[0m");
    };
  };

  const disconnect = () => wsRef.current?.close();

  const stats = statsRef.current;
  const diag = phase === "closed" && stats ? diagnose(stats, auth, isLan) : null;
  const connected = phase === "open";

  return (
    <div className="space-y-6">
      <div>
        <div className="lbl">Console · SSH + local shell</div>
        <h1 className="h-page mt-1">Interactive shell</h1>
        <p className="mt-2 max-w-3xl text-sm text-slate-400">
          SSH into any host this laptop can reach — a rover, a switch, anything — using a
          private key stored on the laptop, or open a PowerShell on the laptop itself. Both
          are proxied over a WebSocket to{" "}
          <span className="font-mono text-slate-300">/ws/term/*</span>, and no credential is
          ever put in that URL.
        </p>
      </div>

      {/* Availability — answered BEFORE anyone presses Connect. */}
      <Card>
        <CardHeader
          title="Can this console connect?"
          subtitle="Preflight"
          right={
            <button className="btn" onClick={loadAuth}>Re-check</button>
          }
        />
        {blockers.length === 0 ? (
          <div className="rounded-lg border border-emerald-500/25 bg-emerald-500/5 p-3 text-sm text-emerald-200">
            Nothing is blocking a session. {mode === "local"
              ? "Local PowerShell is allowed from this address."
              : "SSH goes out from the laptop, so it reaches whatever the laptop can reach."}
          </div>
        ) : (
          <ul className="space-y-2">
            {blockers.map((b, i) => (
              <li
                key={i}
                className={`rounded-lg border p-3 text-sm ${
                  b.hard
                    ? "border-rose-500/30 bg-rose-500/5 text-rose-100"
                    : "border-amber-500/25 bg-amber-500/5 text-amber-100"
                }`}
              >
                <div className="font-semibold">{b.hard ? "Blocked" : "Heads up"} · {b.title}</div>
                <div className="mt-1 text-xs leading-relaxed opacity-90">{b.detail}</div>
                {b.fix && <div className="mt-1 text-xs leading-relaxed opacity-75">Fix: {b.fix}</div>}
              </li>
            ))}
          </ul>
        )}
        <div className="mt-3 grid gap-2 text-xs text-slate-400 sm:grid-cols-2 lg:grid-cols-4">
          <Fact
            label="Signed in"
            value={
              authErr ? "--"
                : !auth ? "--"
                : auth.auth_required ? (auth.authenticated ? "yes" : "NO") : "no password set"
            }
          />
          <Fact label="Your address" value={auth?.client_host ?? "--"} />
          <Fact label="On the LAN" value={auth ? (auth.is_lan ? "yes" : "no") : "--"} />
          <Fact
            label="Backend gate"
            value={
              !auth ? "--"
                : auth.controls_disabled ? "terminals refused"
                : auth.safe_mode ? `safe mode on · role ${auth.role ?? "--"}`
                : `open · role ${auth.role ?? "--"}`
            }
          />
        </div>
      </Card>

      <Card>
        <CardHeader
          title={mode === "ssh" ? "SSH session" : "Local PowerShell"}
          subtitle="Session setup"
          right={
            <div className="flex items-center gap-2">
              <span className={connected ? "chip-ok" : phase === "connecting" ? "chip-warn" : "chip"}>
                <span
                  className={`inline-block h-2 w-2 rounded-full ${
                    connected ? "bg-emerald-400" : phase === "connecting" ? "bg-amber-400" : "bg-slate-500"
                  }`}
                />
                {phase === "open" ? "connected" : phase === "connecting" ? "connecting…" : phase === "closed" ? "closed" : "idle"}
              </span>
              {isLan
                ? <span className="chip-ok">LAN — local shell allowed</span>
                : <span className="chip-warn">off-LAN — SSH only</span>}
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
            title={isLan ? "" : "Disabled — the backend serves /ws/term/local to LAN clients only"}
            className={`flex-1 rounded-md px-3 py-1.5 font-medium transition ${mode === "local" ? "bg-ember-500/20 text-ember-200" : "text-slate-400"} disabled:opacity-40 disabled:cursor-not-allowed`}
          >
            Local · PowerShell
          </button>
        </div>

        {mode === "ssh" ? (
          <>
            <div className="grid gap-3 md:grid-cols-[1fr_100px_1fr_1.4fr_auto]">
              <Field label="Host / IP" value={host} onChange={setHost} placeholder="192.168.0.42" mono />
              <Field label="Port" value={port} onChange={setPort} mono />
              <Field label="Username" value={username} onChange={setUsername} mono />
              <Field
                label="Private key path (on the laptop)"
                value={keyPath}
                onChange={setKeyPath}
                placeholder="C:\Users\you\.ssh\id_ed25519"
                mono
              />
              <div className="flex items-end">
                {connected || phase === "connecting"
                  ? <button className="btn-danger" onClick={disconnect}>Disconnect</button>
                  : <button className="btn-primary" onClick={connect} disabled={!canConnect}>Connect</button>}
              </div>
            </div>
            <div className="mt-3 rounded-lg border border-white/10 bg-black/30 p-3 text-xs leading-relaxed text-slate-400">
              <div className="font-semibold text-slate-200">There is no password box, on purpose.</div>
              <p className="mt-1">{PASSWORD_AUTH_NOTE}</p>
              <p className="mt-1.5">
                The key path is read by <b>the backend</b>, not by your browser — it must exist on the
                laptop running FPMS. paramiko is called with{" "}
                <span className="font-mono">look_for_keys=False</span> and{" "}
                <span className="font-mono">allow_agent=False</span>, so it will not discover a key or
                use an agent on its own: name the file explicitly, and use a key without a passphrase
                (there is nowhere to type one). Password login for the initial Pi setup still exists
                on the <b>Devices</b> tab, where it travels in a POST body rather than a URL.
              </p>
            </div>
          </>
        ) : (
          <div className="flex items-end justify-between gap-4">
            <p className="text-sm text-slate-400">
              Runs <code className="rounded bg-black/50 px-1 py-0.5 text-xs">pwsh</code> or{" "}
              <code className="rounded bg-black/50 px-1 py-0.5 text-xs">powershell.exe</code> on the laptop
              hosting this dashboard. Without <span className="font-mono">pywinpty</span> installed there,
              the backend falls back to a line-buffered pipe — ordinary commands work, full-screen
              programs (vim, htop-alikes) do not.
            </p>
            {connected || phase === "connecting"
              ? <button className="btn-danger" onClick={disconnect}>Disconnect</button>
              : <button className="btn-primary" onClick={connect} disabled={!canConnect}>Open PowerShell</button>}
          </div>
        )}

        {/* Only Things that carry an `ip` attribute can be quick-connected. The
            section used to render on `things.length` alone, so a registry full
            of Things provisioned without an address produced an empty
            "Quick connect" heading and nothing under it — which reads as a
            broken panel rather than as missing data. */}
        {mode === "ssh" && quickConnect.length > 0 && (
          <div className="mt-3">
            <div className="lbl mb-1">Quick connect · from Devices</div>
            <div className="flex flex-wrap gap-1.5">
              {quickConnect.map(({ name, ip }) => (
                <button key={name} className="chip" onClick={() => setHost(ip)}>
                  {name} · {ip}
                </button>
              ))}
            </div>
          </div>
        )}
        {mode === "ssh" && things.length > 0 && quickConnect.length === 0 && (
          <div className="mt-3 text-xs text-slate-500">
            {things.length} Thing{things.length > 1 ? "s" : ""} registered, none
            carrying an <span className="font-mono">ip</span> attribute — nothing
            to quick-connect to. Enter the address above.
          </div>
        )}
      </Card>

      <div className="card overflow-hidden p-0">
        <div ref={containerRef} style={{ height: "60vh", padding: 12 }} />
        <div className="flex flex-wrap items-center gap-x-5 gap-y-1 border-t border-white/5 bg-black/40 px-4 py-2 font-mono text-[11px] text-slate-400">
          <span>state <span className="text-slate-200">{phase}</span></span>
          <span>in <span className="text-slate-200">{stats ? stats.bytesIn : "--"}</span> B</span>
          <span>out <span className="text-slate-200">{stats ? stats.bytesOut : "--"}</span> B</span>
          <span>
            last output{" "}
            <span className="text-slate-200">
              {stats?.lastDataAt ? `${agoText(Date.now() - stats.lastDataAt)} ago` : "--"}
            </span>
          </span>
          <span>
            open for{" "}
            <span className="text-slate-200">
              {stats?.openedAt ? agoText((stats.closedAt ?? Date.now()) - stats.openedAt) : "--"}
            </span>
          </span>
          <span>
            close{" "}
            <span className="text-slate-200">
              {stats?.closeCode !== null && stats?.closeCode !== undefined ? stats.closeCode : "--"}
            </span>
          </span>
        </div>
      </div>

      {diag && (
        <div
          className={`rounded-lg border p-4 text-sm ${
            diag.tone === "ok"
              ? "border-slate-500/25 bg-black/30 text-slate-300"
              : diag.tone === "warn"
                ? "border-amber-500/30 bg-amber-500/5 text-amber-100"
                : "border-rose-500/30 bg-rose-500/5 text-rose-100"
          }`}
        >
          <div className="font-semibold">{diag.title}</div>
          <ul className="mt-2 space-y-1 text-xs leading-relaxed opacity-90">
            {diag.lines.map((l, i) => <li key={i}>· {l}</li>)}
          </ul>
          {stats?.head.trim() && (
            <>
              <div className="lbl mt-3 mb-1">What the server sent</div>
              <pre className="max-h-40 overflow-auto rounded bg-black/60 p-2 font-mono text-[10px] leading-relaxed text-slate-300">
{stripAnsi(stats.head).trim()}
              </pre>
            </>
          )}
        </div>
      )}

      <div className="rounded-lg border border-amber-500/20 bg-amber-500/5 p-3 text-xs leading-relaxed text-amber-200">
        <div className="font-semibold">What crosses the wire</div>
        Keystrokes and shell output are proxied by the FPMS backend and go nowhere else.
        Two things worth knowing before you use this from anywhere but the laptop:
        <ul className="mt-1 space-y-0.5 pl-4">
          <li>
            · <b>No secret is ever placed in the WebSocket URL.</b> The backend will accept a{" "}
            <span className="font-mono">password=</span> query parameter and this page used to send
            one; URLs are logged by every proxy on the path and kept in browser history, so that has
            been removed. Only host, port, username and a key <i>path</i> travel in the URL now.
          </li>
          <li>
            · <span className="font-mono">/ws/term/*</span> is guarded by the safe-mode middleware
            (which refuses public visitors), <b>not</b> by the password gate — that one is HTTP-only.
            Treat LAN access to this port as equivalent to a shell.
          </li>
        </ul>
      </div>
    </div>
  );
}

// ---- diagnosis ------------------------------------------------------------

type Blocker = { hard: boolean; title: string; detail: string; fix?: string };

/** Reasons a session cannot (or may not) start, known before pressing Connect. */
function preflight(
  mode: Mode,
  auth: AuthStatus | null,
  authErr: string | null,
  isLan: boolean,
  keyPath: string,
): Blocker[] {
  const out: Blocker[] = [];

  if (authErr) {
    out.push({
      hard: false,
      title: "cannot read /api/auth-status",
      detail: `The page could not ask the backend whether terminals are allowed (${authErr}). ` +
              "That usually means the backend is down or this page is being served by something else.",
      fix: "Reload once the FPMS backend is running.",
    });
    return out;
  }
  if (!auth) return out; // still loading — say nothing rather than guess

  if (auth.controls_disabled) {
    out.push({
      hard: true,
      title: auth.role === "cloud" ? "this is the cloud deployment" : "safe mode, public visitor",
      detail:
        auth.role === "cloud"
          ? "The cloud role refuses every /ws/term/* connection outright — there is no laptop " +
            "behind it to open a shell on. The socket would close immediately with code 1008."
          : "Safe mode is on and this request arrived over the public link, so the backend refuses " +
            "/ws/term/* with close code 1008. Rover telemetry stays available; machine control does not.",
      fix: "Open the dashboard on the HQ laptop itself (http://127.0.0.1:8010 or its LAN address).",
    });
  }

  if (auth.auth_required && !auth.authenticated) {
    out.push({
      hard: false,
      title: "not signed in",
      detail:
        "A password is configured and this browser has no valid session cookie. HTTP calls will " +
        "return 401. The terminal WebSocket itself is NOT password-gated (the password middleware " +
        "is HTTP-only), so it may still open — but the rest of the page will be empty.",
      fix: "Sign in on the login screen so the whole dashboard works.",
    });
  }

  if (mode === "ssh" && !keyPath.trim()) {
    out.push({
      hard: false,
      title: "no private key named",
      detail:
        "The backend connects with look_for_keys=False and allow_agent=False, so with no key path " +
        "it has no credential at all and the target will refuse authentication. This page does not " +
        "offer password auth — " + PASSWORD_AUTH_NOTE.charAt(0).toLowerCase() + PASSWORD_AUTH_NOTE.slice(1),
      fix: "Give the full path to a passphrase-less private key that exists on the FPMS laptop.",
    });
  }

  if (mode === "local" && !isLan) {
    out.push({
      hard: true,
      title: "local shell is LAN-only",
      detail:
        `The backend serves /ws/term/local only when the client address is loopback, RFC1918 or ` +
        `link-local. Yours is ${auth.client_host ?? "unknown"}. It will accept the socket, write ` +
        "one refusal line, and close.",
      fix: "Use SSH mode, or open the dashboard from the laptop / the same network.",
    });
  }

  return out;
}

type Diagnosis = { tone: "ok" | "warn" | "bad"; title: string; lines: string[] };

/**
 * Turn the instrumented socket into a cause. Ordered most specific first —
 * "never opened" and "opened but silent" are completely different faults with
 * completely different fixes, and both used to render as an empty screen.
 */
function diagnose(s: SessionStats, auth: AuthStatus | null, isLan: boolean): Diagnosis {
  const code = s.closeCode;
  const codeText = code === null ? "--" : `${code}${CLOSE_CODES[code] ? ` (${CLOSE_CODES[code]})` : ""}`;
  const target = s.mode === "ssh" ? "the SSH proxy" : "the local shell";

  if (code === 1008) {
    return {
      tone: "bad",
      title: "The backend refused the terminal.",
      lines: [
        "Close code 1008 comes from one place only: the safe-mode guard in backend/auth.py, " +
        "which blocks every /ws/term/* path for a cloud-role backend or a visitor arriving over " +
        "the public tunnel.",
        auth?.role ? `This backend reports role "${auth.role}", safe_mode=${String(auth.safe_mode)}, controls_disabled=${String(auth.controls_disabled)}.` : "Press Re-check above to read the current gate.",
        "Fix: use the dashboard on the HQ laptop. Nothing about the credentials you typed is at fault.",
      ],
    };
  }

  if (s.openedAt === null) {
    return {
      tone: "bad",
      title: "The WebSocket never opened.",
      lines: [
        `The HTTP upgrade to ${s.url} did not complete, so ${target} was never reached.`,
        `Close: ${codeText}.`,
        "Most likely: the FPMS backend is not running, or whatever is serving this page does not " +
        "proxy WebSocket upgrades (a plain static file server, or a dev proxy without ws:true).",
        "Check: the Preflight card above should be able to read /api/auth-status. If that also " +
        "fails, the backend is down.",
      ],
    };
  }

  if (s.bytesIn === 0) {
    return {
      tone: "bad",
      title: "Connected, then the server closed without sending a single byte.",
      lines: [
        `The socket was open for ${agoText((s.closedAt ?? Date.now()) - s.openedAt)} and 0 bytes arrived. ` +
        `Close: ${codeText}.`,
        s.mode === "ssh"
          ? "backend/terminal.py imports paramiko INSIDE run_ssh_session but OUTSIDE its try/except — " +
            "so if paramiko is missing from the backend environment, the handler raises before it can " +
            "report anything and you get exactly this: an accepted socket, zero output. Every other SSH " +
            "failure (bad host, refused auth, timeout) sends a red 'SSH connect failed:' line first."
          : "The local handler writes a banner as its first action, so zero bytes means it raised before " +
            "that — PowerShell could not be spawned, or the backend is not on Windows (it refuses with a " +
            "message in that case, so a silent close points at the spawn itself).",
        "Check the backend's log (launch.log / service.log) for the traceback — this failure mode leaves " +
        "one there and nothing on the wire.",
      ],
    };
  }

  const looksLikeSshFailure = /SSH connect failed/i.test(s.head);
  if (looksLikeSshFailure) {
    return {
      tone: "warn",
      title: "The backend reached the SSH stage and the target refused.",
      lines: [
        "This is paramiko's own error, quoted below — host unreachable, wrong port, key rejected, " +
        "or the host key policy. The dashboard and the WebSocket both worked.",
        `Close: ${codeText}.`,
        "The backend connects with look_for_keys=False, allow_agent=False and a 6s timeout, so no key " +
        "on the laptop is tried unless you named its path — and an 'Authentication failed' with no " +
        "key path given means exactly that.",
      ],
    };
  }

  if (s.mode === "local" && /not on the LAN/i.test(s.head)) {
    return {
      tone: "bad",
      title: "The local shell was refused: you are not on the LAN.",
      lines: [
        `The backend saw your address as ${auth?.client_host ?? "an address it does not consider local"} ` +
        `(is_lan=${String(isLan)}) and served the refusal instead of a shell.`,
        "Use SSH mode, or reach the dashboard from the laptop itself or the same network.",
      ],
    };
  }

  const shortLived = s.openedAt !== null && (s.closedAt ?? Date.now()) - s.openedAt < 2000;
  return {
    tone: shortLived ? "warn" : "ok",
    title: shortLived
      ? "The session ended almost immediately."
      : "Session ended.",
    lines: [
      `${s.bytesIn} bytes received, ${s.bytesOut} sent, open for ${agoText((s.closedAt ?? Date.now()) - (s.openedAt ?? Date.now()))}.`,
      `Close: ${codeText}.`,
      shortLived
        ? "The server said something (quoted below) and hung up. Read it — it is the actual reason."
        : "Normal end of a shell: you exited, the socket dropped, or the backend restarted. Press Connect to start another.",
    ],
  };
}

// ---- small helpers --------------------------------------------------------

/** Elapsed milliseconds, readable. Never renders a bare 0. */
function agoText(ms: number): string {
  if (!Number.isFinite(ms)) return "--";
  const s = Math.max(0, ms) / 1000;
  if (s < 10) return `${s.toFixed(1)}s`;
  if (s < 60) return `${Math.round(s)}s`;
  if (s < 3600) return `${Math.floor(s / 60)}m ${Math.round(s % 60)}s`;
  return `${Math.floor(s / 3600)}h ${Math.floor((s % 3600) / 60)}m`;
}

/** The diagnosis quotes server output as plain text, so strip the colours. */
function stripAnsi(s: string): string {
  // eslint-disable-next-line no-control-regex
  return s.replace(/\x1b\[[0-9;]*m/g, "");
}

function Fact({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-lg border border-white/5 bg-black/30 px-3 py-2">
      <div className="lbl">{label}</div>
      <div className="mt-0.5 font-mono text-xs text-slate-200">{value}</div>
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
