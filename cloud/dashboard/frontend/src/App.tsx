import { useEffect, useState } from "react";
import { Route, Routes } from "react-router-dom";
import Layout from "./components/Layout";
import Intro from "./pages/Intro";
import Control from "./pages/Control";
import Drive from "./pages/Drive";
import Devices from "./pages/Devices";
import Aws from "./pages/Aws";
import Lidar from "./pages/Lidar";
import Camera from "./pages/Camera";
import Thermal from "./pages/Thermal";
import Terminal from "./pages/Terminal";
import Analyst from "./pages/Analyst";
import Install from "./pages/Install";
import Login from "./pages/Login";

type AuthStatus = {
  auth_required: boolean;
  authenticated: boolean;
  is_lan: boolean;
  client_host: string | null;
  is_remote?: boolean;
  safe_mode?: boolean;
  /** Safe mode is on AND this visitor arrived over the public link. */
  controls_disabled?: boolean;
};

/** Shown instead of an HQ-only page when someone reaches it over the tunnel. */
function NotAvailableRemotely({ what }: { what: string }) {
  return (
    <div className="rounded-xl border border-white/10 bg-black/30 p-8 text-center">
      <div className="text-lg font-semibold text-slate-100">{what} is HQ-only</div>
      <p className="mx-auto mt-2 max-w-md text-sm text-slate-400">
        This page can act on the HQ laptop, so it's disabled over the public link.
        Open the FPMS app on that laptop to use it. Live rover data stays available here.
      </p>
    </div>
  );
}

export default function App() {
  const [status, setStatus] = useState<AuthStatus | null>(null);

  const refresh = () =>
    fetch("/api/auth-status", { credentials: "include" })
      .then((r) => r.json())
      .then(setStatus)
      .catch(() => setStatus({ auth_required: false, authenticated: true, is_lan: true, client_host: null }));

  useEffect(() => { refresh(); }, []);

  if (!status) {
    return (
      <div className="flex min-h-screen items-center justify-center text-slate-500">
        Loading…
      </div>
    );
  }

  if (status.auth_required && !status.authenticated) {
    return <Login onAuthed={refresh} />;
  }

  // Hiding the tab isn't enough — someone can still type the URL. The server
  // refuses these routes regardless; this just avoids a broken-looking page.
  const locked = !!status.controls_disabled;

  return (
    <Layout controlsDisabled={locked}>
      <Routes>
        <Route path="/" element={<Intro />} />
        <Route
          path="/control"
          element={locked ? <NotAvailableRemotely what="Control" /> : <Control />}
        />
        <Route
          path="/drive"
          element={locked ? <NotAvailableRemotely what="Drive" /> : <Drive />}
        />
        <Route
          path="/devices"
          element={locked ? <NotAvailableRemotely what="Devices" /> : <Devices />}
        />
        <Route path="/aws" element={<Aws />} />
        <Route path="/lidar" element={<Lidar />} />
        <Route path="/camera" element={<Camera />} />
        <Route path="/thermal" element={<Thermal />} />
        <Route path="/analyst" element={<Analyst />} />
        <Route
          path="/terminal"
          element={locked ? <NotAvailableRemotely what="Terminal" /> : <Terminal isLan={status.is_lan} />}
        />
        <Route path="/install" element={<Install />} />
      </Routes>
    </Layout>
  );
}
