import { useEffect, useState } from "react";
import { Link, Navigate, Route, Routes, useLocation } from "react-router-dom";
import Layout from "./components/Layout";
import { ErrorBoundary } from "./components/ErrorBoundary";
import Intro from "./pages/Intro";
import Control from "./pages/Control";
import Drive from "./pages/Drive";
import Mission from "./pages/Mission";
import Rover2Test from "./pages/Rover2Test";
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

/**
 * Anything that matches no route.
 *
 * This exists because its absence was mistaken for the app losing its tabs.
 * The backend answers 200 for every path so the SPA can own routing, so a typo,
 * an old bookmark, or a link to a page that has since been renamed used to
 * render the normal nav and footer wrapped around a COMPLETELY EMPTY main --
 * which is indistinguishable from "that tab is gone", and was reported as
 * exactly that.
 *
 * It names the path it could not match, so the next person gets a fact instead
 * of a blank rectangle.
 */
function NoSuchPage() {
  const { pathname } = useLocation();
  return (
    <div className="rounded-xl border border-white/10 bg-black/30 p-8 text-center">
      <div className="text-lg font-semibold text-slate-100">No page at this address</div>
      <p className="mx-auto mt-2 max-w-md text-sm text-slate-400">
        Nothing is routed to <span className="font-mono text-slate-300">{pathname}</span>.
        It may have been renamed, or the link may be out of date. Every page the
        app has is in the navigation above.
      </p>
      {/* Points at the panel rather than at "/", which now redirects there
          anyway — a button labelled "Overview" that lands somewhere else is
          its own small lie. */}
      <Link to="/rover2-test" className="btn mt-5 inline-block">
        Back to Rover 2 Mission Test
      </Link>
    </div>
  );
}

export default function App() {
  // MUST be above the early returns below. App returns early while auth
  // status is loading and again for the login screen; a hook called after
  // those would be skipped on that render and React would see the hook
  // order change on the next one.
  const { pathname } = useLocation();
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
      {/*
        EVERY page is wrapped, and the key resets the boundary on navigation.

        Without this, one throw anywhere in a page unmounted the WHOLE React
        tree -- nav, footer and all -- leaving only the Layout's empty grid
        containers behind. Reported as "overview bugs it and goes into a grid",
        with every other tab fine, because the throw was in a component only
        Overview renders. A page-level fault should cost you that page, not the
        application, and it should say what it was.

        Keyed on pathname so navigating away clears a caught error; otherwise
        the boundary latches and every subsequent page looks broken too.
      */}
      <ErrorBoundary key={pathname} label="page">
      <Routes>
        {/*
          THE APP OPENS ON THE THING IT IS OPENED FOR.

          The launcher points a browser at "/", and "/" used to be the Overview
          — a page nobody acts on, one click away from the panel that runs the
          rover. The operator asked for the mission-test tab to be in front of
          them; making it the landing page is the difference between that and a
          tab they still have to find.

          Overview keeps its content at /overview rather than being deleted, and
          the redirect is `replace` so Back does not bounce between the two.

          Public visitors are NOT redirected: /rover2-test is hqOnly and would
          hand them the "HQ-only" notice as the first thing they ever see. They
          get the Overview, which is what that page is for.
        */}
        <Route
          path="/"
          element={locked ? <Intro /> : <Navigate to="/rover2-test" replace />}
        />
        <Route path="/overview" element={<Intro />} />
        <Route
          path="/control"
          element={locked ? <NotAvailableRemotely what="Control" /> : <Control />}
        />
        <Route
          path="/drive"
          element={locked ? <NotAvailableRemotely what="Drive" /> : <Drive />}
        />
        {/* Mission drives the rover, so it is HQ-only exactly like Drive. */}
        <Route path="/mission" element={locked ? <NotAvailableRemotely what="Mission" /> : <Mission />} />
        {/*
          The bare mission-test panel. Same hqOnly rule as Mission — it drives
          the rover — and deliberately a SEPARATE route rather than a mode of
          /mission, so a bookmark or a mis-click cannot land the operator on
          the wrong one of the two while standing beside the arena.
        */}
        <Route
          path="/rover2-test"
          element={locked ? <NotAvailableRemotely what="Rover 2 Mission Test" /> : <Rover2Test />}
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
        {/* Must be LAST: react-router takes the first match. */}
        <Route path="*" element={<NoSuchPage />} />
      </Routes>
      </ErrorBoundary>
    </Layout>
  );
}
