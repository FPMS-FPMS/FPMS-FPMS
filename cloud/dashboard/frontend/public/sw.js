// FPMS PWA service worker.
//
// -----------------------------------------------------------------------------
// WHY THIS IS NETWORK-FIRST FOR HTML, AND WHY THAT IS NOT A PREFERENCE
// -----------------------------------------------------------------------------
// The previous version was cache-first for EVERYTHING that was not /api or /ws,
// and it precached "/". Navigations therefore resolved out of the cache, which
// means the app shell the operator got was whatever index.html happened to be
// cached the first time they ever opened the app.
//
// index.html is the one file that is NOT content-hashed — it is the file that
// NAMES the hashed bundles. So a stale index.html pins the whole application to
// a stale build permanently. Rebuilding, reinstalling, even replacing the files
// on disk changed nothing: the browser never asked for the new one, and the old
// cache name (fpms-shell-v1) never changed, so `activate` never cleared it.
//
// That failure is invisible from the server side. Every check that fetches the
// page WITHOUT a service worker — curl, Invoke-WebRequest, a fresh incognito
// window — sees the new build and reports success, while the operator's actual
// window keeps rendering the old one. A whole tab was shipped, verified, and
// still absent from the only screen that mattered.
//
// So:
//   NAVIGATIONS AND HTML  -> network first, cache only as an offline fallback.
//                            A live-data console that cannot reach its backend
//                            is useless anyway; there is nothing to protect by
//                            serving it a stale shell.
//   HASHED BUILD ASSETS   -> cache first. Their filenames change on every build,
//                            so a cached one can never be the wrong one.
//   ICONS / MANIFEST      -> cache first. They are not versioned and not
//                            load-bearing.
//   /api and /ws          -> never touched. Always live.
//
// Kept intentionally small — this is a live-data app, not an offline one.

// Bumping this name is what evicts the poisoned v1 cache: `activate` deletes
// every cache that is not the current one. Bump it whenever the strategy below
// changes.
const SHELL_CACHE = "fpms-shell-v3";

// "/" is deliberately NOT precached. Precaching the shell is exactly what
// pinned the old build.
const SHELL = [
  "/manifest.webmanifest",
  "/favicon.svg",
  "/pwa-192.png",
  "/pwa-512.png",
  "/apple-touch-icon.png",
];

self.addEventListener("install", (event) => {
  event.waitUntil(
    caches.open(SHELL_CACHE).then((c) => c.addAll(SHELL)).catch(() => {})
  );
  // Take over immediately rather than waiting for every tab to close. An
  // operator mid-run should not have to know that closing the window is what
  // applies a fix.
  self.skipWaiting();
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches
      .keys()
      .then((keys) =>
        Promise.all(
          keys.filter((k) => k !== SHELL_CACHE).map((k) => caches.delete(k))
        )
      )
      .then(() => self.clients.claim())
  );
});

/** Content-hashed build output: /assets/index-a1b2c3d4.js and friends. */
function isHashedAsset(url) {
  return /^\/assets\/.+-[A-Za-z0-9_-]{8,}\.(js|css|woff2?|svg|png|jpg|webp)$/.test(
    url.pathname
  );
}

self.addEventListener("fetch", (event) => {
  const url = new URL(event.request.url);

  // Never touch API/WebSocket — always live.
  if (url.pathname.startsWith("/api/") || url.pathname.startsWith("/ws/")) {
    return;
  }
  if (event.request.method !== "GET") return;
  // Cross-origin requests are none of this worker's business.
  if (url.origin !== self.location.origin) return;

  const isNavigation =
    event.request.mode === "navigate" ||
    (event.request.headers.get("accept") || "").includes("text/html");

  if (isNavigation) {
    // ---- network first -----------------------------------------------------
    event.respondWith(
      (async () => {
        try {
          const resp = await fetch(event.request);
          if (resp && resp.status === 200 && resp.type === "basic") {
            const clone = resp.clone();
            caches.open(SHELL_CACHE).then((c) => c.put("/", clone));
          }
          return resp;
        } catch {
          // Offline: the last shell we actually saw beats a browser error page,
          // and the app's own panels will report their feeds as dead.
          const cached = await caches.match("/");
          if (cached) return cached;
          return new Response(
            "<h1>FPMS is offline</h1><p>The dashboard backend is not reachable from this browser.</p>",
            { status: 503, headers: { "Content-Type": "text/html" } }
          );
        }
      })()
    );
    return;
  }

  // ---- cache first, for things whose identity is in their name -------------
  event.respondWith(
    (async () => {
      const cached = await caches.match(event.request);
      if (cached) return cached;
      try {
        const resp = await fetch(event.request);
        if (
          resp &&
          resp.status === 200 &&
          resp.type === "basic" &&
          (isHashedAsset(url) || SHELL.includes(url.pathname))
        ) {
          const clone = resp.clone();
          caches.open(SHELL_CACHE).then((c) => c.put(event.request, clone));
        }
        return resp;
      } catch {
        return new Response("", { status: 504 });
      }
    })()
  );
});
