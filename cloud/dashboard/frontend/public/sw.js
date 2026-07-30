// FPMS PWA service worker.
// Cache-first for the app shell (HTML/CSS/JS), network-only for /api and /ws.
// Kept intentionally small — this is a live-data app, not an offline one.

const SHELL_CACHE = "fpms-shell-v1";
const SHELL = [
  "/",
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
  self.skipWaiting();
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(
        keys.filter((k) => k !== SHELL_CACHE).map((k) => caches.delete(k))
      )
    )
  );
  self.clients.claim();
});

self.addEventListener("fetch", (event) => {
  const url = new URL(event.request.url);

  // Never touch API/WebSocket — always live.
  if (url.pathname.startsWith("/api/") || url.pathname.startsWith("/ws/")) {
    return;
  }

  if (event.request.method !== "GET") return;

  event.respondWith(
    (async () => {
      const cached = await caches.match(event.request);
      if (cached) return cached;
      try {
        const resp = await fetch(event.request);
        if (resp && resp.status === 200 && resp.type === "basic") {
          const clone = resp.clone();
          caches.open(SHELL_CACHE).then((c) => c.put(event.request, clone));
        }
        return resp;
      } catch {
        // Offline fallback for navigation → serve the SPA shell.
        if (event.request.mode === "navigate") {
          return caches.match("/");
        }
        return new Response("", { status: 504 });
      }
    })()
  );
});
