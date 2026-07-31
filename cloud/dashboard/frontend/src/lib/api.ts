export async function apiGet<T>(path: string): Promise<T> {
  const r = await fetch(path);
  if (!r.ok) throw new Error(`${r.status} ${r.statusText}`);
  return r.json();
}

export async function apiPost<T>(path: string): Promise<T> {
  const r = await fetch(path, { method: "POST" });
  if (!r.ok) throw new Error(`${r.status} ${r.statusText}`);
  return r.json();
}

/**
 * POST a JSON body. Pages used to inline this fetch with its three easy-to-
 * forget details — the Content-Type header, `credentials: "include"` so the
 * session cookie rides along, and an explicit !ok check (fetch resolves happily
 * on a 500, so without it a failed command looks like a successful one).
 */
export async function apiPostJson<T>(path: string, body?: unknown): Promise<T> {
  const r = await fetch(path, {
    method: "POST",
    credentials: "include",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body ?? {}),
  });
  if (!r.ok) throw new Error(`${r.status} ${r.statusText}`);
  return r.json();
}
