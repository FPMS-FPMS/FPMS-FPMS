import { useEffect, useRef, useState } from "react";

export type ChannelState<T> = {
  data: T | null;
  connected: boolean;
  lastAt: number | null;
  messages: number;
};

/**
 * Subscribes to /ws/{channel} and returns the latest envelope. Auto-reconnects
 * with a modest backoff. Not a general-purpose Rx solution — just enough for a
 * live dashboard where dropped frames don't matter.
 */
export function useChannel<T = any>(channel: string | null): ChannelState<T> {
  const [state, setState] = useState<ChannelState<T>>({
    data: null,
    connected: false,
    lastAt: null,
    messages: 0,
  });
  const wsRef = useRef<WebSocket | null>(null);
  const attemptRef = useRef(0);
  const timerRef = useRef<number | null>(null);

  useEffect(() => {
    if (!channel) return;
    let cancelled = false;

    const connect = () => {
      if (cancelled) return;
      const proto = window.location.protocol === "https:" ? "wss" : "ws";
      const url = `${proto}://${window.location.host}/ws/${encodeURIComponent(channel)}`;
      const ws = new WebSocket(url);
      wsRef.current = ws;

      ws.onopen = () => {
        attemptRef.current = 0;
        setState((s) => ({ ...s, connected: true }));
      };
      ws.onmessage = (ev) => {
        try {
          const parsed = JSON.parse(ev.data);
          setState((s) => ({
            data: parsed,
            connected: true,
            lastAt: Date.now(),
            messages: s.messages + 1,
          }));
        } catch {
          /* ignore */
        }
      };
      ws.onclose = () => {
        setState((s) => ({ ...s, connected: false }));
        if (cancelled) return;
        attemptRef.current += 1;
        const delay = Math.min(5000, 500 * attemptRef.current);
        timerRef.current = window.setTimeout(connect, delay);
      };
      ws.onerror = () => ws.close();
    };

    connect();
    return () => {
      cancelled = true;
      if (timerRef.current) window.clearTimeout(timerRef.current);
      wsRef.current?.close();
    };
  }, [channel]);

  return state;
}
