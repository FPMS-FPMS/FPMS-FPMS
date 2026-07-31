import { useEffect, useRef, useState, type MutableRefObject } from "react";

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

export type ChannelMeta = {
  connected: boolean;
  lastAt: number | null;
  messages: number;
};

export type ChannelRef<T> = {
  /** Latest parsed envelope, or null before the first message. */
  dataRef: MutableRefObject<T | null>;
  /** Monotonic counter, bumped once per message. Poll it to detect new data. */
  seqRef: MutableRefObject<number>;
  /** Connection bookkeeping, also outside React state. */
  metaRef: MutableRefObject<ChannelMeta>;
};

/**
 * Render-free sibling of useChannel: same URL, same reconnect backoff, but the
 * socket writes into refs and never calls setState.
 *
 * useChannel is right for text and status chrome — a re-render per message is
 * what makes those update. It is wrong for a canvas: a 2 Hz LiDAR stream
 * driving React means the whole card subtree reconciles twice a second purely
 * to hand a Float32Array to a draw call that a requestAnimationFrame loop was
 * going to make anyway. Consumers here mount once, poll `seqRef.current` from
 * their rAF loop, and recompute only when it changes.
 *
 * Deliberately a separate hook rather than an option on useChannel — several
 * other pages depend on that hook's exact re-render behaviour.
 */
export function useChannelRef<T = any>(channel: string | null): ChannelRef<T> {
  const dataRef = useRef<T | null>(null);
  const seqRef = useRef(0);
  const metaRef = useRef<ChannelMeta>({ connected: false, lastAt: null, messages: 0 });
  const wsRef = useRef<WebSocket | null>(null);
  const attemptRef = useRef(0);
  const timerRef = useRef<number | null>(null);

  // The returned object is itself held in a ref. Consumers put it in effect
  // dependency arrays; a fresh literal per render would tear down and rebuild
  // their animation loop on every unrelated re-render.
  const handleRef = useRef<ChannelRef<T> | null>(null);
  if (handleRef.current === null) handleRef.current = { dataRef, seqRef, metaRef };

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
        metaRef.current.connected = true;
      };
      ws.onmessage = (ev) => {
        try {
          dataRef.current = JSON.parse(ev.data) as T;
        } catch {
          return; // keep the previous frame rather than blanking the view
        }
        const meta = metaRef.current;
        meta.connected = true;
        meta.lastAt = Date.now();
        meta.messages += 1;
        seqRef.current += 1;
      };
      ws.onclose = () => {
        metaRef.current.connected = false;
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

  return handleRef.current;
}
