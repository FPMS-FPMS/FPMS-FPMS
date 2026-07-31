import { Component, type ErrorInfo, type ReactNode } from "react";

type Props = {
  children: ReactNode;
  /** Shown in the panel header so an operator knows which tile died. */
  label?: string;
};

type State = {
  error: Error | null;
};

/**
 * Contains a render crash to one subtree.
 *
 * This exists because of a specific, recorded failure: the LiDAR page called
 * `pose.data.data.x_m.toFixed()` on a rover with no odometry. React has no
 * default recovery for a throw during render — it unmounts the entire tree —
 * so a missing telemetry field on one card blanked the whole dashboard to
 * white with nothing in the UI to say why.
 *
 * The field guards in arena.readPose() are the real fix. This is the seatbelt:
 * whatever the next unguarded field turns out to be, the operator gets a red
 * panel naming the failure and keeps the rest of the fleet on screen.
 */
export class ErrorBoundary extends Component<Props, State> {
  state: State = { error: null };

  static getDerivedStateFromError(error: Error): State {
    return { error };
  }

  componentDidCatch(error: Error, info: ErrorInfo): void {
    // Console is the only sink available here; the dashboard has no telemetry
    // upload path for frontend faults and a LAN tool does not need one.
    console.error(
      `[ErrorBoundary${this.props.label ? ` ${this.props.label}` : ""}]`,
      error,
      info.componentStack,
    );
  }

  private retry = (): void => {
    this.setState({ error: null });
  };

  render(): ReactNode {
    const { error } = this.state;
    if (!error) return this.props.children;

    return (
      <div className="rounded-xl border border-rose-500/40 bg-rose-950/30 p-4">
        <div className="text-xs uppercase tracking-widest text-rose-400">
          Render fault{this.props.label ? ` — ${this.props.label}` : ""}
        </div>
        <pre className="mt-2 max-h-40 overflow-auto whitespace-pre-wrap break-words font-mono text-xs leading-relaxed text-rose-200">
          {error.message || String(error)}
        </pre>
        <p className="mt-2 text-xs text-rose-300/70">
          This panel failed, the rest of the dashboard is unaffected. Details are in the
          browser console.
        </p>
        <button className="btn-danger mt-3" onClick={this.retry}>
          Retry
        </button>
      </div>
    );
  }
}

export default ErrorBoundary;
