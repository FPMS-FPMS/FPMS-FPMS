import { ReactNode } from "react";

/**
 * The console's primary surface. ~47 call sites, and only three prop shapes
 * are ever used (bare, `glow`, `className`) — the API is deliberately narrow
 * so the cards stay visually identical to each other.
 */
export function Card({
  children,
  className = "",
  glow = false,
}: {
  children: ReactNode;
  className?: string;
  glow?: boolean;
}) {
  return (
    <section className={`${glow ? "card-glow" : "card"} p-5 ${className}`}>
      {children}
    </section>
  );
}

/**
 * Card header.
 *
 * NOTE THE ORDER: `subtitle` renders ABOVE `title`. It is an eyebrow/kicker,
 * not a description, and 45 call sites are written on that assumption — do
 * not "fix" it by swapping them.
 */
export function CardHeader({
  title,
  subtitle,
  right,
}: {
  title: string;
  subtitle?: string;
  right?: ReactNode;
}) {
  return (
    <div className="mb-4 flex items-start justify-between gap-4 border-b border-white/[0.06] pb-3">
      <div className="min-w-0">
        {subtitle ? (
          <div className="truncate text-2xs font-medium uppercase tracking-[0.16em] text-slate-400">
            {subtitle}
          </div>
        ) : null}
        <h2 className="mt-1 text-lg font-semibold leading-tight tracking-tight text-slate-50">
          {title}
        </h2>
      </div>
      {right ? <div className="shrink-0">{right}</div> : null}
    </div>
  );
}
