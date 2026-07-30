import { ReactNode } from "react";

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
    <div className="mb-4 flex items-start justify-between gap-4">
      <div>
        <div className="text-xs uppercase tracking-widest text-slate-500">{subtitle}</div>
        <div className="mt-0.5 text-lg font-semibold tracking-tight text-slate-100">{title}</div>
      </div>
      {right}
    </div>
  );
}
