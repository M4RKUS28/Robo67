import clsx from "clsx";
import type { ReactNode } from "react";

interface Props {
  label: string;
  value: ReactNode;
  unit?: string;
  hint?: ReactNode;
  accent?: string;
  alarm?: boolean;
}

export function StatTile({ label, value, unit, hint, accent, alarm }: Props) {
  return (
    <div
      className={clsx(
        "panel panel-pad flex flex-col justify-between",
        alarm && "ring-1 ring-red-500/60",
      )}
    >
      <div className="flex items-center justify-between">
        <span className="label">{label}</span>
        {accent && <span className="h-2 w-2 rounded-full" style={{ background: accent }} />}
      </div>
      <div className="mt-1 flex items-baseline gap-1">
        <span className={clsx("value-xl", alarm && "text-red-400")}>{value}</span>
        {unit && <span className="text-sm text-slate-500">{unit}</span>}
      </div>
      {hint && <div className="mt-1 text-xs text-slate-500">{hint}</div>}
    </div>
  );
}
