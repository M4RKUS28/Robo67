import clsx from "clsx";
import { ChevronRight } from "lucide-react";
import { PHASE_ORDER, phaseMeta } from "../lib/phases";

interface Props {
  current: string;
  error?: boolean;
}

export function PhaseTimeline({ current, error }: Props) {
  const order = PHASE_ORDER as readonly string[];
  const curIdx = order.indexOf(current);
  const meta = phaseMeta(error ? "ERROR" : current);

  return (
    <div className="panel panel-pad">
      <div className="mb-3 flex items-center justify-between">
        <span className="label">Insertion state machine</span>
        <span className="text-xs text-slate-500">{order.length} phases</span>
      </div>

      <div className="flex flex-wrap items-center gap-1.5">
        {order.map((p, i) => {
          const m = phaseMeta(p);
          const done = curIdx > i && curIdx >= 0;
          const active = p === current;
          return (
            <div key={p} className="flex items-center gap-1.5">
              <div
                className={clsx(
                  "flex items-center gap-1.5 rounded-md border px-2 py-1 text-xs transition-colors",
                  active
                    ? "border-transparent font-semibold text-ink-950"
                    : done
                      ? "border-ink-600 text-slate-300"
                      : "border-ink-700 text-slate-500",
                )}
                style={
                  active
                    ? { background: m.color, boxShadow: `0 0 12px ${m.color}55` }
                    : done
                      ? { borderColor: `${m.color}66` }
                      : undefined
                }
              >
                <span
                  className={clsx("h-1.5 w-1.5 rounded-full", active && "animate-pulsesoft")}
                  style={{ background: active ? "#0b0f17" : m.color, opacity: done || active ? 1 : 0.5 }}
                />
                {m.label}
              </div>
              {i < order.length - 1 && <ChevronRight size={13} className="text-ink-600" />}
            </div>
          );
        })}
      </div>

      <div className="mt-3 flex items-start gap-2 rounded-lg bg-ink-900/60 px-3 py-2">
        <span className="mt-0.5 h-2.5 w-2.5 shrink-0 rounded-full" style={{ background: meta.color }} />
        <div>
          <div className="text-sm font-medium" style={{ color: meta.color }}>
            {meta.label}
          </div>
          <div className="text-xs text-slate-400">{meta.blurb}</div>
        </div>
      </div>
    </div>
  );
}
