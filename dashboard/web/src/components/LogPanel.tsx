import { useEffect, useRef } from "react";
import clsx from "clsx";
import { Loader2, CheckCircle2, AlertTriangle, CircleSlash } from "lucide-react";

// A scrollable, monospace viewer for a managed subprocess's stdout. Auto-sticks
// to the bottom while running so the freshest line stays in view. Works for any
// of the dashboard's managed runs (insertion / arm relaunch / home).
export function LogPanel({
  title,
  subtitle,
  enabled = false,
  running = false,
  elapsedS = null,
  outcome,
  log = [],
  height = 300,
}: {
  title: string;
  subtitle?: string;
  enabled?: boolean;
  running?: boolean;
  elapsedS?: number | null;
  outcome?: { ok: boolean; label?: string };
  log?: string[];
  height?: number;
}) {
  const boxRef = useRef<HTMLDivElement>(null);

  // Stick to the bottom as new lines stream in.
  useEffect(() => {
    const el = boxRef.current;
    if (el) el.scrollTop = el.scrollHeight;
  }, [log.length]);

  let stateChip: React.ReactNode = (
    <span className="chip bg-slate-500/15 text-slate-400">
      <CircleSlash size={12} /> idle
    </span>
  );
  if (!enabled) {
    stateChip = (
      <span className="chip bg-slate-500/15 text-slate-400" title="live mode only">
        live-only
      </span>
    );
  } else if (running) {
    stateChip = (
      <span className="chip bg-amber-500/15 text-amber-300">
        <Loader2 size={12} className="animate-spin" /> running
        {elapsedS != null ? ` · ${elapsedS.toFixed(0)}s` : ""}
      </span>
    );
  } else if (outcome) {
    stateChip = outcome.ok ? (
      <span className="chip bg-emerald-500/15 text-emerald-300">
        <CheckCircle2 size={12} /> {outcome.label ?? "done"}
      </span>
    ) : (
      <span className="chip bg-red-500/15 text-red-300">
        <AlertTriangle size={12} /> {outcome.label ?? "failed"}
      </span>
    );
  }

  return (
    <div className="panel overflow-hidden">
      <div className="flex items-center justify-between border-b border-ink-700/70 px-3 py-2">
        <div className="flex items-baseline gap-2">
          <span className="label">{title}</span>
          {subtitle && <span className="hidden text-xs text-slate-500 sm:inline">{subtitle}</span>}
        </div>
        <div className="flex items-center gap-2">
          {stateChip}
          <span className="text-xs text-slate-500">{log.length} lines</span>
        </div>
      </div>
      <div
        ref={boxRef}
        className="overflow-auto bg-ink-950/40 px-3 py-2 font-mono text-[11px] leading-relaxed"
        style={{ maxHeight: height }}
      >
        {log.length === 0 ? (
          <div className="py-6 text-center text-xs text-slate-500">
            {enabled ? "no output yet — run it to see logs here" : "live mode only"}
          </div>
        ) : (
          log.map((line, i) => (
            <div
              key={i}
              className={clsx(
                "whitespace-pre-wrap break-words",
                /error|fail|abort|reflex|warn/i.test(line)
                  ? "text-amber-300/90"
                  : line.startsWith("[dashboard]") || /\[\d\d:\d\d:\d\d\]/.test(line)
                    ? "text-sky-300/80"
                    : "text-slate-300",
              )}
            >
              {line}
            </div>
          ))
        )}
      </div>
    </div>
  );
}
