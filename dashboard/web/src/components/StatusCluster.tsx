import clsx from "clsx";
import type { ReactNode } from "react";
import { AlertTriangle, CheckCircle2, Power, PowerOff, Radio } from "lucide-react";
import { useHealth } from "../api/queries";
import { useTelemetry } from "../state/TelemetryProvider";
import { useControls } from "../state/ControlsProvider";
import { phaseMeta } from "../lib/phases";
import { fmtClock } from "../lib/format";

// StatusCluster: the one read-only status area in the header. It consolidates
// everything that used to be scattered as per-control chips -- current phase,
// FCI on/off, arm-ready, the live/offline + sim/live indicators, the clock, and
// a single labelled list of subsystem errors. Actions live in <ControlSidebar/>.

const truncate = (s: string, n = 36) => (s.length > n ? s.slice(0, n) + "…" : s);

// thin vertical rule between logical sub-groups
function Sep() {
  return <span className="hidden h-4 w-px bg-ink-700 sm:inline-block" aria-hidden />;
}

export function StatusCluster() {
  const { connected, latest } = useTelemetry();
  const health = useHealth();
  const { fci, bringup, errors } = useControls();

  const mode = latest?.mode ?? health.data?.mode ?? "—";
  const meta = phaseMeta(latest?.abort ? "ERROR" : latest?.phase);

  // FCI on/off — only when live, idle, and the state is known.
  const f = fci.status;
  const fciChip: ReactNode =
    fci.enabled && !f?.busy && fci.active != null ? (
      <span
        className={clsx(
          "chip",
          fci.active ? "bg-emerald-500/15 text-emerald-300" : "bg-slate-500/15 text-slate-400",
        )}
        title={fci.active ? "FCI active (Desk UI locked out)" : "FCI inactive"}
      >
        {fci.active ? <Power size={12} /> : <PowerOff size={12} />}
        FCI {fci.active ? "on" : "off"}
      </span>
    ) : null;

  // Arm-ready — only after a relaunch has produced a verdict (ok != null).
  const b = bringup.status;
  let armChip: ReactNode = null;
  if (bringup.enabled && !b?.busy && b?.ok != null) {
    if (b.ok) {
      armChip = (
        <span
          className="chip bg-emerald-500/15 text-emerald-300"
          title={`robot_mode=${b.robot_mode_label} · /panda_gripper/move present`}
        >
          <CheckCircle2 size={12} /> arm ready
        </span>
      );
    } else {
      const parts: string[] = [];
      if (!b.mode_ok) parts.push(`mode ${b.robot_mode_label}`);
      if (!b.gripper_ok) parts.push("no gripper");
      armChip = (
        <span
          className="chip bg-amber-500/15 text-amber-300"
          title={b.error ?? "relaunch verification failed"}
        >
          <AlertTriangle size={12} /> arm: {parts.join(" · ") || "not ready"}
        </span>
      );
    }
  }

  return (
    <div className="flex flex-wrap items-center justify-end gap-x-3 gap-y-1.5">
      {/* current phase (most prominent) */}
      <div
        className="flex items-center gap-2 rounded-lg px-3.5 py-2"
        style={{ background: `${meta.color}1a`, border: `1px solid ${meta.color}55` }}
      >
        <span className="h-3 w-3 animate-pulsesoft rounded-full" style={{ background: meta.color }} />
        <span className="text-base font-semibold" style={{ color: meta.color }}>
          {meta.label}
        </span>
        {latest && <span className="font-mono text-xs text-slate-500">· {latest.robot_mode_label}</span>}
      </div>

      {/* robot subsystem state (live only) */}
      {(fciChip || armChip) && (
        <>
          <Sep />
          <div className="flex flex-wrap items-center gap-1.5">
            {fciChip}
            {armChip}
          </div>
        </>
      )}

      <Sep />

      {/* connection + mode + clock */}
      <div className="flex items-center gap-2">
        <span
          className={clsx(
            "chip",
            connected ? "bg-emerald-500/15 text-emerald-300" : "bg-red-500/15 text-red-300",
          )}
        >
          <Radio size={12} className={connected ? "animate-pulsesoft" : ""} />
          {connected ? "live" : "offline"}
        </span>
        <span
          className={clsx(
            "chip uppercase",
            mode === "live" ? "bg-violet-500/15 text-violet-300" : "bg-sky-500/15 text-sky-300",
          )}
        >
          {mode}
        </span>
        {latest && (
          <span className="hidden font-mono text-xs text-slate-500 sm:inline">{fmtClock(latest.t)}</span>
        )}
      </div>

      {/* consolidated subsystem errors */}
      {errors.length > 0 && (
        <>
          <Sep />
          <div className="flex flex-wrap items-center gap-1.5">
            {errors.map((e) => (
              <span
                key={e.source}
                className="chip bg-red-500/15 text-red-300"
                title={`${e.source}: ${e.message}`}
              >
                <AlertTriangle size={12} /> {e.source}: {truncate(e.message)}
              </span>
            ))}
          </div>
        </>
      )}
    </div>
  );
}
