import clsx from "clsx";
import type { ReactNode } from "react";
import { Link } from "@tanstack/react-router";
import { Activity, Cctv, ListTree, Radio, Cpu, ScrollText } from "lucide-react";
import { useHealth } from "../api/queries";
import { useTelemetry } from "../state/TelemetryProvider";
import { phaseMeta } from "../lib/phases";
import { fmtClock } from "../lib/format";
import { InsertionControl } from "./InsertionControl";
import { BringupControl } from "./BringupControl";
import { HomeControl } from "./HomeControl";

const NAV = [
  { to: "/", label: "Overview", icon: Activity },
  { to: "/cameras", label: "Cameras", icon: Cctv },
  { to: "/decisions", label: "Decisions", icon: ListTree },
  { to: "/logs", label: "Logs", icon: ScrollText },
] as const;

export function AppShell({ children }: { children: ReactNode }) {
  const { connected, latest } = useTelemetry();
  const health = useHealth();
  const mode = latest?.mode ?? health.data?.mode ?? "—";
  const meta = phaseMeta(latest?.abort ? "ERROR" : latest?.phase);

  return (
    <div className="mx-auto flex min-h-full max-w-[1500px] flex-col px-3 sm:px-5">
      <header className="sticky top-0 z-30 -mx-3 mb-3 border-b border-ink-700/60 bg-ink-950/85 px-3 py-2.5 backdrop-blur sm:-mx-5 sm:px-5">
        <div className="flex flex-wrap items-center gap-x-4 gap-y-2">
          <div className="flex items-center gap-2.5">
            <div className="grid h-8 w-8 place-items-center rounded-lg bg-accent/15 text-accent">
              <Cpu size={18} />
            </div>
            <div>
              <div className="text-sm font-semibold leading-tight text-slate-100">
                Robo67 · Insertion Telemetry
              </div>
              <div className="text-[11px] leading-tight text-slate-500">
                Franka peg-in-hole · classical vision + force
              </div>
            </div>
          </div>

          {/* current phase */}
          <div
            className="flex items-center gap-2 rounded-lg px-3 py-1.5"
            style={{ background: `${meta.color}1a`, border: `1px solid ${meta.color}55` }}
          >
            <span className="h-2.5 w-2.5 animate-pulsesoft rounded-full" style={{ background: meta.color }} />
            <span className="text-sm font-semibold" style={{ color: meta.color }}>
              {meta.label}
            </span>
            {latest && (
              <span className="font-mono text-xs text-slate-500">· {latest.robot_mode_label}</span>
            )}
          </div>

          <nav className="ml-auto flex items-center gap-1 rounded-lg bg-ink-850 p-1">
            {NAV.map((n) => (
              <Link
                key={n.to}
                to={n.to}
                className="flex items-center gap-1.5 rounded-md px-2.5 py-1.5 text-xs font-medium text-slate-400 transition-colors hover:text-slate-100"
                activeProps={{ className: "bg-ink-700 !text-slate-100" }}
                activeOptions={{ exact: n.to === "/" }}
              >
                <n.icon size={14} />
                {n.label}
              </Link>
            ))}
          </nav>

          {/* arm bringup relaunch (live mode only) */}
          <BringupControl />

          {/* bring to home — hold current pose (live mode only) */}
          <HomeControl />

          {/* automated-insertion start / stop (live mode only) */}
          <InsertionControl />

          {/* connection + mode */}
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
              <span className="hidden font-mono text-xs text-slate-500 sm:inline">
                {fmtClock(latest.t)}
              </span>
            )}
          </div>
        </div>
      </header>

      <main className="flex-1 pb-6">{children}</main>
    </div>
  );
}
