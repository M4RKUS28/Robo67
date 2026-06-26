import type { ReactNode } from "react";
import { Link } from "@tanstack/react-router";
import { Activity, Cctv, ListTree, Cpu, ScrollText } from "lucide-react";
import { ControlsProvider } from "../state/ControlsProvider";
import { ControlDock } from "./ControlDock";
import { StatusCluster } from "./StatusCluster";
import { FciButtonModal } from "./FciButtonModal";
import { InsertionFailureModal } from "./InsertionFailureModal";

const NAV = [
  { to: "/", label: "Overview", icon: Activity },
  { to: "/cameras", label: "Cameras", icon: Cctv },
  { to: "/decisions", label: "Decisions", icon: ListTree },
  { to: "/logs", label: "Logs", icon: ScrollText },
] as const;

export function AppShell({ children }: { children: ReactNode }) {
  return (
    <ControlsProvider>
      <div className="mx-auto flex min-h-full max-w-[1500px] flex-col px-3 sm:px-5">
        <header className="sticky top-0 z-30 -mx-3 mb-3 border-b border-ink-700/60 bg-ink-950/85 px-3 py-4 backdrop-blur sm:-mx-5 sm:px-5">
          <div className="flex flex-wrap items-center gap-x-5 gap-y-2.5">
            {/* brand */}
            <div className="flex items-center gap-3">
              <div className="grid h-11 w-11 place-items-center rounded-xl bg-accent/15 text-accent">
                <Cpu size={24} />
              </div>
              <div>
                <div className="text-base font-semibold leading-tight text-slate-100">
                  Robo67 · Insertion Telemetry
                </div>
                <div className="text-xs leading-tight text-slate-500">
                  Franka peg-in-hole · classical vision + force
                </div>
              </div>
            </div>

            {/* nav */}
            <nav className="flex items-center gap-1 rounded-lg bg-ink-850 p-1">
              {NAV.map((n) => (
                <Link
                  key={n.to}
                  to={n.to}
                  className="flex items-center gap-1.5 rounded-md px-3 py-2 text-sm font-medium text-slate-400 transition-colors hover:text-slate-100"
                  activeProps={{ className: "bg-ink-700 !text-slate-100" }}
                  activeOptions={{ exact: n.to === "/" }}
                >
                  <n.icon size={16} />
                  {n.label}
                </Link>
              ))}
            </nav>

            {/* consolidated read-only status */}
            <div className="ml-auto">
              <StatusCluster />
            </div>
          </div>
        </header>

        {/* routed page content — extra bottom padding so the last rows clear
            the fixed control dock that floats above it */}
        <main className="flex-1 pb-32">{children}</main>

        {/* fixed, slightly-transparent control dock pinned near the bottom */}
        <ControlDock />

        {/* prominent prompt while the FCI flow waits for the physical button tap */}
        <FciButtonModal />

        {/* global recovery dialog when a started insertion run fails */}
        <InsertionFailureModal />
      </div>
    </ControlsProvider>
  );
}
