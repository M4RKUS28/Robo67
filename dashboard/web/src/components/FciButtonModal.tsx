import { useFciStatus } from "../api/queries";
import { Hand, Loader2, Circle } from "lucide-react";

// Full-screen prompt shown while the FCI activate/deactivate flow is waiting for
// the user to physically confirm control (Single Point of Control): when another
// Desk session holds control, the robot only grants our forced request after the
// circle button on the robot's Pilot is pressed. The server sets awaiting_button
// for that window; this modal makes the ask impossible to miss (the header chip
// alone was too subtle). It closes itself as soon as control is granted or the
// window times out. Mounted once in the AppShell so it overlays the whole app.
export function FciButtonModal() {
  const st = useFciStatus().data;
  const showing = (st?.busy ?? false) && (st?.awaiting_button ?? false);
  if (!showing) return null;

  const total = st?.take_timeout_s ?? 30;
  const elapsed = st?.elapsed_s ?? 0;
  const remaining = Math.max(0, Math.ceil(total - elapsed));

  return (
    <div className="fixed inset-0 z-[60] flex items-center justify-center bg-ink-950/80 p-4 backdrop-blur-sm">
      <div className="panel w-full max-w-md p-6 text-center shadow-xl ring-1 ring-amber-400/40">
        <div className="mx-auto mb-4 grid h-16 w-16 place-items-center rounded-full bg-amber-500/15 text-amber-300">
          <div className="relative grid place-items-center">
            <Circle size={40} className="animate-pulsesoft" strokeWidth={2.5} />
            <Hand size={18} className="absolute" />
          </div>
        </div>

        <div className="text-base font-semibold text-slate-100">
          Press the circle button on the robot
        </div>
        <p className="mx-auto mt-2 max-w-sm text-xs leading-relaxed text-slate-400">
          Control is held by another Desk session, so the robot needs physical
          confirmation. Press the <span className="font-semibold text-amber-300">○ circle</span>{" "}
          button on the robot's Pilot (or base) to hand control to the dashboard.
        </p>

        <div className="mt-5 flex items-center justify-center gap-2">
          <span className="chip bg-amber-500/15 text-amber-300">
            <Loader2 size={12} className="animate-spin" />
            waiting for confirmation
          </span>
          <span className="chip bg-ink-700 font-mono text-slate-300" title="time left to confirm">
            {remaining}s left
          </span>
        </div>

        <div className="mt-4 text-[11px] text-slate-500">
          This closes automatically once control is granted (or the window expires).
        </div>
      </div>
    </div>
  );
}
