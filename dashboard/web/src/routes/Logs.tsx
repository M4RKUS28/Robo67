import {
  useInsertionStatus,
  useBringupStatus,
  useHomeStatus,
  useFciStatus,
} from "../api/queries";
import { LogPanel } from "../components/LogPanel";

// Process logs for the dashboard's managed real-arm runs. Each panel mirrors the
// ring-buffered stdout the server captures from the spawned subprocess:
//   - Insertion: hw_peg_in_hole_vision.py (Start insertion)
//   - Arm relaunch: franka.launch.py + gripper relaunch (Relaunch arm)
//   - Home: hw_cartesian_hold.py (Home — hold current pose)
//   - FCI: Desk-API login / take-control / activate-FCI (Activate/Deactivate FCI)
export function Logs() {
  const ins = useInsertionStatus().data;
  const brk = useBringupStatus().data;
  const home = useHomeStatus().data;
  const fci = useFciStatus().data;

  return (
    <div className="space-y-3">
      <p className="px-1 text-xs text-slate-500">
        Live stdout of the dashboard's managed real-arm runs (ring-buffered on the server).
        These populate only in <span className="font-mono">live</span> mode, when the run is
        started from the header buttons. Newest lines stick to the bottom.
      </p>

      <LogPanel
        title="Insertion log"
        subtitle="hw_peg_in_hole_vision.py · Start insertion"
        enabled={ins?.enabled ?? false}
        running={ins?.running ?? false}
        elapsedS={ins?.elapsed_s ?? null}
        outcome={
          ins && !ins.running && ins.last_exit != null
            ? { ok: ins.last_exit === 0, label: `exit ${ins.last_exit}` }
            : undefined
        }
        log={ins?.log ?? []}
        height={360}
      />

      <LogPanel
        title="Arm relaunch log"
        subtitle="franka.launch.py + gripper · Relaunch arm"
        enabled={brk?.enabled ?? false}
        running={brk?.busy ?? false}
        elapsedS={brk?.elapsed_s ?? null}
        outcome={
          brk && !brk.busy && brk.ok != null
            ? { ok: brk.ok, label: brk.ok ? "ready" : "failed" }
            : undefined
        }
        log={brk?.log ?? []}
        height={300}
      />

      <LogPanel
        title="Home log"
        subtitle="hw_move_to.py · move to defined home pose"
        enabled={home?.enabled ?? false}
        running={home?.running ?? false}
        elapsedS={home?.elapsed_s ?? null}
        outcome={
          home && !home.running && home.last_exit != null
            ? { ok: home.last_exit === 0, label: `exit ${home.last_exit}` }
            : undefined
        }
        log={home?.log ?? []}
        height={220}
      />

      <LogPanel
        title="FCI log"
        subtitle="Desk API · login → take control → activate/deactivate FCI"
        enabled={fci?.enabled ?? false}
        running={fci?.busy ?? false}
        elapsedS={fci?.elapsed_s ?? null}
        outcome={
          fci && !fci.busy && fci.ok != null
            ? { ok: fci.ok, label: fci.ok ? (fci.last_action ?? "ok") : "failed" }
            : undefined
        }
        log={fci?.log ?? []}
        height={300}
      />
    </div>
  );
}
