import { useState } from "react";
import { useQueryClient } from "@tanstack/react-query";
import { RotateCcw, Loader2, AlertTriangle, CheckCircle2 } from "lucide-react";
import { useBringupStatus, relaunchBringup } from "../api/queries";

// Relaunch the arm bringup + gripper (live mode only). The server stops any
// running franka.launch.py + gripper, relaunches both, clears a reflex via
// error_recovery if the robot isn't in Move (2), then verifies mode 2 +
// /panda_gripper/move. This kills a running bringup (and any insertion talking
// to it), so it goes through a confirm step.
export function BringupControl() {
  const status = useBringupStatus();
  const qc = useQueryClient();
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [confirm, setConfirm] = useState(false);

  const st = status.data;
  const enabled = st?.enabled ?? false;
  const running = st?.busy ?? false;
  const lastLog = st?.log?.length ? st.log[st.log.length - 1] : null;

  const refresh = () => qc.invalidateQueries({ queryKey: ["bringup-status"] });

  async function doRelaunch() {
    setBusy(true);
    setErr(null);
    try {
      const r = await relaunchBringup();
      if (!r.ok) setErr(r.error ?? "relaunch failed");
    } catch (e) {
      setErr(String(e));
    } finally {
      setBusy(false);
      setConfirm(false);
      refresh();
    }
  }

  if (!enabled) {
    return (
      <span
        className="chip bg-slate-500/15 text-slate-400"
        title="Bringup relaunch can only run in live mode (inside the container)."
      >
        bringup · live-only
      </span>
    );
  }

  // Outcome chip for the last completed relaunch (only when idle).
  const outcome =
    !running && st?.ok != null ? (
      st.ok ? (
        <span
          className="chip bg-emerald-500/15 text-emerald-300"
          title={`robot_mode=${st.robot_mode_label} · /panda_gripper/move present`}
        >
          <CheckCircle2 size={12} /> arm ready
        </span>
      ) : (
        <span
          className="chip bg-amber-500/15 text-amber-300"
          title={st.error ?? "relaunch verification failed"}
        >
          <AlertTriangle size={12} />
          {st.mode_ok ? "" : ` mode ${st.robot_mode_label}`}
          {st.gripper_ok ? "" : " · no gripper"}
        </span>
      )
    ) : null;

  return (
    <div className="flex items-center gap-2">
      {running ? (
        <span className="chip bg-sky-500/15 text-sky-300">
          <Loader2 size={12} className="animate-spin" />
          {st?.phase_label ?? "relaunching"}
          {st?.elapsed_s != null ? ` · ${st.elapsed_s.toFixed(0)}s` : ""}
        </span>
      ) : confirm ? (
        <>
          <span className="text-xs font-medium text-amber-300">
            Restart the arm bringup?
          </span>
          <button
            onClick={doRelaunch}
            disabled={busy}
            className="rounded-md bg-sky-600 px-2.5 py-1.5 text-xs font-semibold text-white transition-colors hover:bg-sky-500 disabled:opacity-50"
          >
            Confirm
          </button>
          <button
            onClick={() => setConfirm(false)}
            disabled={busy}
            className="rounded-md bg-ink-700 px-2.5 py-1.5 text-xs font-medium text-slate-300 transition-colors hover:bg-ink-600"
          >
            Cancel
          </button>
        </>
      ) : (
        <button
          onClick={() => {
            setErr(null);
            setConfirm(true);
          }}
          className="flex items-center gap-1.5 rounded-md bg-ink-700 px-3 py-1.5 text-xs font-semibold text-slate-100 transition-colors hover:bg-ink-600"
          title="Stop + relaunch franka.launch.py and the gripper, clear any reflex, verify Move (2) + /panda_gripper/move"
        >
          <RotateCcw size={13} /> Relaunch arm
        </button>
      )}

      {outcome}

      {err && (
        <span className="chip bg-red-500/15 text-red-300" title={err}>
          <AlertTriangle size={12} /> {err.length > 32 ? err.slice(0, 32) + "…" : err}
        </span>
      )}

      {running && lastLog && (
        <span
          className="hidden max-w-[300px] truncate font-mono text-[11px] text-slate-500 xl:inline"
          title={lastLog}
        >
          {lastLog}
        </span>
      )}
    </div>
  );
}
