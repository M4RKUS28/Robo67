import { useState } from "react";
import { useQueryClient } from "@tanstack/react-query";
import { LocateFixed, Loader2, AlertTriangle } from "lucide-react";
import { useHomeStatus, runHome } from "../api/queries";

// "Bring to home" (live mode only): capture the pose the arm is in RIGHT NOW
// and command the controller to hold it (re-anchors the equilibrium at the
// current measured EE -- ~no net motion). The server runs hw_cartesian_hold.py.
// It commands the real arm, so it goes through a confirm step.
export function HomeControl() {
  const status = useHomeStatus();
  const qc = useQueryClient();
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [confirm, setConfirm] = useState(false);

  const st = status.data;
  const enabled = st?.enabled ?? false;
  const running = st?.running ?? false;

  const refresh = () => qc.invalidateQueries({ queryKey: ["home-status"] });

  async function doRun() {
    setBusy(true);
    setErr(null);
    try {
      const r = await runHome();
      if (!r.ok) setErr(r.error ?? "home failed");
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
        title="Bring-to-home can only run in live mode (inside the container)."
      >
        home · live-only
      </span>
    );
  }

  return (
    <div className="flex items-center gap-2">
      {running ? (
        <span className="chip bg-sky-500/15 text-sky-300">
          <Loader2 size={12} className="animate-spin" />
          homing{st?.elapsed_s != null ? ` · ${st.elapsed_s.toFixed(0)}s` : ""}
        </span>
      ) : confirm ? (
        <>
          <span className="text-xs font-medium text-amber-300">Hold the current pose?</span>
          <button
            onClick={doRun}
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
          title="Hold the pose the arm is in right now (re-anchor the controller equilibrium at the current EE)"
        >
          <LocateFixed size={13} /> Home
        </button>
      )}

      {err && (
        <span className="chip bg-red-500/15 text-red-300" title={err}>
          <AlertTriangle size={12} /> {err.length > 32 ? err.slice(0, 32) + "…" : err}
        </span>
      )}
    </div>
  );
}
