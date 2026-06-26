import { useState } from "react";
import { useQueryClient } from "@tanstack/react-query";
import {
  ChevronsLeftRight,
  ChevronsRightLeft,
  Loader2,
  AlertTriangle,
} from "lucide-react";
import { useGripperStatus, openGripper, closeGripper } from "../api/queries";

// Gripper open/close (live mode only). Open -> franka_msgs/action/Move (no grip
// force); Close -> franka_msgs/action/Grasp (clamps with force, so it holds a
// peg). These are quick, discrete actions so they fire directly (no confirm
// modal) -- only one runs at a time.
export function GripperControl() {
  const status = useGripperStatus();
  const qc = useQueryClient();
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const st = status.data;
  const enabled = st?.enabled ?? false;
  const running = st?.busy ?? false;

  const refresh = () => qc.invalidateQueries({ queryKey: ["gripper-status"] });

  async function doAction(which: "open" | "close") {
    setBusy(true);
    setErr(null);
    try {
      const r = which === "open" ? await openGripper() : await closeGripper();
      if (!r.ok) setErr(r.error ?? `${which} failed`);
    } catch (e) {
      setErr(String(e));
    } finally {
      setBusy(false);
      refresh();
    }
  }

  if (!enabled) {
    return (
      <span
        className="chip bg-slate-500/15 text-slate-400"
        title="Gripper open/close can only run in live mode (needs the gripper node)."
      >
        gripper · live-only
      </span>
    );
  }

  if (running) {
    return (
      <span className="chip bg-sky-500/15 text-sky-300">
        <Loader2 size={12} className="animate-spin" />
        gripper {st?.last_action === "close" ? "closing" : "opening"}
        {st?.elapsed_s != null ? ` · ${st.elapsed_s.toFixed(0)}s` : ""}
      </span>
    );
  }

  return (
    <div className="flex items-center gap-1.5">
      <button
        onClick={() => doAction("open")}
        disabled={busy}
        className="flex items-center gap-1.5 rounded-md bg-ink-700 px-3 py-1.5 text-xs font-semibold text-slate-100 transition-colors hover:bg-ink-600 disabled:opacity-50"
        title="Open the gripper (Move to full width)"
      >
        <ChevronsLeftRight size={13} /> Open
      </button>
      <button
        onClick={() => doAction("close")}
        disabled={busy}
        className="flex items-center gap-1.5 rounded-md bg-ink-700 px-3 py-1.5 text-xs font-semibold text-slate-100 transition-colors hover:bg-ink-600 disabled:opacity-50"
        title="Close the gripper (Grasp with force — clamps/holds a peg)"
      >
        <ChevronsRightLeft size={13} /> Close
      </button>

      {err && (
        <span className="chip bg-red-500/15 text-red-300" title={err}>
          <AlertTriangle size={12} /> {err.length > 28 ? err.slice(0, 28) + "…" : err}
        </span>
      )}
      {!err && st?.ok === false && st?.error && (
        <span className="chip bg-red-500/15 text-red-300" title={st.error}>
          <AlertTriangle size={12} />
          {st.error.length > 28 ? st.error.slice(0, 28) + "…" : st.error}
        </span>
      )}
    </div>
  );
}
