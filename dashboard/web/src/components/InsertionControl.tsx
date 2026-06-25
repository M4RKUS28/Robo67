import { useState } from "react";
import { useQueryClient } from "@tanstack/react-query";
import { Play, Square, Loader2, AlertTriangle } from "lucide-react";
import { useInsertionStatus, startInsertion, stopInsertion } from "../api/queries";

// Start / Stop the automated peg-in-hole insertion (live mode only). The server
// spawns hw_peg_in_hole_vision.py with the verified-live params; Stop sends
// SIGINT so the arm holds its last pose. A one-click Start commands the REAL
// arm, so it goes through a confirm step.
export function InsertionControl() {
  const status = useInsertionStatus();
  const qc = useQueryClient();
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [confirm, setConfirm] = useState(false);

  const st = status.data;
  const enabled = st?.enabled ?? false;
  const running = st?.running ?? false;
  const lastLog = st?.log?.length ? st.log[st.log.length - 1] : null;

  const refresh = () => qc.invalidateQueries({ queryKey: ["insertion-status"] });

  async function doStart() {
    setBusy(true);
    setErr(null);
    try {
      const r = await startInsertion();
      if (!r.ok) setErr(r.error ?? "start failed");
    } catch (e) {
      setErr(String(e));
    } finally {
      setBusy(false);
      setConfirm(false);
      refresh();
    }
  }

  async function doStop() {
    setBusy(true);
    setErr(null);
    try {
      const r = await stopInsertion();
      if (!r.ok) setErr(r.error ?? "stop failed");
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
        title="Automated insertion can only be started in live mode (inside the container)."
      >
        insertion · live-only
      </span>
    );
  }

  return (
    <div className="flex items-center gap-2">
      {running ? (
        <>
          <span className="chip bg-amber-500/15 text-amber-300">
            <Loader2 size={12} className="animate-spin" />
            inserting{st?.elapsed_s != null ? ` · ${st.elapsed_s.toFixed(0)}s` : ""}
          </span>
          <button
            onClick={doStop}
            disabled={busy}
            className="flex items-center gap-1.5 rounded-md bg-red-600 px-3 py-1.5 text-xs font-semibold text-white transition-colors hover:bg-red-500 disabled:opacity-50"
            title="Cancel: SIGINT the insertion (arm holds its last pose)"
          >
            <Square size={13} /> Stop
          </button>
        </>
      ) : confirm ? (
        <>
          <span className="text-xs font-medium text-amber-300">Move the real arm?</span>
          <button
            onClick={doStart}
            disabled={busy}
            className="rounded-md bg-emerald-600 px-2.5 py-1.5 text-xs font-semibold text-white transition-colors hover:bg-emerald-500 disabled:opacity-50"
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
          className="flex items-center gap-1.5 rounded-md bg-emerald-600 px-3 py-1.5 text-xs font-semibold text-white transition-colors hover:bg-emerald-500"
          title="Run the full automated insertion (detect → move above → contact → spiral → release)"
        >
          <Play size={13} /> Start insertion
        </button>
      )}

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
