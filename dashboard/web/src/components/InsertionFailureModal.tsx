import { useEffect, useRef, useState } from "react";
import { useQueryClient } from "@tanstack/react-query";
import { AlertTriangle, Wrench, X, Loader2 } from "lucide-react";
import { useInsertionStatus, relaunchBringup, startInsertion } from "../api/queries";
import { useControls } from "../state/ControlsProvider";
import type { BringupStatus } from "../api/types";

// A healthy run logs one of these; their ABSENCE (for a run that wasn't a user
// Stop) means it failed. NOTE: the insertion process exits 0 even on a force/
// torque abort (it just holds + tries error recovery), so last_exit alone is
// NOT a reliable failure signal -- we classify from the log instead.
const SUCCESS_RE = /release-on-insert complete|sequence finished|RESULT\s*:\s*PASS/i;
const STOP_RE = /STOP requested/;
// The most informative failure CAUSE line, preferred over the generic
// "[dashboard] ... exited (rc=N)" footer and the "attempting error recovery"
// consequence line. Matches the force/torque abort, error-level log lines
// (`[ERROR]` / `ERROR:`), and the script's setup refusals.
const REASON_RE =
  /FORCE ABORT|\[ERROR\]|ERROR:|refus|unavailable|outside workspace|no socket|no robot_state/i;

const RELAUNCH_TIMEOUT_MS = 90_000;

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

async function fetchBringup(): Promise<BringupStatus> {
  const res = await fetch("/api/bringup/status");
  if (!res.ok) throw new Error(`bringup status ${res.status}`);
  return (await res.json()) as BringupStatus;
}

// Poll the relaunch until the sequence finishes (busy true -> false). Returns
// the final status, or null on timeout. Reports the current phase label so the
// modal can show progress.
async function waitRelaunchDone(onPhase: (p: string) => void): Promise<BringupStatus | null> {
  const start = Date.now();
  let sawBusy = false;
  while (Date.now() - start < RELAUNCH_TIMEOUT_MS) {
    try {
      const s = await fetchBringup();
      if (s.busy) sawBusy = true;
      onPhase(s.phase_label || (s.busy ? "relaunching" : ""));
      if (sawBusy && !s.busy) return s;
    } catch {
      /* transient during the bringup restart; keep polling */
    }
    await sleep(700);
  }
  return null;
}

// Watches the automated-insertion run (live mode). When a run we observed start
// ends WITHOUT succeeding (and wasn't a user Stop), it pops a recovery dialog.
// The one action does BOTH steps in sequence: relaunch the arm, and on a
// successful relaunch (Move + gripper verified) immediately restart insertion.
// Mounted once in the AppShell so the modal overlays the whole app.
export function InsertionFailureModal() {
  const status = useInsertionStatus();
  const { insertion } = useControls();
  const qc = useQueryClient();
  const st = status.data;
  const enabled = st?.enabled ?? false;
  const running = st?.running ?? false;

  const armedRef = useRef(false); // saw this run go "running"
  const prevRunningRef = useRef<boolean | null>(null);
  const [open, setOpen] = useState(false);
  const [reason, setReason] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [stage, setStage] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);

  useEffect(() => {
    const prev = prevRunningRef.current;
    if (enabled && running) armedRef.current = true;
    // end-edge: was running, now stopped
    if (prev === true && !running && armedRef.current) {
      armedRef.current = false;
      const log = st?.log ?? [];
      const text = log.join("\n");
      if (!STOP_RE.test(text) && !SUCCESS_RE.test(text)) {
        const line =
          [...log].reverse().find((l) => REASON_RE.test(l) && !l.startsWith("[dashboard]")) ??
          [...log].reverse().find((l) => !l.startsWith("[dashboard]")) ??
          (log.length ? log[log.length - 1] : "insertion ended without completing");
        setReason(line);
        setErr(null);
        setStage(null);
        setOpen(true);
      }
    }
    prevRunningRef.current = running;
  }, [enabled, running, st?.log]);

  if (!open) return null;

  // ONE action: relaunch the arm, then on success auto-restart the insertion.
  async function doRecover() {
    setBusy(true);
    setErr(null);
    try {
      setStage("Relaunching arm…");
      const r = await relaunchBringup();
      if (!r.ok) {
        setErr(r.error ?? "relaunch failed to start");
        return;
      }
      const done = await waitRelaunchDone((p) => setStage(`Relaunching arm · ${p}`));
      qc.invalidateQueries({ queryKey: ["bringup-status"] });
      if (!done) {
        setErr("relaunch timed out — check the Logs page");
        return;
      }
      if (!done.ok) {
        setErr(done.error ?? `relaunch failed (mode ${done.robot_mode_label})`);
        return;
      }
      // relaunch verified (Move + gripper) -> start the insertion automatically,
      // honoring the dock's force-mode toggle (defaults to force on).
      setStage("Starting insertion…");
      const s = await startInsertion(insertion.forceMode);
      qc.invalidateQueries({ queryKey: ["insertion-status"] });
      if (!s.ok) {
        setErr(s.error ?? "insertion failed to start");
        return;
      }
      setOpen(false); // success — header chip now shows the new run
    } catch (e) {
      setErr(String(e));
    } finally {
      setBusy(false);
      setStage(null);
    }
  }

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-ink-950/70 p-4 backdrop-blur-sm">
      <div className="panel w-full max-w-md p-5 shadow-xl">
        <div className="mb-3 flex items-start justify-between gap-3">
          <div className="flex items-center gap-2.5">
            <div className="grid h-9 w-9 place-items-center rounded-lg bg-red-500/15 text-red-300">
              <AlertTriangle size={20} />
            </div>
            <div>
              <div className="text-sm font-semibold text-slate-100">Insertion failed</div>
              <div className="text-[11px] text-slate-500">
                the run ended without seating the peg
              </div>
            </div>
          </div>
          <button
            onClick={() => setOpen(false)}
            disabled={busy}
            className="rounded-md p-1 text-slate-500 transition-colors hover:bg-ink-700 hover:text-slate-200 disabled:opacity-40"
            title="Dismiss"
          >
            <X size={16} />
          </button>
        </div>

        {reason && (
          <div className="mb-4 max-h-24 overflow-auto rounded-md border border-ink-700/70 bg-ink-950/50 px-3 py-2 font-mono text-[11px] leading-relaxed text-amber-300/90">
            {reason}
          </div>
        )}

        <div className="mb-2 label">Recover</div>
        <p className="mb-4 text-xs leading-relaxed text-slate-400">
          Relaunch the arm (clean restart of <span className="text-slate-300">franka.launch.py</span>{" "}
          + gripper, clear reflex, verify Move) and — once it comes back up —{" "}
          <span className="text-slate-300">restart the insertion</span> automatically.
        </p>

        {(busy || err) && (
          <div className="mb-4 flex items-center gap-2 text-xs">
            {busy ? (
              <span className="chip bg-sky-500/15 text-sky-300">
                <Loader2 size={12} className="animate-spin" />
                {stage ?? "working…"}
              </span>
            ) : (
              err && (
                <span className="chip bg-red-500/15 text-red-300" title={err}>
                  <AlertTriangle size={12} /> {err.length > 48 ? err.slice(0, 48) + "…" : err}
                </span>
              )
            )}
          </div>
        )}

        <div className="flex flex-wrap items-center justify-end gap-2">
          <button
            onClick={() => setOpen(false)}
            disabled={busy}
            className="rounded-md bg-ink-700 px-3 py-2 text-xs font-medium text-slate-300 transition-colors hover:bg-ink-600 disabled:opacity-50"
          >
            Dismiss
          </button>
          <button
            onClick={doRecover}
            disabled={busy}
            className="flex items-center gap-1.5 rounded-md bg-emerald-600 px-3 py-2 text-xs font-semibold text-white transition-colors hover:bg-emerald-500 disabled:opacity-50"
            title="Relaunch the arm, then automatically restart the insertion once it's back in Move"
          >
            {busy ? <Loader2 size={13} className="animate-spin" /> : <Wrench size={13} />}
            {err ? "Retry: relaunch & restart" : "Relaunch & restart insertion"}
          </button>
        </div>
      </div>
    </div>
  );
}
