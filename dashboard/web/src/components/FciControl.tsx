import { useState } from "react";
import { useQueryClient } from "@tanstack/react-query";
import { Power, PowerOff, Loader2, AlertTriangle, Hand } from "lucide-react";
import { useFciStatus, activateFci, deactivateFci } from "../api/queries";

// FCI on/off (live mode only): toggle the Franka Control Interface over the
// Desk HTTP API (login + take control + activate/deactivate). Taking control
// when the Desk is held elsewhere needs a physical button tap on the robot, so
// the busy state surfaces that. FCI active locks out the Desk UI, so the toggle
// is confirm-gated.
export function FciControl() {
  const status = useFciStatus();
  const qc = useQueryClient();
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [confirm, setConfirm] = useState(false);

  const st = status.data;
  const enabled = st?.enabled ?? false;
  const running = st?.busy ?? false;
  const active = st?.fci_active ?? null;
  // unknown state defaults to "activate" (the common pre-run need)
  const willDeactivate = active === true;

  const refresh = () => qc.invalidateQueries({ queryKey: ["fci-status"] });

  async function doToggle() {
    setBusy(true);
    setErr(null);
    try {
      const r = willDeactivate ? await deactivateFci() : await activateFci();
      if (!r.ok) setErr(r.error ?? "FCI toggle failed");
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
        title="FCI on/off can only run in live mode (talks to the robot's Desk API)."
      >
        FCI · live-only
      </span>
    );
  }

  // small state chip (only when we actually know the state)
  const stateChip =
    !running && active != null ? (
      <span
        className={
          active
            ? "chip bg-emerald-500/15 text-emerald-300"
            : "chip bg-slate-500/15 text-slate-400"
        }
        title={active ? "FCI active (Desk UI locked out)" : "FCI inactive"}
      >
        FCI {active ? "on" : "off"}
      </span>
    ) : null;

  return (
    <div className="flex items-center gap-2">
      {running ? (
        st?.awaiting_button ? (
          <span
            className="chip bg-amber-500/15 text-amber-300"
            title="Control is held elsewhere — press the button on the robot to grant control."
          >
            <Hand size={12} className="animate-pulsesoft" />
            press robot button{st?.elapsed_s != null ? ` · ${st.elapsed_s.toFixed(0)}s` : ""}
          </span>
        ) : (
          <span className="chip bg-sky-500/15 text-sky-300">
            <Loader2 size={12} className="animate-spin" />
            {st?.last_action === "deactivate" ? "deactivating FCI" : "activating FCI"}
            {st?.elapsed_s != null ? ` · ${st.elapsed_s.toFixed(0)}s` : ""}
          </span>
        )
      ) : confirm ? (
        <>
          <span className="text-xs font-medium text-amber-300">
            {willDeactivate ? "Deactivate FCI?" : "Activate FCI?"}
          </span>
          <button
            onClick={doToggle}
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
          title={
            willDeactivate
              ? "Deactivate the FCI (frees the Desk UI)"
              : "Activate the FCI over the Desk API (may need a physical button tap on the robot)"
          }
        >
          {willDeactivate ? <PowerOff size={13} /> : <Power size={13} />}
          {willDeactivate ? "Deactivate FCI" : "Activate FCI"}
        </button>
      )}

      {stateChip}

      {err && (
        <span className="chip bg-red-500/15 text-red-300" title={err}>
          <AlertTriangle size={12} /> {err.length > 32 ? err.slice(0, 32) + "…" : err}
        </span>
      )}
      {!running && !err && st?.ok === false && st?.error && (
        <span className="chip bg-red-500/15 text-red-300" title={st.error}>
          <AlertTriangle size={12} />
          {st.error.length > 32 ? st.error.slice(0, 32) + "…" : st.error}
        </span>
      )}
    </div>
  );
}
