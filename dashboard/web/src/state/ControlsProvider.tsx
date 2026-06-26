import {
  createContext,
  useCallback,
  useContext,
  useState,
  type ReactNode,
} from "react";
import { useQueryClient } from "@tanstack/react-query";
import {
  activateFci,
  closeGripper,
  deactivateFci,
  openGripper,
  relaunchBringup,
  runHome,
  startInsertion,
  stopInsertion,
  useBringupStatus,
  useFciStatus,
  useGripperStatus,
  useHomeStatus,
  useInsertionStatus,
} from "../api/queries";
import type {
  BringupStatus,
  FciStatus,
  GripperStatus,
  HomeStatus,
  InsertionStatus,
} from "../api/types";

// ControlsProvider centralises every real-arm action the dashboard can take.
// The five robot subsystems (FCI / arm bringup / home / gripper / insertion)
// used to be five near-identical header components that each rendered a button
// AND their own status chips. Here we own the shared bits once:
//   - the polled server status (via the existing react-query hooks),
//   - the local action lifecycle (confirm gate, in-flight, last error),
//   - the action callbacks themselves.
// Two consumers split the UI cleanly: <ControlSidebar/> renders the grouped
// action buttons (busy/confirm shown inline), and <StatusCluster/> renders the
// consolidated read-only state + a single labelled list of errors.

export type ActionResult = { ok: boolean; error?: string };

export type ControlSource = "FCI" | "Arm" | "Home" | "Gripper" | "Insertion";

export interface SubsystemError {
  source: ControlSource;
  message: string;
}

// Shared local lifecycle every action goes through. Gripper open/close skip the
// confirm gate (confirm stays false), everything else is confirm-gated.
interface ActionRunner {
  inFlight: boolean;
  confirm: boolean;
  err: string | null;
  askConfirm: () => void;
  cancel: () => void;
  run: (fn: () => Promise<ActionResult>) => Promise<void>;
}

function useActionRunner(queryKey: string): ActionRunner {
  const qc = useQueryClient();
  const [inFlight, setInFlight] = useState(false);
  const [confirm, setConfirm] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const askConfirm = useCallback(() => {
    setErr(null);
    setConfirm(true);
  }, []);
  const cancel = useCallback(() => setConfirm(false), []);

  const run = useCallback(
    async (fn: () => Promise<ActionResult>) => {
      setInFlight(true);
      setErr(null);
      try {
        const r = await fn();
        if (!r.ok) setErr(r.error ?? "request failed");
      } catch (e) {
        setErr(String(e));
      } finally {
        setInFlight(false);
        setConfirm(false);
        qc.invalidateQueries({ queryKey: [queryKey] });
      }
    },
    [qc, queryKey],
  );

  return { inFlight, confirm, err, askConfirm, cancel, run };
}

// What each consumer needs per subsystem: the raw polled status (for derived
// read-only display), the local lifecycle, and the action handlers.
interface SubsystemBase<S> {
  enabled: boolean;
  status: S | undefined;
  confirm: boolean;
  inFlight: boolean;
  err: string | null;
  askConfirm: () => void;
  cancel: () => void;
}

interface FciControls extends SubsystemBase<FciStatus> {
  active: boolean | null;
  willDeactivate: boolean;
  toggle: () => void;
}
interface BringupControls extends SubsystemBase<BringupStatus> {
  relaunch: () => void;
}
interface HomeControls extends SubsystemBase<HomeStatus> {
  homeStr: string | null;
  run: () => void;
}
interface GripperControls extends SubsystemBase<GripperStatus> {
  open: () => void;
  close: () => void;
}
interface InsertionControls extends SubsystemBase<InsertionStatus> {
  forceMode: boolean;
  setForceMode: (v: boolean) => void;
  insertionMode: "peg" | "cable";
  setInsertionMode: (v: "peg" | "cable") => void;
  start: () => void;
  stop: () => void;
}

export interface ControlsValue {
  live: boolean; // any subsystem enabled => live mode (controls usable)
  fci: FciControls;
  bringup: BringupControls;
  home: HomeControls;
  gripper: GripperControls;
  insertion: InsertionControls;
  errors: SubsystemError[]; // consolidated for the status cluster
}

const Ctx = createContext<ControlsValue | null>(null);

// A server-reported error counts only when the run is idle (not busy) and the
// last result failed -- mirrors the old per-component "st.ok === false" chips.
function serverErr(
  busy: boolean | undefined,
  ok: boolean | null | undefined,
  error: string | null | undefined,
): string | null {
  return !busy && ok === false && error ? error : null;
}

export function ControlsProvider({ children }: { children: ReactNode }) {
  const fciStatus = useFciStatus().data;
  const bringupStatus = useBringupStatus().data;
  const homeStatus = useHomeStatus().data;
  const gripperStatus = useGripperStatus().data;
  const insertionStatus = useInsertionStatus().data;

  const fciR = useActionRunner("fci-status");
  const bringupR = useActionRunner("bringup-status");
  const homeR = useActionRunner("home-status");
  const gripperR = useActionRunner("gripper-status");
  const insertionR = useActionRunner("insertion-status");

  // Default the insertion to force-guided mode (ADR-0002) -- it's the preferred
  // behavior, so both the dock toggle and the relaunch+restart recovery start
  // with force on (the user can still turn it off before a run).
  const [forceMode, setForceMode] = useState(true);
  // Insertion target: peg-in-hole (default) or cable connector into the I/O box.
  const [insertionMode, setInsertionMode] = useState<"peg" | "cable">("peg");

  const fciActive = fciStatus?.fci_active ?? null;
  const willDeactivate = fciActive === true;

  const home = homeStatus?.home_xyz;
  const homeStr =
    home && home.length === 3
      ? `${home[0].toFixed(2)}, ${home[1].toFixed(2)}, ${home[2].toFixed(2)}`
      : null;

  // Consolidated errors: prefer the local action error, fall back to the last
  // server-reported failure. Home/Insertion have no server `ok` field, so they
  // surface local errors only (process failures pop the InsertionFailureModal).
  const errors: SubsystemError[] = [];
  const pushErr = (source: ControlSource, msg: string | null) => {
    if (msg) errors.push({ source, message: msg });
  };
  pushErr("FCI", fciR.err ?? serverErr(fciStatus?.busy, fciStatus?.ok, fciStatus?.error));
  // Arm: server-reported relaunch failures surface as the "arm not ready" status
  // chip (with the cause in its tooltip), so only the local action error is a
  // hard error here.
  pushErr("Arm", bringupR.err);
  pushErr("Home", homeR.err);
  pushErr(
    "Gripper",
    gripperR.err ?? serverErr(gripperStatus?.busy, gripperStatus?.ok, gripperStatus?.error),
  );
  pushErr("Insertion", insertionR.err);

  const value: ControlsValue = {
    live: !!(
      fciStatus?.enabled ||
      bringupStatus?.enabled ||
      homeStatus?.enabled ||
      gripperStatus?.enabled ||
      insertionStatus?.enabled
    ),
    fci: {
      enabled: fciStatus?.enabled ?? false,
      status: fciStatus,
      active: fciActive,
      willDeactivate,
      confirm: fciR.confirm,
      inFlight: fciR.inFlight,
      err: fciR.err,
      askConfirm: fciR.askConfirm,
      cancel: fciR.cancel,
      toggle: () => fciR.run(() => (willDeactivate ? deactivateFci() : activateFci())),
    },
    bringup: {
      enabled: bringupStatus?.enabled ?? false,
      status: bringupStatus,
      confirm: bringupR.confirm,
      inFlight: bringupR.inFlight,
      err: bringupR.err,
      askConfirm: bringupR.askConfirm,
      cancel: bringupR.cancel,
      relaunch: () => bringupR.run(() => relaunchBringup()),
    },
    home: {
      enabled: homeStatus?.enabled ?? false,
      status: homeStatus,
      homeStr,
      confirm: homeR.confirm,
      inFlight: homeR.inFlight,
      err: homeR.err,
      askConfirm: homeR.askConfirm,
      cancel: homeR.cancel,
      run: () => homeR.run(() => runHome()),
    },
    gripper: {
      enabled: gripperStatus?.enabled ?? false,
      status: gripperStatus,
      // gripper actions are direct (no confirm gate)
      confirm: false,
      inFlight: gripperR.inFlight,
      err: gripperR.err,
      askConfirm: () => {},
      cancel: () => {},
      open: () => gripperR.run(() => openGripper()),
      close: () => gripperR.run(() => closeGripper()),
    },
    insertion: {
      enabled: insertionStatus?.enabled ?? false,
      status: insertionStatus,
      forceMode,
      setForceMode,
      insertionMode,
      setInsertionMode,
      confirm: insertionR.confirm,
      inFlight: insertionR.inFlight,
      err: insertionR.err,
      askConfirm: insertionR.askConfirm,
      cancel: insertionR.cancel,
      start: () => insertionR.run(() => startInsertion(forceMode, insertionMode)),
      stop: () => insertionR.run(() => stopInsertion()),
    },
    errors,
  };

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useControls(): ControlsValue {
  const v = useContext(Ctx);
  if (!v) throw new Error("useControls must be used within ControlsProvider");
  return v;
}
