import clsx from "clsx";
import type { ReactNode } from "react";
import {
  Bot,
  ChevronsLeftRight,
  ChevronsRightLeft,
  Gauge,
  Grip,
  Hand,
  Loader2,
  LocateFixed,
  type LucideIcon,
  Play,
  Power,
  PowerOff,
  RotateCcw,
  Square,
  Workflow,
} from "lucide-react";
import { useControls } from "../state/ControlsProvider";

// ControlDock: a fixed, slightly-transparent floating bar pinned near the
// bottom of the viewport (stays put while the page scrolls). It holds every
// real-arm action, grouped by subsystem (Robot / Gripper / Insertion). Buttons
// stay clean -- only the busy spinner and the confirm gate render inline here;
// persistent state and errors live in <StatusCluster/> in the header. All logic
// comes from <ControlsProvider/>, so these are pure presentation.

const BTN =
  "flex items-center gap-1.5 whitespace-nowrap rounded-md px-3.5 py-2.5 text-sm font-semibold transition-colors disabled:opacity-50";

type Tone = "neutral" | "emerald" | "red" | "sky";
const TONE: Record<Tone, string> = {
  neutral: "bg-ink-700 text-slate-100 hover:bg-ink-600",
  emerald: "bg-emerald-600 text-white hover:bg-emerald-500",
  red: "bg-red-600 text-white hover:bg-red-500",
  sky: "bg-sky-600 text-white hover:bg-sky-500",
};

function ActionButton({
  onClick,
  icon,
  children,
  tone = "neutral",
  disabled,
  title,
}: {
  onClick: () => void;
  icon: ReactNode;
  children: ReactNode;
  tone?: Tone;
  disabled?: boolean;
  title?: string;
}) {
  return (
    <button onClick={onClick} disabled={disabled} title={title} className={clsx(BTN, TONE[tone])}>
      {icon}
      {children}
    </button>
  );
}

function DisabledButton({ icon, children }: { icon: ReactNode; children: ReactNode }) {
  return (
    <div
      title="Available in live mode only"
      className={clsx(BTN, "cursor-not-allowed bg-ink-800 text-slate-600")}
    >
      {icon}
      {children}
    </div>
  );
}

function BusyPill({
  tone = "sky",
  icon,
  children,
}: {
  tone?: "sky" | "amber";
  icon: ReactNode;
  children: ReactNode;
}) {
  return (
    <div
      className={clsx(
        "flex items-center gap-1.5 whitespace-nowrap rounded-md px-3.5 py-2.5 text-sm font-medium",
        tone === "amber" ? "bg-amber-500/15 text-amber-300" : "bg-sky-500/15 text-sky-300",
      )}
    >
      {icon}
      {children}
    </div>
  );
}

function ConfirmRow({
  question,
  confirmLabel = "Confirm",
  confirmTone = "sky",
  onConfirm,
  onCancel,
  disabled,
}: {
  question: string;
  confirmLabel?: string;
  confirmTone?: Tone;
  onConfirm: () => void;
  onCancel: () => void;
  disabled?: boolean;
}) {
  return (
    <div className="flex items-center gap-2">
      <span className="max-w-[170px] text-xs font-medium leading-snug text-amber-300">
        {question}
      </span>
      <button
        onClick={onConfirm}
        disabled={disabled}
        className={clsx(BTN, TONE[confirmTone])}
      >
        {confirmLabel}
      </button>
      <button
        onClick={onCancel}
        disabled={disabled}
        className={clsx(BTN, "bg-ink-700 text-slate-300 hover:bg-ink-600")}
      >
        Cancel
      </button>
    </div>
  );
}

function Group({
  label,
  icon: Icon,
  children,
}: {
  label: string;
  icon: LucideIcon;
  children: ReactNode;
}) {
  return (
    <section className="flex flex-col gap-1.5">
      <div className="flex items-center gap-1.5 px-0.5 text-[11px] font-semibold uppercase tracking-wider text-slate-500">
        <Icon size={13} />
        {label}
      </div>
      <div className="flex items-center gap-2">{children}</div>
    </section>
  );
}

function Divider() {
  return <div className="w-px self-stretch bg-ink-700/60" aria-hidden />;
}

const elapsed = (s: number | null | undefined) => (s != null ? ` · ${s.toFixed(0)}s` : "");

// ---- per-subsystem rows ----------------------------------------------------

function FciRow() {
  const { fci } = useControls();
  const st = fci.status;
  if (!fci.enabled) return <DisabledButton icon={<Power size={15} />}>Activate FCI</DisabledButton>;

  const busy = fci.inFlight || (st?.busy ?? false);
  if (busy) {
    return st?.awaiting_button ? (
      <BusyPill tone="amber" icon={<Hand size={15} className="animate-pulsesoft" />}>
        press robot button{elapsed(st?.elapsed_s)}
      </BusyPill>
    ) : (
      <BusyPill icon={<Loader2 size={15} className="animate-spin" />}>
        {st?.last_action === "deactivate" ? "deactivating" : "activating"}
        {elapsed(st?.elapsed_s)}
      </BusyPill>
    );
  }
  if (fci.confirm) {
    return (
      <ConfirmRow
        question={fci.willDeactivate ? "Deactivate the FCI?" : "Activate the FCI?"}
        onConfirm={fci.toggle}
        onCancel={fci.cancel}
        disabled={fci.inFlight}
      />
    );
  }
  return (
    <ActionButton
      onClick={fci.askConfirm}
      icon={fci.willDeactivate ? <PowerOff size={15} /> : <Power size={15} />}
      title={
        fci.willDeactivate
          ? "Deactivate the FCI (frees the Desk UI)"
          : "Activate the FCI over the Desk API (may need a physical button tap)"
      }
    >
      {fci.willDeactivate ? "Deactivate FCI" : "Activate FCI"}
    </ActionButton>
  );
}

function BringupRow() {
  const { bringup } = useControls();
  const st = bringup.status;
  if (!bringup.enabled)
    return <DisabledButton icon={<RotateCcw size={15} />}>Relaunch arm</DisabledButton>;

  const busy = bringup.inFlight || (st?.busy ?? false);
  if (busy) {
    return (
      <BusyPill icon={<Loader2 size={15} className="animate-spin" />}>
        {st?.phase_label ?? "relaunching"}
        {elapsed(st?.elapsed_s)}
      </BusyPill>
    );
  }
  if (bringup.confirm) {
    return (
      <ConfirmRow
        question="Restart the arm bringup? (kills any running insertion)"
        onConfirm={bringup.relaunch}
        onCancel={bringup.cancel}
        disabled={bringup.inFlight}
      />
    );
  }
  return (
    <ActionButton
      onClick={bringup.askConfirm}
      icon={<RotateCcw size={15} />}
      title="Stop + relaunch franka.launch.py and the gripper, clear any reflex, verify Move (2)"
    >
      Relaunch arm
    </ActionButton>
  );
}

function HomeRow() {
  const { home } = useControls();
  const st = home.status;
  if (!home.enabled) return <DisabledButton icon={<LocateFixed size={15} />}>Home</DisabledButton>;

  const busy = home.inFlight || (st?.running ?? false);
  if (busy) {
    return (
      <BusyPill icon={<Loader2 size={15} className="animate-spin" />}>homing{elapsed(st?.elapsed_s)}</BusyPill>
    );
  }
  if (home.confirm) {
    return (
      <ConfirmRow
        question={`Move the arm to home${home.homeStr ? ` (${home.homeStr})` : ""}?`}
        onConfirm={home.run}
        onCancel={home.cancel}
        disabled={home.inFlight}
      />
    );
  }
  return (
    <ActionButton
      onClick={home.askConfirm}
      icon={<LocateFixed size={15} />}
      title={`Move the arm to the defined home pose${home.homeStr ? ` (${home.homeStr})` : ""}, tool-down`}
    >
      Home
    </ActionButton>
  );
}

function GripperRow() {
  const { gripper } = useControls();
  const st = gripper.status;
  if (!gripper.enabled) {
    return (
      <>
        <DisabledButton icon={<ChevronsLeftRight size={15} />}>Open</DisabledButton>
        <DisabledButton icon={<ChevronsRightLeft size={15} />}>Close</DisabledButton>
      </>
    );
  }

  const busy = gripper.inFlight || (st?.busy ?? false);
  if (busy) {
    return (
      <BusyPill icon={<Loader2 size={15} className="animate-spin" />}>
        {st?.last_action === "close" ? "closing" : "opening"}
        {elapsed(st?.elapsed_s)}
      </BusyPill>
    );
  }
  return (
    <>
      <ActionButton
        onClick={gripper.open}
        icon={<ChevronsLeftRight size={15} />}
        title="Open the gripper (Move to full width)"
      >
        Open
      </ActionButton>
      <ActionButton
        onClick={gripper.close}
        icon={<ChevronsRightLeft size={15} />}
        title="Close the gripper (Grasp with force — clamps/holds a peg)"
      >
        Close
      </ActionButton>
    </>
  );
}

function InsertionRow() {
  const { insertion } = useControls();
  const st = insertion.status;
  if (!insertion.enabled)
    return <DisabledButton icon={<Play size={15} />}>Start insertion</DisabledButton>;

  const running = st?.running ?? false;
  const runningForce = running && (st?.force_mode ?? false);

  if (running) {
    return (
      <>
        <BusyPill tone="amber" icon={<Loader2 size={15} className="animate-spin" />}>
          inserting{elapsed(st?.elapsed_s)}
        </BusyPill>
        <div
          className="flex items-center gap-1.5 whitespace-nowrap rounded-md bg-indigo-500/15 px-3 py-2 text-xs font-medium text-indigo-200"
          title="Insertion mode"
        >
          {st?.mode === "cable" ? "cable" : "peg"}
        </div>
        <div
          className={clsx(
            "flex items-center gap-1.5 whitespace-nowrap rounded-md px-3 py-2 text-xs font-medium",
            runningForce ? "bg-sky-500/15 text-sky-300" : "bg-slate-500/15 text-slate-400",
          )}
          title={
            runningForce
              ? "Force-guided mode (admittance press + force-slacken detect, ADR-0002)"
              : "Verified fixed-equilibrium mode"
          }
        >
          <Gauge size={14} /> {runningForce ? "force mode" : "fixed mode"}
        </div>
        <ActionButton
          onClick={insertion.stop}
          tone="red"
          disabled={insertion.inFlight}
          icon={<Square size={15} />}
          title="Cancel: SIGINT the insertion (arm holds its last pose)"
        >
          Stop
        </ActionButton>
      </>
    );
  }
  if (insertion.confirm) {
    return (
      <ConfirmRow
        question={`Move the real arm? (${insertion.insertionMode}, ${insertion.forceMode ? "force mode" : "fixed mode"})`}
        confirmTone="emerald"
        onConfirm={insertion.start}
        onCancel={insertion.cancel}
        disabled={insertion.inFlight}
      />
    );
  }
  return (
    <>
      <div className="flex items-center overflow-hidden rounded-md border border-ink-700/60">
        {(["peg", "cable"] as const).map((m) => (
          <button
            key={m}
            onClick={() => insertion.setInsertionMode(m)}
            aria-pressed={insertion.insertionMode === m}
            className={clsx(
              "px-3 py-2 text-xs font-medium transition-colors",
              insertion.insertionMode === m
                ? "bg-indigo-500/25 text-indigo-200"
                : "bg-ink-800 text-slate-400 hover:bg-ink-700",
            )}
            title={
              m === "cable"
                ? "Cable insertion: perceive the I/O box, move above, seat the connector (force-guided)"
                : "Peg-in-hole insertion"
            }
          >
            {m === "peg" ? "Peg" : "Cable"}
          </button>
        ))}
      </div>
      <button
        onClick={() => insertion.setForceMode(!insertion.forceMode)}
        aria-pressed={insertion.forceMode}
        className={clsx(
          BTN,
          insertion.forceMode
            ? "bg-sky-500/20 text-sky-300 hover:bg-sky-500/30"
            : "bg-ink-700 text-slate-400 hover:bg-ink-600",
        )}
        title="Force-guided mode (ADR-0002): regulate a constant gentle press and detect insertion from the force-slacken. Off = verified fixed-equilibrium behavior."
      >
        <Gauge size={15} /> Force {insertion.forceMode ? "on" : "off"}
      </button>
      <ActionButton
        onClick={insertion.askConfirm}
        tone="emerald"
        icon={<Play size={15} />}
        title="Run the full automated insertion (detect → move above → contact → spiral → release)"
      >
        Start insertion
      </ActionButton>
    </>
  );
}

export function ControlDock() {
  const { live } = useControls();
  return (
    // wrapper is click-through so the empty space beside the dock doesn't block
    // the page underneath; the dock itself re-enables pointer events.
    <div className="pointer-events-none fixed inset-x-0 bottom-6 z-40 flex justify-center px-4">
      <div className="pointer-events-auto flex max-w-[calc(100vw-2rem)] items-stretch gap-4 overflow-x-auto rounded-2xl border border-ink-700/60 bg-ink-900/70 px-4 py-3 shadow-panel ring-1 ring-white/5 backdrop-blur-md">
        <Group label="Robot" icon={Bot}>
          <FciRow />
          <BringupRow />
          <HomeRow />
        </Group>

        <Divider />

        <Group label="Gripper" icon={Grip}>
          <GripperRow />
        </Group>

        <Divider />

        <Group label="Insertion" icon={Workflow}>
          <InsertionRow />
        </Group>

        {!live && (
          <>
            <Divider />
            <div className="flex items-center">
              <span
                className="chip bg-slate-500/15 text-slate-500"
                title="Robot controls only work in live mode"
              >
                live only
              </span>
            </div>
          </>
        )}
      </div>
    </div>
  );
}
