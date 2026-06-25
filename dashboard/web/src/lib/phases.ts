// Canonical phase metadata: colours + ordering for the FSM the robot walks
// through (IDLE → MOVE_ABOVE → … → DONE/ERROR). Mirrors
// robo67_insertion.lib.insertion_intent.PHASES.

export interface PhaseMeta {
  label: string;
  /** hex used by SVG charts/overlays */
  color: string;
  /** short description of the decision happening in this phase */
  blurb: string;
}

export const PHASE_ORDER = [
  "MOVE_ABOVE",
  "DESCEND_TO_CONTACT",
  "SEARCH_SPIRAL",
  "PUSH_INSERT",
  "CONFIRM",
  "RETRACT",
  "DONE",
] as const;

export const PHASE_META: Record<string, PhaseMeta> = {
  IDLE: { label: "Idle", color: "#64748b", blurb: "Waiting to start." },
  MOVE_ABOVE: {
    label: "Move above",
    color: "#38bdf8",
    blurb: "Free-space move to the standoff pose above the socket.",
  },
  DESCEND_TO_CONTACT: {
    label: "Descend",
    color: "#f59e0b",
    blurb: "Lower until the force baseline trips — find the surface.",
  },
  SEARCH_SPIRAL: {
    label: "Spiral search",
    color: "#a78bfa",
    blurb: "Archimedean spiral until the peg drops into the hole.",
  },
  PUSH_INSERT: {
    label: "Push / insert",
    color: "#fb923c",
    blurb: "Press the peg down to the seated depth.",
  },
  CONFIRM: {
    label: "Confirm",
    color: "#34d399",
    blurb: "Verify the peg is seated below the surface.",
  },
  RETRACT: {
    label: "Retract",
    color: "#22d3ee",
    blurb: "Lift back to the standoff pose.",
  },
  DONE: { label: "Done", color: "#22c55e", blurb: "Insertion complete." },
  ERROR: { label: "Error", color: "#ef4444", blurb: "Aborted / search exhausted." },
  UNKNOWN: { label: "Unknown", color: "#6b7280", blurb: "No phase reported." },
};

export function phaseMeta(id: string | undefined | null): PhaseMeta {
  if (!id) return PHASE_META.UNKNOWN;
  return PHASE_META[id] ?? { label: id, color: "#6b7280", blurb: "" };
}

export const EVENT_COLOR: Record<string, string> = {
  transition: "#38bdf8",
  contact: "#f59e0b",
  drop: "#a78bfa",
  done: "#22c55e",
  error: "#ef4444",
};
