// Telemetry contract — mirrors the JSON emitted by dashboard/server (see
// mock_provider._snapshot / live_provider._snapshot).

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export interface Wrench {
  fx: number;
  fy: number;
  fz: number;
  tx: number;
  ty: number;
  tz: number;
}

export interface C920Detection {
  present: boolean;
  u?: number;
  v?: number;
  radius_px?: number;
  score?: number;
  base_x?: number | null;
  base_y?: number | null;
  img_w: number;
  img_h: number;
}

export interface D405Detection {
  present: boolean;
  u?: number;
  v?: number;
  radius_px?: number;
  score?: number | null;
  servo_dx?: number;
  servo_dy?: number;
  img_w: number;
  img_h: number;
}

export interface Detections {
  c920: C920Detection;
  d405: D405Detection;
}

export type EventKind = "transition" | "contact" | "drop" | "error" | "done";

export interface DecisionEvent {
  t: number;
  kind: EventKind;
  from?: string;
  to?: string;
  msg: string;
}

export interface Telemetry {
  t: number;
  wall: number;
  mode: "mock" | "live";
  phase: string;
  phase_label: string;
  robot_mode: number;
  robot_mode_label: string;
  ee: Vec3 | null;
  cmd: Vec3 | null;
  socket: Vec3 | null;
  speed: number;
  speed_cap: number;
  wrench: Wrench;
  force_mag: number;
  fz: number;
  fz_baseline: number;
  contact_threshold_n: number;
  f_abort_n: number;
  contact: boolean;
  retries: number;
  abort: boolean;
  done: boolean;
  error: string | null;
  detections: Detections;
  events: DecisionEvent[];
}

export interface Health {
  mode: string;
  ros: boolean;
  cameras: Record<string, boolean>;
  rate_hz?: number;
  phase_topic?: boolean;
  telemetry?: boolean;
  devices?: Record<string, number>;
}

export interface InsertionStatus {
  enabled: boolean; // true only in live mode
  running: boolean;
  pid: number | null;
  elapsed_s: number | null;
  last_exit: number | null;
  log: string[];
}

export interface PhaseInfo {
  id: string;
  label: string;
}

export interface Config {
  mode: string;
  phases: PhaseInfo[];
  robot_modes: Record<string, string>;
  thresholds: {
    contact_fz_n: number;
    f_abort_n: number;
    speed_cap_mps: number;
    insert_depth_m: number;
  };
  workspace_aabb?: number[][];
  cameras: Record<
    string,
    { label: string; size: [number, number]; kind: string; overlay?: string }
  >;
}

// One charting sample kept in the ring buffer.
export interface Sample {
  t: number;
  speed: number;
  speedCap: number;
  fz: number;
  forceMag: number;
  contactThreshold: number;
  abortThreshold: number;
  phase: string;
  eeX: number | null;
  eeY: number | null;
  eeZ: number | null;
}
