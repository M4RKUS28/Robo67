import { useTelemetry } from "../state/TelemetryProvider";
import { useConfig } from "../api/queries";
import { CameraPanel } from "../components/CameraPanel";
import { StatTile } from "../components/StatTile";
import { fmt } from "../lib/format";

export function Cameras() {
  const { latest } = useTelemetry();
  const cfg = useConfig().data;
  const cam = cfg?.cameras;
  const c920 = latest?.detections.c920;
  const d405 = latest?.detections.d405;

  return (
    <div className="space-y-3">
      <div className="grid gap-3 lg:grid-cols-2">
        <CameraPanel
          camId="c920"
          label={cam?.c920?.label ?? "C920 overhead"}
          kind={cam?.c920?.kind ?? "static-overhead"}
          available={true}
          detection={c920}
          accent="#38bdf8"
          overlayCamId={cam?.c920?.overlay}
          markerShape="rect"
          workspacePx={cam?.c920?.workspace_px}
        />
        <CameraPanel
          camId="d405"
          label={cam?.d405?.label ?? "D405 eye-in-hand"}
          kind={cam?.d405?.kind ?? "eye-in-hand"}
          available={true}
          detection={d405}
          accent="#a78bfa"
          overlayCamId={cam?.d405?.overlay}
        />
      </div>

      <div className="grid grid-cols-2 gap-3 sm:grid-cols-4">
        <StatTile
          label="C920 hole pixel"
          value={c920?.present ? `${fmt(c920.u, 0)}, ${fmt(c920.v, 0)}` : "—"}
          hint={`r ${c920?.present ? fmt(c920.radius_px, 0) : "—"} px`}
          accent="#38bdf8"
        />
        <StatTile
          label="C920 → base XY"
          value={
            c920?.base_x != null && c920?.base_y != null
              ? `${fmt(c920.base_x)}, ${fmt(c920.base_y)}`
              : "—"
          }
          unit="m"
          hint="via homography"
        />
        <StatTile
          label="D405 servo |Δ|"
          value={
            d405?.present && d405.servo_dx != null && d405.servo_dy != null
              ? (Math.hypot(d405.servo_dx, d405.servo_dy) * 1000).toFixed(1)
              : "—"
          }
          unit="mm"
          hint="eye-in-hand correction"
          accent="#fbbf24"
        />
        <StatTile
          label="D405 detection"
          value={d405?.present ? "locked" : "—"}
          accent={d405?.present ? "#a78bfa" : undefined}
          hint={d405?.score != null ? `score ${fmt(d405.score, 2)}` : "no servo target"}
        />
      </div>

      <p className="px-1 text-xs text-slate-500">
        The overhead C920 gives the coarse socket position (mapped to the robot base frame through
        the calibrated homography). The wrist-mounted D405 refines alignment with a pixel-error
        servo vector (yellow arrow → where the tool should move). In <span className="font-mono">live</span>{" "}
        mode these come from <span className="font-mono">/robo67/socket_detection</span> and{" "}
        <span className="font-mono">/robo67/servo_correction</span>; in mock mode they are computed from
        the saved captures and the simulated insertion.
      </p>
    </div>
  );
}
