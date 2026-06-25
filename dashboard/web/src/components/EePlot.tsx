import { useMemo } from "react";
import { ParentSize } from "@visx/responsive";
import type { Sample, Telemetry } from "../api/types";
import { phaseMeta } from "../lib/phases";

interface Props {
  history: Sample[];
  latest: Telemetry | null;
}

// Top-down (base-frame XY) view of the tool centre relative to the socket. The
// recent trail makes the approach + Archimedean spiral search visible.
function Plot({ history, latest, width, height }: Props & { width: number; height: number }) {
  const socket = latest?.socket ?? null;

  const trail = useMemo(
    () => history.filter((s) => s.eeX != null && s.eeY != null).slice(-400),
    [history],
  );

  if (width < 40 || height < 40 || !socket) {
    return (
      <div className="flex h-full items-center justify-center text-xs text-slate-500">
        no pose data
      </div>
    );
  }

  // fixed ±4 cm window around the socket (insertion is a fine-alignment task)
  const span = 0.04;
  const pad = 14;
  const sx = (x: number) =>
    pad + ((x - (socket.x - span)) / (2 * span)) * (width - 2 * pad);
  const sy = (y: number) =>
    // base +Y maps up on screen
    height - pad - ((y - (socket.y - span)) / (2 * span)) * (height - 2 * pad);

  const cx = sx(socket.x);
  const cy = sy(socket.y);
  const ee = latest?.ee;
  const meta = phaseMeta(latest?.phase);

  const ringR = (m: number) => (m / (2 * span)) * (width - 2 * pad);

  return (
    <svg width={width} height={height}>
      {/* tolerance rings around the socket (1 cm / 2 cm) */}
      {[0.01, 0.02].map((m) => (
        <circle key={m} cx={cx} cy={cy} r={ringR(m)} fill="none" stroke="#1c2435" strokeDasharray="3,3" />
      ))}
      {/* socket target crosshair */}
      <line x1={cx - 9} y1={cy} x2={cx + 9} y2={cy} stroke="#22c55e" strokeWidth={1.5} />
      <line x1={cx} y1={cy - 9} x2={cx} y2={cy + 9} stroke="#22c55e" strokeWidth={1.5} />
      <circle cx={cx} cy={cy} r={4} fill="none" stroke="#22c55e" strokeWidth={1.5} />

      {/* EE trail */}
      {trail.length > 1 && (
        <polyline
          points={trail.map((s) => `${sx(s.eeX as number)},${sy(s.eeY as number)}`).join(" ")}
          fill="none"
          stroke={meta.color}
          strokeWidth={1.5}
          opacity={0.55}
        />
      )}

      {/* current EE */}
      {ee && (
        <g>
          <circle cx={sx(ee.x)} cy={sy(ee.y)} r={6} fill={meta.color} opacity={0.25} />
          <circle cx={sx(ee.x)} cy={sy(ee.y)} r={3.5} fill={meta.color} />
        </g>
      )}

      <text x={pad} y={height - 4} fontSize={9} fill="#475569">
        ±{(span * 100).toFixed(0)} cm · base XY
      </text>
    </svg>
  );
}

export function EePlot(props: Props) {
  return <ParentSize>{({ width, height }) => <Plot {...props} width={width} height={height} />}</ParentSize>;
}
