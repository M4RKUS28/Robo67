interface Props {
  value: number;
  min?: number;
  max: number;
  label: string;
  unit: string;
  color: string;
  /** optional threshold marks drawn on the arc */
  marks?: { value: number; color: string; label?: string }[];
  decimals?: number;
}

const START = 135; // degrees
const SWEEP = 270; // total arc span

function polar(cx: number, cy: number, r: number, deg: number) {
  const a = (deg * Math.PI) / 180;
  return { x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) };
}

function arcPath(cx: number, cy: number, r: number, a0: number, a1: number) {
  const p0 = polar(cx, cy, r, a0);
  const p1 = polar(cx, cy, r, a1);
  const large = a1 - a0 <= 180 ? 0 : 1;
  return `M ${p0.x} ${p0.y} A ${r} ${r} 0 ${large} 1 ${p1.x} ${p1.y}`;
}

export function Gauge({ value, min = 0, max, label, unit, color, marks = [], decimals = 3 }: Props) {
  const size = 132;
  const cx = size / 2;
  const cy = size / 2;
  const r = size / 2 - 12;
  const frac = Math.max(0, Math.min(1, (value - min) / (max - min || 1)));
  const valAngle = START + frac * SWEEP;

  return (
    <div className="flex flex-col items-center">
      <svg width={size} height={size} viewBox={`0 0 ${size} ${size}`}>
        {/* track */}
        <path
          d={arcPath(cx, cy, r, START, START + SWEEP)}
          fill="none"
          stroke="#1c2435"
          strokeWidth={9}
          strokeLinecap="round"
        />
        {/* value */}
        {frac > 0.001 && (
          <path
            d={arcPath(cx, cy, r, START, valAngle)}
            fill="none"
            stroke={color}
            strokeWidth={9}
            strokeLinecap="round"
            style={{ filter: `drop-shadow(0 0 4px ${color}66)` }}
          />
        )}
        {/* threshold marks */}
        {marks.map((m) => {
          const f = Math.max(0, Math.min(1, (m.value - min) / (max - min || 1)));
          const a = START + f * SWEEP;
          const p0 = polar(cx, cy, r - 7, a);
          const p1 = polar(cx, cy, r + 7, a);
          return (
            <line
              key={m.label ?? m.value}
              x1={p0.x}
              y1={p0.y}
              x2={p1.x}
              y2={p1.y}
              stroke={m.color}
              strokeWidth={2.5}
            />
          );
        })}
        <text x={cx} y={cy - 2} textAnchor="middle" fontSize={22} fontWeight={600} fill="#e2e8f0" className="tabular-nums">
          {Number.isFinite(value) ? value.toFixed(decimals) : "—"}
        </text>
        <text x={cx} y={cy + 16} textAnchor="middle" fontSize={10} fill="#64748b">
          {unit}
        </text>
      </svg>
      <div className="label -mt-1">{label}</div>
    </div>
  );
}
