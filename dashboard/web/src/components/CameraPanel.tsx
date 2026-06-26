import { useEffect, useRef, useState } from "react";
import clsx from "clsx";
import { Crosshair, VideoOff } from "lucide-react";
import type { C920Detection, D405Detection } from "../api/types";
import { fmt } from "../lib/format";

type Det = C920Detection | D405Detection;

interface Props {
  camId: string;
  label: string;
  kind: string;
  available: boolean;
  detection: Det | undefined;
  accent?: string;
  /** Server-rendered overlay feed id (e.g. "c920_overlay"); enables the toggle. */
  overlayCamId?: string;
  /** Client overlay marker shape: "rect" (bounding box) or "circle" (ring). */
  markerShape?: "circle" | "rect";
  /**
   * Enforced workspace AABB projected into this camera's pixels: 4 corners
   * [[u,v],...] in the source frame. When present, a dashed boundary is drawn so
   * the operator can see whether a perceived socket falls inside the box the run
   * will actually accept (overhead C920 only).
   */
  workspacePx?: [number, number][] | null;
}

function isD405(d: Det | undefined): d is D405Detection {
  return !!d && "servo_dx" in d;
}

export function CameraPanel({
  camId,
  label,
  kind,
  available,
  detection,
  accent = "#38bdf8",
  overlayCamId,
  markerShape = "circle",
  workspacePx,
}: Props) {
  // bump the MJPEG <img> src on (re)mount so the stream restarts cleanly
  const [nonce] = useState(() => Date.now());
  const [imgOk, setImgOk] = useState(true);
  // "raw" = camera_publisher feed + client SVG overlay; "processed" = the
  // ROS-rendered overlay feed (detection burned in by the detector node).
  const [view, setView] = useState<"raw" | "processed">("raw");
  const imgRef = useRef<HTMLImageElement>(null);

  const streamId = view === "processed" && overlayCamId ? overlayCamId : camId;
  const showClientOverlay = view === "raw";

  useEffect(() => {
    setImgOk(true);
  }, [available, streamId]);

  const det = detection;
  const present = !!det?.present;
  const w = det?.img_w ?? 1280;
  const h = det?.img_h ?? 720;
  const u = det?.u ?? w / 2;
  const v = det?.v ?? h / 2;
  const r = det?.radius_px ?? 24;

  const d405 = isD405(det) ? det : undefined;
  const servoMag =
    d405 && d405.servo_dx !== undefined && d405.servo_dy !== undefined
      ? Math.hypot(d405.servo_dx, d405.servo_dy)
      : undefined;

  return (
    <div className="panel overflow-hidden">
      <div className="flex items-center justify-between border-b border-ink-700/70 px-3 py-2">
        <div className="flex items-center gap-2">
          <span
            className="inline-block h-2 w-2 rounded-full"
            style={{ background: accent, boxShadow: `0 0 8px ${accent}` }}
          />
          <span className="text-sm font-semibold text-slate-100">{label}</span>
          <span className="label">{kind}</span>
        </div>
        <div className="flex items-center gap-2">
          {overlayCamId && (
            <div className="flex overflow-hidden rounded border border-ink-700 text-[10px]">
              {(["raw", "processed"] as const).map((m) => (
                <button
                  key={m}
                  type="button"
                  onClick={() => setView(m)}
                  className={clsx(
                    "px-2 py-0.5 font-medium uppercase tracking-wide",
                    view === m
                      ? "bg-slate-200 text-ink-900"
                      : "bg-ink-800 text-slate-400 hover:text-slate-200",
                  )}
                >
                  {m}
                </button>
              ))}
            </div>
          )}
          <span
            className={clsx(
              "chip",
              present
                ? "bg-emerald-500/15 text-emerald-300"
                : "bg-ink-700 text-slate-400",
            )}
          >
            <Crosshair size={12} />
            {present ? "hole locked" : "no detection"}
          </span>
        </div>
      </div>

      <div className="relative aspect-video w-full bg-black">
        {available && imgOk ? (
          <img
            ref={imgRef}
            src={`/api/cam/${streamId}?n=${nonce}`}
            alt={`${label} feed`}
            className="absolute inset-0 h-full w-full object-contain"
            onError={() => setImgOk(false)}
          />
        ) : (
          <div className="absolute inset-0 flex flex-col items-center justify-center gap-2 text-slate-500">
            <VideoOff size={28} />
            <span className="text-xs">feed unavailable</span>
          </div>
        )}

        {/* Workspace boundary — the enforced AABB projected into this camera's
            pixels (overhead C920 only). Drawn independently of any detection (and
            on both views) so the operator can see whether the socket falls inside
            the box the run will accept. Same viewBox as the detection overlay. */}
        {available && imgOk && workspacePx && workspacePx.length >= 3 && (
          <svg
            className="pointer-events-none absolute inset-0 h-full w-full"
            viewBox={`0 0 ${w} ${h}`}
            preserveAspectRatio="xMidYMid meet"
          >
            <polygon
              points={workspacePx.map(([wu, wv]) => `${wu},${wv}`).join(" ")}
              fill="#fbbf24"
              fillOpacity={0.06}
              stroke="#fbbf24"
              strokeWidth={3}
              strokeDasharray="12 8"
              strokeLinejoin="round"
              opacity={0.85}
            />
            <text
              x={workspacePx[0][0] + 8}
              y={workspacePx[0][1] - 8}
              fill="#fbbf24"
              fontSize={20}
              fontWeight={600}
              opacity={0.9}
            >
              workspace
            </text>
          </svg>
        )}

        {/* Detection overlay — viewBox matches the source frame so pixel
            coords map 1:1 under object-contain (xMidYMid meet). Skipped on the
            "processed" feed, where the detector already burned the overlay in. */}
        {present && showClientOverlay && (
          <svg
            className="pointer-events-none absolute inset-0 h-full w-full"
            viewBox={`0 0 ${w} ${h}`}
            preserveAspectRatio="xMidYMid meet"
          >
            {/* faint full-frame crosshair */}
            <line x1={u} y1={0} x2={u} y2={h} stroke={accent} strokeWidth={1} opacity={0.25} />
            <line x1={0} y1={v} x2={w} y2={v} stroke={accent} strokeWidth={1} opacity={0.25} />
            {/* detection marker: bounding rectangle (rect) or ring (circle) */}
            {markerShape === "rect" ? (
              <rect
                x={u - Math.max(r, 8)}
                y={v - Math.max(r, 8)}
                width={2 * Math.max(r, 8)}
                height={2 * Math.max(r, 8)}
                fill="none"
                stroke="#00ff00"
                strokeWidth={7}
              />
            ) : (
              <>
                <circle
                  cx={u}
                  cy={v}
                  r={Math.max(r, 8)}
                  fill="none"
                  stroke={accent}
                  strokeWidth={3}
                />
                {/* corner ticks */}
                {[-1, 1].map((sx) =>
                  [-1, 1].map((sy) => {
                    const rr = Math.max(r, 8) + 7;
                    const len = 10;
                    return (
                      <path
                        key={`${sx}${sy}`}
                        d={`M ${u + sx * rr} ${v + sy * rr} h ${sx * len} M ${u + sx * rr} ${
                          v + sy * rr
                        } v ${sy * len}`}
                        stroke={accent}
                        strokeWidth={2}
                        fill="none"
                      />
                    );
                  }),
                )}
              </>
            )}
            <circle
              cx={u}
              cy={v}
              r={markerShape === "rect" ? 7 : 3}
              fill={markerShape === "rect" ? "#ff0000" : accent}
            />
            {/* D405 servo-correction arrow: from frame centre toward the hole */}
            {d405 && (
              <g>
                <defs>
                  <marker
                    id="servoArrow"
                    markerWidth="8"
                    markerHeight="8"
                    refX="5"
                    refY="3"
                    orient="auto"
                  >
                    <path d="M0,0 L6,3 L0,6 Z" fill="#fbbf24" />
                  </marker>
                </defs>
                <line
                  x1={w / 2}
                  y1={h / 2}
                  x2={u}
                  y2={v}
                  stroke="#fbbf24"
                  strokeWidth={2.5}
                  markerEnd="url(#servoArrow)"
                  opacity={0.9}
                />
                <circle cx={w / 2} cy={h / 2} r={4} fill="none" stroke="#fbbf24" strokeWidth={2} />
              </g>
            )}
          </svg>
        )}
      </div>

      {/* footer telemetry */}
      <div className="grid grid-cols-3 gap-px border-t border-ink-700/70 bg-ink-700/40 text-center">
        {present ? (
          <>
            <Cell label="pixel u,v" value={`${fmt(u, 0)}, ${fmt(v, 0)}`} />
            {d405 ? (
              <Cell
                label="servo |Δ|"
                value={servoMag !== undefined ? `${(servoMag * 1000).toFixed(1)} mm` : "—"}
              />
            ) : (
              <Cell
                label="base x,y"
                value={
                  det && "base_x" in det && det.base_x != null && det.base_y != null
                    ? `${fmt(det.base_x, 3)}, ${fmt(det.base_y, 3)}`
                    : "—"
                }
              />
            )}
            <Cell
              label="score"
              value={det?.score != null ? fmt(det.score, 2) : "—"}
            />
          </>
        ) : (
          <div className="col-span-3 bg-ink-850 py-2 text-xs text-slate-500">
            searching for the socket hole…
          </div>
        )}
      </div>
    </div>
  );
}

function Cell({ label, value }: { label: string; value: string }) {
  return (
    <div className="bg-ink-850 px-2 py-1.5">
      <div className="label">{label}</div>
      <div className="font-mono text-sm text-slate-200">{value}</div>
    </div>
  );
}
