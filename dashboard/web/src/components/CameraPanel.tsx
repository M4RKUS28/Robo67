import { useEffect, useRef, useState } from "react";
import clsx from "clsx";
import { Crosshair, VideoOff } from "lucide-react";
import type { C920Detection, D405Detection } from "../api/types";
import { fmt } from "../lib/format";

type Det = C920Detection | D405Detection;

interface Props {
  camId: "c920" | "d405";
  label: string;
  kind: string;
  available: boolean;
  detection: Det | undefined;
  accent?: string;
}

function isD405(d: Det | undefined): d is D405Detection {
  return !!d && "servo_dx" in d;
}

export function CameraPanel({ camId, label, kind, available, detection, accent = "#38bdf8" }: Props) {
  // bump the MJPEG <img> src on (re)mount so the stream restarts cleanly
  const [nonce] = useState(() => Date.now());
  const [imgOk, setImgOk] = useState(true);
  const imgRef = useRef<HTMLImageElement>(null);

  useEffect(() => {
    setImgOk(true);
  }, [available]);

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

      <div className="relative aspect-video w-full bg-black">
        {available && imgOk ? (
          <img
            ref={imgRef}
            src={`/api/cam/${camId}?n=${nonce}`}
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

        {/* Detection overlay — viewBox matches the source frame so pixel
            coords map 1:1 under object-contain (xMidYMid meet). */}
        {present && (
          <svg
            className="pointer-events-none absolute inset-0 h-full w-full"
            viewBox={`0 0 ${w} ${h}`}
            preserveAspectRatio="xMidYMid meet"
          >
            {/* faint full-frame crosshair */}
            <line x1={u} y1={0} x2={u} y2={h} stroke={accent} strokeWidth={1} opacity={0.25} />
            <line x1={0} y1={v} x2={w} y2={v} stroke={accent} strokeWidth={1} opacity={0.25} />
            {/* detection ring */}
            <circle
              cx={u}
              cy={v}
              r={Math.max(r, 8)}
              fill="none"
              stroke={accent}
              strokeWidth={3}
            />
            <circle cx={u} cy={v} r={3} fill={accent} />
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
