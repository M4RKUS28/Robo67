import { useTelemetry } from "../state/TelemetryProvider";
import { useConfig } from "../api/queries";
import { CameraPanel } from "../components/CameraPanel";
import { TimeSeriesChart } from "../components/TimeSeriesChart";
import { Gauge } from "../components/Gauge";
import { PhaseTimeline } from "../components/PhaseTimeline";
import { DecisionLog } from "../components/DecisionLog";
import { StatTile } from "../components/StatTile";
import { EePlot } from "../components/EePlot";
import { fmt } from "../lib/format";

export function Overview() {
  const { latest, history, events } = useTelemetry();
  const cfg = useConfig().data;

  const speedCap = latest?.speed_cap ?? cfg?.thresholds.speed_cap_mps ?? 0.05;
  const contactN = latest?.contact_threshold_n ?? cfg?.thresholds.contact_fz_n ?? 5;
  const abortN = latest?.f_abort_n ?? cfg?.thresholds.f_abort_n ?? 25;
  const forceMag = latest?.force_mag ?? 0;
  const aboveContact = forceMag >= contactN;
  const camMeta = cfg?.cameras;

  return (
    <div className="space-y-3">
      <PhaseTimeline current={latest?.phase ?? "UNKNOWN"} error={latest?.abort} />

      <div className="grid gap-3 lg:grid-cols-3">
        {/* left: cameras + charts */}
        <div className="space-y-3 lg:col-span-2">
          <div className="grid gap-3 sm:grid-cols-2">
            <CameraPanel
              camId="c920"
              label={camMeta?.c920?.label ?? "C920 overhead"}
              kind={camMeta?.c920?.kind ?? "static-overhead"}
              available={!!latest}
              detection={latest?.detections.c920}
              accent="#38bdf8"
              markerShape="rect"
              workspacePx={camMeta?.c920?.workspace_px}
            />
            <CameraPanel
              camId="d405"
              label={camMeta?.d405?.label ?? "D405 eye-in-hand"}
              kind={camMeta?.d405?.kind ?? "eye-in-hand"}
              available={!!latest}
              detection={latest?.detections.d405}
              accent="#a78bfa"
            />
          </div>

          <div className="grid gap-3 sm:grid-cols-2">
            <div className="panel panel-pad">
              <div className="mb-1 flex items-center justify-between">
                <span className="label">EE speed</span>
                <span className="font-mono text-xs text-slate-400">
                  {fmt(latest?.speed ?? 0)} m/s
                </span>
              </div>
              <div className="h-40">
                <TimeSeriesChart
                  gradientId="g-speed"
                  data={history}
                  unit="m/s"
                  domainMax={speedCap * 1.3}
                  series={[{ accessor: (s) => s.speed, color: "#38bdf8", label: "speed", fill: true }]}
                  refLines={[{ value: speedCap, color: "#f59e0b", label: "cap" }]}
                />
              </div>
            </div>

            <div className="panel panel-pad">
              <div className="mb-1 flex items-center justify-between">
                <span className="label">Contact force</span>
                <span className="font-mono text-xs text-slate-400">{fmt(forceMag, 1)} N</span>
              </div>
              <div className="h-40">
                <TimeSeriesChart
                  gradientId="g-force"
                  data={history}
                  unit="N"
                  domainMax={abortN * 1.1}
                  series={[
                    { accessor: (s) => s.forceMag, color: "#fb923c", label: "|F|", fill: true },
                    { accessor: (s) => Math.abs(s.fz), color: "#a78bfa", label: "|Fz|" },
                  ]}
                  refLines={[
                    { value: contactN, color: "#34d399", label: "contact" },
                    { value: abortN, color: "#ef4444", label: "abort" },
                  ]}
                />
              </div>
            </div>
          </div>
        </div>

        {/* right: gauges + stats + xy plot */}
        <div className="space-y-3">
          <div className="panel panel-pad">
            <div className="grid grid-cols-2 gap-2">
              <Gauge
                value={latest?.speed ?? 0}
                max={speedCap * 1.3}
                marks={[{ value: speedCap, color: "#f59e0b" }]}
                label="speed"
                unit="m/s"
                color="#38bdf8"
                decimals={3}
              />
              <Gauge
                value={forceMag}
                max={abortN}
                marks={[
                  { value: contactN, color: "#34d399" },
                  { value: abortN, color: "#ef4444" },
                ]}
                label="|force|"
                unit="N"
                color={aboveContact ? "#fb923c" : "#34d399"}
                decimals={1}
              />
            </div>
          </div>

          <div className="grid grid-cols-2 gap-3">
            <StatTile
              label="EE height z"
              value={latest?.ee ? fmt(latest.ee.z) : "—"}
              unit="m"
              hint={latest?.socket ? `socket ${fmt(latest.socket.z)} m` : undefined}
            />
            <StatTile
              label="Fz / baseline"
              value={fmt(latest?.fz ?? 0, 1)}
              unit="N"
              hint={`base ${fmt(latest?.fz_baseline ?? 0, 1)} N`}
              accent={aboveContact ? "#fb923c" : "#34d399"}
            />
            <StatTile
              label="spiral retries"
              value={latest?.retries ?? 0}
              hint={`limit 3`}
              alarm={(latest?.retries ?? 0) >= 3}
            />
            <StatTile
              label="contact"
              value={latest?.contact ? "YES" : "no"}
              accent={latest?.contact ? "#34d399" : undefined}
              hint={`thr ${contactN} N`}
            />
          </div>

          <div className="panel panel-pad">
            <div className="mb-1 flex items-center justify-between">
              <span className="label">Alignment (top-down)</span>
              <span className="font-mono text-[11px] text-slate-500">
                {latest?.ee && latest?.socket
                  ? `Δ ${(
                      Math.hypot(latest.ee.x - latest.socket.x, latest.ee.y - latest.socket.y) * 1000
                    ).toFixed(1)} mm`
                  : "—"}
              </span>
            </div>
            <div className="h-44">
              <EePlot history={history} latest={latest} />
            </div>
          </div>
        </div>
      </div>

      <DecisionLog events={events} height={240} />
    </div>
  );
}
