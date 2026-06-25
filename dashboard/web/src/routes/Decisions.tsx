import { useTelemetry } from "../state/TelemetryProvider";
import { PhaseTimeline } from "../components/PhaseTimeline";
import { DecisionLog } from "../components/DecisionLog";
import { TimeSeriesChart } from "../components/TimeSeriesChart";

export function Decisions() {
  const { latest, history, events, clearEvents } = useTelemetry();
  const contactN = latest?.contact_threshold_n ?? 5;
  const abortN = latest?.f_abort_n ?? 25;

  return (
    <div className="space-y-3">
      <PhaseTimeline current={latest?.phase ?? "UNKNOWN"} error={latest?.abort} />

      <div className="panel panel-pad">
        <div className="mb-1 flex items-center justify-between">
          <span className="label">Force vs. decision thresholds</span>
          <span className="text-xs text-slate-500">
            contact at {contactN} N · abort at {abortN} N
          </span>
        </div>
        <div className="h-48">
          <TimeSeriesChart
            gradientId="g-force-dec"
            data={history}
            unit="N"
            domainMax={abortN * 1.1}
            windowSeconds={30}
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

      <div className="flex justify-end">
        <button
          onClick={clearEvents}
          className="rounded-md border border-ink-600 px-3 py-1.5 text-xs text-slate-300 hover:bg-ink-700"
        >
          Clear log
        </button>
      </div>

      <DecisionLog events={events} height={520} />
    </div>
  );
}
