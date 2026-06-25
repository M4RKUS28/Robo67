import {
  createContext,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from "react";
import type { DecisionEvent, Sample, Telemetry } from "../api/types";

const HISTORY_CAP = 1500; // ~30 s at the 50 Hz mock rate
const EVENTS_CAP = 400;
const FLUSH_MS = 66; // ~15 fps UI flush regardless of stream rate

interface TelemetryState {
  connected: boolean;
  latest: Telemetry | null;
  history: Sample[];
  events: DecisionEvent[];
  clearEvents: () => void;
}

const Ctx = createContext<TelemetryState | null>(null);

export function TelemetryProvider({ children }: { children: ReactNode }) {
  const latestRef = useRef<Telemetry | null>(null);
  const historyRef = useRef<Sample[]>([]);
  const eventsRef = useRef<DecisionEvent[]>([]);
  const dirtyRef = useRef(false);
  const eventSeq = useRef(0);

  const [connected, setConnected] = useState(false);
  const [latest, setLatest] = useState<Telemetry | null>(null);
  const [history, setHistory] = useState<Sample[]>([]);
  const [events, setEvents] = useState<DecisionEvent[]>([]);

  const clearEvents = () => {
    eventsRef.current = [];
    setEvents([]);
  };

  useEffect(() => {
    const es = new EventSource("/api/stream");
    es.onopen = () => setConnected(true);
    es.onerror = () => setConnected(false);
    es.onmessage = (e) => {
      let t: Telemetry;
      try {
        t = JSON.parse(e.data) as Telemetry;
      } catch {
        return;
      }
      latestRef.current = t;
      const hist = historyRef.current;
      hist.push({
        t: t.t,
        speed: t.speed,
        speedCap: t.speed_cap,
        fz: t.fz,
        forceMag: t.force_mag,
        contactThreshold: t.contact_threshold_n,
        abortThreshold: t.f_abort_n,
        phase: t.phase,
        eeX: t.ee ? t.ee.x : null,
        eeY: t.ee ? t.ee.y : null,
        eeZ: t.ee ? t.ee.z : null,
      });
      if (hist.length > HISTORY_CAP) hist.splice(0, hist.length - HISTORY_CAP);
      if (t.events && t.events.length) {
        for (const ev of t.events) {
          eventsRef.current.push({ ...ev, t: ev.t });
        }
        if (eventsRef.current.length > EVENTS_CAP) {
          eventsRef.current.splice(0, eventsRef.current.length - EVENTS_CAP);
        }
        eventSeq.current += t.events.length;
      }
      dirtyRef.current = true;
    };

    const id = window.setInterval(() => {
      if (!dirtyRef.current) return;
      dirtyRef.current = false;
      setLatest(latestRef.current);
      setHistory(historyRef.current.slice());
      setEvents(eventsRef.current.slice());
    }, FLUSH_MS);

    return () => {
      es.close();
      window.clearInterval(id);
    };
  }, []);

  const value = useMemo<TelemetryState>(
    () => ({ connected, latest, history, events, clearEvents }),
    [connected, latest, history, events],
  );

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useTelemetry(): TelemetryState {
  const v = useContext(Ctx);
  if (!v) throw new Error("useTelemetry must be used within TelemetryProvider");
  return v;
}
