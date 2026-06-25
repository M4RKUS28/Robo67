export const fmt = (v: number | null | undefined, digits = 3): string =>
  v === null || v === undefined || Number.isNaN(v) ? "—" : v.toFixed(digits);

export const fmtMm = (m: number | null | undefined): string =>
  m === null || m === undefined ? "—" : `${(m * 1000).toFixed(1)} mm`;

export const fmtClock = (t: number): string => {
  const s = Math.floor(t % 60);
  const m = Math.floor(t / 60);
  return `${m.toString().padStart(2, "0")}:${s.toString().padStart(2, "0")}`;
};
