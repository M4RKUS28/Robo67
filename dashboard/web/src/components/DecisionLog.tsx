import { useMemo } from "react";
import {
  createColumnHelper,
  flexRender,
  getCoreRowModel,
  useReactTable,
} from "@tanstack/react-table";
import clsx from "clsx";
import { ArrowRightLeft, CheckCircle2, CircleDot, Hand, AlertTriangle } from "lucide-react";
import type { DecisionEvent } from "../api/types";
import { EVENT_COLOR } from "../lib/phases";
import { fmtClock } from "../lib/format";

const KIND_ICON: Record<string, typeof CircleDot> = {
  transition: ArrowRightLeft,
  contact: Hand,
  drop: CircleDot,
  done: CheckCircle2,
  error: AlertTriangle,
};

const col = createColumnHelper<DecisionEvent>();

export function DecisionLog({ events, height = 320 }: { events: DecisionEvent[]; height?: number }) {
  const data = useMemo(() => [...events].reverse(), [events]);

  const columns = useMemo(
    () => [
      col.accessor("t", {
        header: "t",
        cell: (c) => <span className="font-mono text-xs text-slate-500">{fmtClock(c.getValue())}</span>,
        size: 56,
      }),
      col.accessor("kind", {
        header: "event",
        cell: (c) => {
          const kind = c.getValue();
          const Icon = KIND_ICON[kind] ?? CircleDot;
          const color = EVENT_COLOR[kind] ?? "#94a3b8";
          return (
            <span className="chip" style={{ background: `${color}1f`, color }}>
              <Icon size={12} />
              {kind}
            </span>
          );
        },
        size: 110,
      }),
      col.accessor("msg", {
        header: "detail",
        cell: (c) => <span className="text-sm text-slate-300">{c.getValue()}</span>,
      }),
    ],
    [],
  );

  const table = useReactTable({ data, columns, getCoreRowModel: getCoreRowModel() });

  return (
    <div className="panel overflow-hidden">
      <div className="flex items-center justify-between border-b border-ink-700/70 px-3 py-2">
        <span className="label">Decision log</span>
        <span className="text-xs text-slate-500">{events.length} events</span>
      </div>
      <div className="overflow-auto" style={{ maxHeight: height }}>
        <table className="w-full border-collapse">
          <thead className="sticky top-0 z-10 bg-ink-850">
            {table.getHeaderGroups().map((hg) => (
              <tr key={hg.id}>
                {hg.headers.map((h) => (
                  <th
                    key={h.id}
                    className="border-b border-ink-700/70 px-3 py-1.5 text-left label"
                    style={{ width: h.getSize() }}
                  >
                    {flexRender(h.column.columnDef.header, h.getContext())}
                  </th>
                ))}
              </tr>
            ))}
          </thead>
          <tbody>
            {data.length === 0 && (
              <tr>
                <td colSpan={3} className="px-3 py-6 text-center text-sm text-slate-500">
                  waiting for the robot to make a move…
                </td>
              </tr>
            )}
            {table.getRowModel().rows.map((row, i) => (
              <tr
                key={row.id}
                className={clsx(
                  "border-b border-ink-800/60 hover:bg-ink-800/40",
                  i === 0 && "bg-ink-800/30",
                )}
              >
                {row.getVisibleCells().map((cell) => (
                  <td key={cell.id} className="px-3 py-1.5 align-middle">
                    {flexRender(cell.column.columnDef.cell, cell.getContext())}
                  </td>
                ))}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}
