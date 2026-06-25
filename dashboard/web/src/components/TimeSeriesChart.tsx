import { useMemo } from "react";
import { Group } from "@visx/group";
import { scaleLinear } from "@visx/scale";
import { AreaClosed, LinePath, Line } from "@visx/shape";
import { GridRows } from "@visx/grid";
import { AxisLeft, AxisBottom } from "@visx/axis";
import { LinearGradient } from "@visx/gradient";
import { curveMonotoneX } from "@visx/curve";
import { ParentSize } from "@visx/responsive";
import { max as d3max } from "d3-array";
import type { Sample } from "../api/types";

export interface SeriesDef {
  accessor: (s: Sample) => number;
  color: string;
  label: string;
  fill?: boolean;
}

export interface RefLine {
  value: number;
  color: string;
  label: string;
  dash?: boolean;
}

interface Props {
  data: Sample[];
  series: SeriesDef[];
  refLines?: RefLine[];
  unit: string;
  windowSeconds?: number;
  domainMax?: number;
  gradientId: string;
}

const MARGIN = { top: 10, right: 14, bottom: 22, left: 40 };

function Chart({
  data,
  series,
  refLines = [],
  unit,
  windowSeconds = 18,
  domainMax,
  gradientId,
  width,
  height,
}: Props & { width: number; height: number }) {
  const innerW = Math.max(0, width - MARGIN.left - MARGIN.right);
  const innerH = Math.max(0, height - MARGIN.top - MARGIN.bottom);

  const view = useMemo(() => {
    if (data.length === 0) return [] as Sample[];
    const tEnd = data[data.length - 1].t;
    const tStart = tEnd - windowSeconds;
    return data.filter((d) => d.t >= tStart);
  }, [data, windowSeconds]);

  const { xScale, yScale } = useMemo(() => {
    const tEnd = view.length ? view[view.length - 1].t : 0;
    const tStart = tEnd - windowSeconds;
    const dataMax =
      d3max(view, (d) => Math.max(...series.map((s) => s.accessor(d)))) ?? 1;
    const refMax = refLines.length ? Math.max(...refLines.map((r) => r.value)) : 0;
    const top = domainMax ?? Math.max(dataMax * 1.15, refMax * 1.1, 0.01);
    return {
      xScale: scaleLinear({ domain: [tStart, tEnd], range: [0, innerW] }),
      yScale: scaleLinear({ domain: [0, top], range: [innerH, 0], nice: true }),
    };
  }, [view, series, refLines, windowSeconds, domainMax, innerW, innerH]);

  if (width < 40 || height < 40) return null;

  return (
    <svg width={width} height={height}>
      <LinearGradient id={gradientId} from={series[0].color} to={series[0].color} fromOpacity={0.28} toOpacity={0} />
      <Group left={MARGIN.left} top={MARGIN.top}>
        <GridRows scale={yScale} width={innerW} height={innerH} stroke="#1c2435" strokeDasharray="2,3" />

        {refLines.map((r) => {
          const y = yScale(r.value);
          if (!Number.isFinite(y)) return null;
          return (
            <Group key={r.label}>
              <Line
                from={{ x: 0, y }}
                to={{ x: innerW, y }}
                stroke={r.color}
                strokeWidth={1.25}
                strokeDasharray={r.dash === false ? undefined : "5,4"}
                opacity={0.7}
              />
              <text x={innerW - 2} y={y - 3} textAnchor="end" fontSize={9} fill={r.color} opacity={0.9}>
                {r.label}
              </text>
            </Group>
          );
        })}

        {series.map((s, i) => (
          <Group key={s.label}>
            {s.fill && (
              <AreaClosed<Sample>
                data={view}
                x={(d) => xScale(d.t)}
                y={(d) => yScale(s.accessor(d))}
                yScale={yScale}
                curve={curveMonotoneX}
                fill={`url(#${gradientId})`}
              />
            )}
            <LinePath<Sample>
              data={view}
              x={(d) => xScale(d.t)}
              y={(d) => yScale(s.accessor(d))}
              stroke={s.color}
              strokeWidth={i === 0 ? 1.8 : 1.4}
              curve={curveMonotoneX}
            />
          </Group>
        ))}

        <AxisLeft
          scale={yScale}
          numTicks={4}
          stroke="#283246"
          tickStroke="#283246"
          tickLabelProps={() => ({ fill: "#64748b", fontSize: 9, dx: -2, dy: 3, textAnchor: "end" })}
        />
        <AxisBottom
          top={innerH}
          scale={xScale}
          numTicks={5}
          stroke="#283246"
          tickStroke="#283246"
          tickFormat={(v) => `${Number(v).toFixed(0)}s`}
          tickLabelProps={() => ({ fill: "#64748b", fontSize: 9, textAnchor: "middle", dy: 2 })}
        />
        <text x={2} y={-1} fontSize={9} fill="#64748b">
          {unit}
        </text>
      </Group>
    </svg>
  );
}

export function TimeSeriesChart(props: Props) {
  return (
    <ParentSize>
      {({ width, height }) => <Chart {...props} width={width} height={height} />}
    </ParentSize>
  );
}
