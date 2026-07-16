import { useMemo, useRef, useState } from 'react'

const WIDTH = 600
const HEIGHT = 220
const PAD = { top: 16, right: 56, bottom: 24, left: 40 }

// 고정 순서 — 색상은 시리즈(차량)에 귀속, 절대 순환하지 않음
const SERIES = [
  { key: 'B', label: 'B 차량', colorVar: '--series-1' },
  { key: 'C', label: 'C 차량', colorVar: '--series-2' },
]

export function DistanceChart({ history }) {
  const svgRef = useRef(null)
  const [hoverIndex, setHoverIndex] = useState(null)

  const points = history.length > 0 ? history : [{ t: Date.now(), B: null, C: null }]
  const innerW = WIDTH - PAD.left - PAD.right
  const innerH = HEIGHT - PAD.top - PAD.bottom

  const maxDistance = useMemo(() => {
    let max = 50
    for (const p of points) {
      if (typeof p.B === 'number') max = Math.max(max, p.B)
      if (typeof p.C === 'number') max = Math.max(max, p.C)
    }
    return Math.ceil(max / 25) * 25
  }, [points])

  const xFor = (i) => PAD.left + (points.length <= 1 ? 0 : (i / (points.length - 1)) * innerW)
  const yFor = (v) => PAD.top + innerH - (v / maxDistance) * innerH

  const linePath = (key) => {
    let d = ''
    let started = false
    points.forEach((p, i) => {
      const v = p[key]
      if (typeof v !== 'number') {
        started = false
        return
      }
      const x = xFor(i)
      const y = yFor(v)
      d += started ? ` L ${x} ${y}` : `M ${x} ${y}`
      started = true
    })
    return d
  }

  const lastValueIndex = (key) => {
    for (let i = points.length - 1; i >= 0; i -= 1) {
      if (typeof points[i][key] === 'number') return i
    }
    return -1
  }

  const yTicks = [0, maxDistance / 2, maxDistance]

  const handleMove = (e) => {
    if (!svgRef.current || points.length === 0) return
    const rect = svgRef.current.getBoundingClientRect()
    const relX = ((e.clientX - rect.left) / rect.width) * WIDTH
    const ratio = (relX - PAD.left) / innerW
    const idx = Math.round(ratio * (points.length - 1))
    setHoverIndex(Math.min(Math.max(idx, 0), points.length - 1))
  }

  const hoverPoint = hoverIndex != null ? points[hoverIndex] : null

  return (
    <div className="chart-card">
      <div className="chart-card__header">
        <h3>실시간 차간거리</h3>
        <ul className="chart-legend">
          {SERIES.map((s) => (
            <li key={s.key}>
              <span className="chart-legend__swatch" style={{ background: `var(${s.colorVar})` }} />
              {s.label}
            </li>
          ))}
        </ul>
      </div>

      <svg
        ref={svgRef}
        viewBox={`0 0 ${WIDTH} ${HEIGHT}`}
        className="chart-svg"
        onMouseMove={handleMove}
        onMouseLeave={() => setHoverIndex(null)}
      >
        {yTicks.map((tick) => (
          <g key={tick}>
            <line x1={PAD.left} x2={WIDTH - PAD.right} y1={yFor(tick)} y2={yFor(tick)} className="chart-gridline" />
            <text x={PAD.left - 8} y={yFor(tick)} className="chart-tick" textAnchor="end" dominantBaseline="middle">
              {Math.round(tick)}
            </text>
          </g>
        ))}

        {SERIES.map((s) => (
          <path key={s.key} d={linePath(s.key)} className="chart-line" style={{ stroke: `var(${s.colorVar})` }} fill="none" />
        ))}

        {SERIES.map((s) => {
          const i = lastValueIndex(s.key)
          if (i === -1) return null
          const v = points[i][s.key]
          return (
            <g key={`${s.key}-end`}>
              <circle cx={xFor(i)} cy={yFor(v)} r="5" className="chart-end-dot" style={{ fill: `var(${s.colorVar})` }} />
              <text x={xFor(i) + 9} y={yFor(v)} className="chart-end-label" dominantBaseline="middle">
                {v}cm
              </text>
            </g>
          )
        })}

        {hoverPoint && (
          <line x1={xFor(hoverIndex)} x2={xFor(hoverIndex)} y1={PAD.top} y2={HEIGHT - PAD.bottom} className="chart-crosshair" />
        )}
      </svg>

      {hoverPoint ? (
        <div className="chart-tooltip">
          {SERIES.map((s) => (
            <div key={s.key}>
              <span className="chart-legend__swatch" style={{ background: `var(${s.colorVar})` }} />
              {s.label}: {typeof hoverPoint[s.key] === 'number' ? `${hoverPoint[s.key]} cm` : '—'}
            </div>
          ))}
        </div>
      ) : (
        <p className="chart-hint">그래프 위에 마우스를 올리면 시점별 값을 볼 수 있습니다.</p>
      )}
    </div>
  )
}
