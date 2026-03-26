import { CSSProperties } from 'react'

// ── Shared overlay primitives ──────────────────────────────────────────────
// All x/y values are percentages (0–100) relative to the robot image container.

interface CircleOverlayProps {
  x: number
  y: number
  color: string
  label: string
  sublabel?: string
  size?: number
}

/** Colored circle indicator with a value label, centered on (x%, y%). */
export function CircleOverlay({ x, y, color, label, sublabel, size = 48 }: CircleOverlayProps) {
  return (
    <div
      style={{
        position: 'absolute',
        left: `${x}%`,
        top: `${y}%`,
        transform: 'translate(-50%, -50%)',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        pointerEvents: 'none',
      }}
    >
      <div
        style={{
          width: size,
          height: size,
          borderRadius: '50%',
          background: color,
          border: '2px solid rgba(255,255,255,0.25)',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          fontSize: size * 0.22,
          fontWeight: 700,
          color: 'white',
          transition: 'background 0.25s ease',
          boxShadow: `0 0 12px ${color}88`,
        }}
      >
        {label}
      </div>
      {sublabel && (
        <span style={{ fontSize: 10, color: '#cbd5e1', marginTop: 3, whiteSpace: 'nowrap' }}>
          {sublabel}
        </span>
      )}
    </div>
  )
}

interface TextOverlayProps {
  x: number
  y: number
  label: string
  value: string
  value2?: string
  note?: string
  color?: string
  align?: 'center' | 'left' | 'right'
}

/** Dark pill showing "Label: value", centered on (x%, y%). */
export function TextOverlay({ x, y, label, value, value2, note, color, align = 'center' }: TextOverlayProps) {
  const translateX = align === 'right' ? '-100%' : align === 'left' ? '0%' : '-50%'
  return (
    <div
      style={{
        position: 'absolute',
        left: `${x}%`,
        top: `${y}%`,
        transform: `translate(${translateX}, -50%)`,
        background: 'rgba(0,0,0,0.72)',
        backdropFilter: 'blur(4px)',
        border: '1px solid rgba(255,255,255,0.12)',
        borderRadius: 6,
        padding: '3px 8px',
        color: color ?? 'white',
        fontSize: 12,
        whiteSpace: 'nowrap',
        pointerEvents: 'none',
        fontVariantNumeric: 'tabular-nums',
      }}
    >
      <span style={{ color: '#94a3b8' }}>{label}: </span>
      <strong>{value}</strong>
      {value2 !== undefined && (
        <div>
          <span style={{ color: '#94a3b8' }}>Target: </span>
          <strong>{value2}</strong>
        </div>
      )}
      {note !== undefined && (
        <div style={{ fontSize: 10, color: '#94a3b8', marginTop: 1 }}>{note}</div>
      )}
    </div>
  )
}

interface DotOverlayProps {
  x: number
  y: number
  color: string
  tooltip?: string
}

/** Small status dot, e.g. for beam break / boolean state. */
export function DotOverlay({ x, y, color, tooltip }: DotOverlayProps) {
  const style: CSSProperties = {
    position: 'absolute',
    left: `${x}%`,
    top: `${y}%`,
    transform: 'translate(-50%, -50%)',
    width: 14,
    height: 14,
    borderRadius: '50%',
    background: color,
    border: '2px solid rgba(255,255,255,0.3)',
    boxShadow: `0 0 8px ${color}aa`,
    transition: 'background 0.15s ease, box-shadow 0.15s ease',
    pointerEvents: 'none',
    title: tooltip,
  }
  return <div style={style} title={tooltip} />
}
