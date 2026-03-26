import { useRef, useState } from 'react'

export interface AnchorDef {
  key:    string
  label:  string
  color?: string
}

export type AnchorMap = Record<string, { x: number; y: number }>

// ── Drag handles ──────────────────────────────────────────────────────────
// Render inside the image-bound div. Calls onAnchorsChange as you drag.

interface HandlesProps {
  anchors:          AnchorMap
  defs:             AnchorDef[]
  onAnchorsChange:  (next: AnchorMap) => void
}

export function AnchorHandles({ anchors, defs, onAnchorsChange }: HandlesProps) {
  const dragging     = useRef<string | null>(null)
  const containerRef = useRef<HTMLDivElement>(null)

  function handleMouseMove(e: React.MouseEvent<HTMLDivElement>) {
    if (!dragging.current || !containerRef.current) return
    const rect = containerRef.current.getBoundingClientRect()
    const x = Math.min(100, Math.max(0, ((e.clientX - rect.left) / rect.width)  * 100))
    const y = Math.min(100, Math.max(0, ((e.clientY - rect.top)  / rect.height) * 100))
    const key = dragging.current
    onAnchorsChange({ ...anchors, [key]: { x, y } })
  }

  function stopDrag() { dragging.current = null }

  return (
    <div
      ref={containerRef}
      onMouseMove={handleMouseMove}
      onMouseUp={stopDrag}
      onMouseLeave={stopDrag}
      style={{ position: 'absolute', inset: 0, zIndex: 20 }}
    >
      {defs.map(def => {
        const pos   = anchors[def.key]
        const color = def.color ?? '#f97316'
        return (
          <div
            key={def.key}
            onMouseDown={e => { e.preventDefault(); dragging.current = def.key }}
            style={{
              position: 'absolute', left: `${pos.x}%`, top: `${pos.y}%`,
              transform: 'translate(-50%, -50%)',
              cursor: 'grab', userSelect: 'none', zIndex: 21,
            }}
          >
            <div style={{
              width: 14, height: 14, borderRadius: '50%',
              background: color, border: '2px solid white',
              boxShadow: '0 1px 6px rgba(0,0,0,0.6)',
            }} />
            <div style={{
              position: 'absolute', left: '50%', top: '100%',
              transform: 'translateX(-50%)', marginTop: 4,
              background: 'rgba(0,0,0,0.82)', color: 'white',
              fontSize: 10, fontWeight: 600, padding: '2px 5px', borderRadius: 4,
              whiteSpace: 'nowrap', pointerEvents: 'none',
              border: `1px solid ${color}66`,
            }}>
              {def.label}
            </div>
          </div>
        )
      })}
    </div>
  )
}

// ── Code panel ────────────────────────────────────────────────────────────
// Render below the image wrapper, inside the view card. No portal needed.

interface PanelProps {
  anchors:    AnchorMap
  defs:       AnchorDef[]
  constName?: string
}

export function AnchorPanel({ anchors, defs, constName = 'ANCHORS' }: PanelProps) {
  const [copied, setCopied] = useState(false)

  const codeLines  = defs
    .map(d => `  ${d.key}: { x: ${anchors[d.key].x.toFixed(1)}, y: ${anchors[d.key].y.toFixed(1)} },`)
    .join('\n')
  const codeOutput = `const ${constName} = {\n${codeLines}\n}`

  function handleCopy() {
    navigator.clipboard.writeText(codeOutput).then(() => {
      setCopied(true)
      setTimeout(() => setCopied(false), 2000)
    })
  }

  return (
    <div style={{
      borderTop:  '1px solid rgba(255,255,255,0.08)',
      background: 'rgba(0,0,0,0.5)',
      padding:    '8px 12px',
      display:    'flex',
      alignItems: 'flex-start',
      gap:        10,
      flexShrink: 0,
    }}>
      <pre style={{
        flex: 1, margin: 0, fontSize: 11, lineHeight: 1.6,
        color: '#a3e635', fontFamily: 'monospace',
        userSelect: 'text', cursor: 'text', overflowX: 'auto',
      }}>
        {codeOutput}
      </pre>
      <button onClick={handleCopy} style={{
        flexShrink: 0, alignSelf: 'center',
        padding: '5px 12px', borderRadius: 5,
        border: '1px solid rgba(255,255,255,0.15)',
        background: copied ? '#22c55e22' : 'rgba(255,255,255,0.07)',
        color:      copied ? '#22c55e'   : 'white',
        fontSize: 11, fontWeight: 600, cursor: 'pointer',
        transition: 'all 0.15s', fontFamily: 'monospace',
      }}>
        {copied ? 'Copied!' : 'Copy'}
      </button>
    </div>
  )
}
