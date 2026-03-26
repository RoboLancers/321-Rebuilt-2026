import { useState } from 'react'

interface Bounds { left: number; top: number; width: number; height: number }

interface Props {
  initialX:   number
  initialY:   number
  bounds:     Bounds
  color?:     string
  configKey?: string  // shown in the readout so user knows which field to update
}

export function PivotCalibrator({
  initialX, initialY, bounds,
  color     = '#f97316',
  configKey = 'pivotFraction',
}: Props) {
  const [pos, setPos] = useState({ x: initialX, y: initialY })

  function handleClick(e: React.MouseEvent<HTMLDivElement>) {
    const rect = e.currentTarget.getBoundingClientRect()
    setPos({
      x: (e.clientX - rect.left) / rect.width,
      y: (e.clientY - rect.top)  / rect.height,
    })
  }

  return (
    <div
      onClick={handleClick}
      style={{
        position: 'absolute',
        left: bounds.left, top: bounds.top,
        width: bounds.width, height: bounds.height,
        cursor: 'crosshair', zIndex: 10,
      }}
    >
      <div style={{ position:'absolute', left:`${pos.x*100}%`, top:0, bottom:0,
        width:1, background:`${color}99`, pointerEvents:'none' }} />
      <div style={{ position:'absolute', top:`${pos.y*100}%`, left:0, right:0,
        height:1, background:`${color}99`, pointerEvents:'none' }} />
      <div style={{
        position:'absolute', left:`${pos.x*100}%`, top:`${pos.y*100}%`,
        transform:'translate(-50%,-50%)', width:10, height:10, borderRadius:'50%',
        background:color, boxShadow:'0 0 0 2px white', pointerEvents:'none',
      }} />
      <div style={{
        position:'absolute', bottom:8, left:'50%', transform:'translateX(-50%)',
        background:'rgba(0,0,0,0.85)', color, fontFamily:'monospace', fontSize:12,
        padding:'4px 10px', borderRadius:6, whiteSpace:'nowrap', pointerEvents:'none',
        border:`1px solid ${color}44`,
      }}>
        {configKey}: {'{ '}x: {pos.x.toFixed(4)}, y: {pos.y.toFixed(4)}{' }'}
      </div>
    </div>
  )
}
