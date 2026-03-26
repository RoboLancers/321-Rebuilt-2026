function hexToRgb(hex: string): [number, number, number] {
  const n = parseInt(hex.replace('#', ''), 16)
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255]
}

/** Linearly interpolate between two hex colors at fraction t (0–1). */
export function lerpColor(colorA: string, colorB: string, t: number): string {
  const a = hexToRgb(colorA)
  const b = hexToRgb(colorB)
  const clamp = (v: number) => Math.round(Math.min(Math.max(v, 0), 255))
  const tt = Math.min(Math.max(t, 0), 1)
  return `rgb(${clamp(a[0] + (b[0] - a[0]) * tt)},${clamp(a[1] + (b[1] - a[1]) * tt)},${clamp(a[2] + (b[2] - a[2]) * tt)})`
}

/**
 * Map a numeric value in [min, max] onto a color gradient.
 * Values outside the range are clamped.
 *
 * Example: rangeColor(rpm, 0, 6000, '#3b82f6', '#22c55e')
 *   → blue at 0 RPM, green at 6000 RPM
 */
export function rangeColor(
  value: number,
  min: number,
  max: number,
  colorLow: string,
  colorHigh: string,
): string {
  return lerpColor(colorLow, colorHigh, (value - min) / (max - min))
}

/** Green when active, gray when idle, red when faulted. */
export function stateColor(state: 'active' | 'idle' | 'fault'): string {
  return { active: '#22c55e', idle: '#6b7280', fault: '#ef4444' }[state]
}

/** Simple boolean → green/red. */
export function boolColor(value: boolean, trueColor = '#22c55e', falseColor = '#6b7280'): string {
  return value ? trueColor : falseColor
}
