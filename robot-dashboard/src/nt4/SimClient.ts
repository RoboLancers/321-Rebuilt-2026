import { NT4Value } from './NT4Client'

type ValueCallback = (value: NT4Value, timestampUs: number) => void

/**
 * Drop-in replacement for NT4Client that generates animated fake data.
 * Toggle via SIM_MODE in config.ts — no robot connection needed.
 */
export class SimClient {
  private subs = new Map<string, Set<ValueCallback>>()
  private handle: ReturnType<typeof setInterval> | null = null
  private t = 0 // seconds elapsed

  constructor(
    _address: string,
    private readonly onConnectionChange: (connected: boolean) => void,
  ) {}

  connect() {
    this.onConnectionChange(true)
    // Tick at 50 ms to match the real client's update rate.
    this.handle = setInterval(() => this.tick(), 50)
  }

  disconnect() {
    if (this.handle) clearInterval(this.handle)
    this.onConnectionChange(false)
  }

  subscribe(topic: string, cb: ValueCallback): () => void {
    if (!this.subs.has(topic)) this.subs.set(topic, new Set())
    this.subs.get(topic)!.add(cb)
    return () => this.subs.get(topic)?.delete(cb)
  }

  // ── Simulation tick ──────────────────────────────────────────────────────

  private tick() {
    this.t += 0.05

    const now = Date.now() * 1000 // microseconds

    // Shooter: ramps up to ~5500 RPM over 4 s, holds, drops, repeats.
    const shooterPhase = (this.t % 8) / 8
    const shooterRpm   = shooterPhase < 0.5
      ? ease(shooterPhase / 0.5) * 5500
      : 5500 - ease((shooterPhase - 0.5) / 0.5) * 5500
    this.emit('/Robot/shooter/leftVelocityRotationsPerMinute',  shooterRpm + rand(30), now)
    this.emit('/Robot/shooter/rightVelocityRotationsPerMinute', shooterRpm + rand(30), now)
    this.emit('/Robot/shooter/atSetpoint', shooterRpm > 5000, now)

    // Hood: sweeps 0.52 rad → 1.05 rad (30°→60°) slowly.
    const hoodPos    = 0.52 + 0.53 * (0.5 + 0.5 * Math.sin(this.t * 0.4))
    const hoodTarget = 0.52 + 0.53 * (0.5 + 0.5 * Math.sin(this.t * 0.4 - 0.2))
    this.emit('/Robot/hood/positionRadians',       hoodPos,    now)
    this.emit('/Robot/hood/targetPositionRadians', hoodTarget, now)
    this.emit('/Robot/hood/homed', true, now)

    // Intake pivot: sweeps 0 → 1.4 rad (deployed) and back, period ~6 s.
    const pivotPos    = 0.7 + 0.7 * Math.sin(this.t * 0.6)
    const pivotTarget = 0.7 + 0.7 * Math.sin(this.t * 0.6 - 0.15)
    this.emit('/Robot/intakePivot/positionRadians',       Math.max(0, pivotPos),    now)
    this.emit('/Robot/intakePivot/targetPositionRadians', Math.max(0, pivotTarget), now)

    // Intake rollers: spin when pivot is deployed.
    const intakeActive = pivotPos > 0.5
    this.emit('/Robot/intakeRollers/velocityRotationsPerMinute', intakeActive ? 2200 + rand(50) : 0, now)

    // Indexer: pulses when beam break triggers.
    const beamBreak = Math.sin(this.t * 0.7) > 0.6
    this.emit('/Robot/tunnel/beamBreak', beamBreak, now)
    this.emit('/Robot/indexer/velocityRotationsPerMinute', beamBreak ? 1800 + rand(40) : 0, now)
    this.emit('/Robot/tunnel/velocityRotationsPerMinute',  beamBreak ? 1600 + rand(40) : 0, now)
  }

  private emit(topic: string, value: NT4Value, timestamp: number) {
    const callbacks = this.subs.get(topic)
    if (!callbacks) return
    for (const cb of callbacks) cb(value, timestamp)
  }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

/** Smooth ease-in-out (cubic). */
function ease(t: number) {
  return t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t
}

/** Small random jitter. */
function rand(mag: number) {
  return (Math.random() - 0.5) * mag
}
