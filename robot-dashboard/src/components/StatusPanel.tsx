import { useConnected, useNT4Boolean, useNT4Number } from '../hooks/useNT4'
import { SHOOTER_MAX_RPM, TOPICS } from '../config'

function Row({ label, value, unit, highlight }: {
  label: string
  value: string
  unit?: string
  highlight?: 'good' | 'warn' | 'bad' | 'neutral'
}) {
  const highlightColor: Record<string, string> = {
    good:    '#22c55e',
    warn:    '#f59e0b',
    bad:     '#ef4444',
    neutral: '#94a3b8',
  }
  return (
    <tr>
      <td className="status-label">{label}</td>
      <td
        className="status-value"
        style={{ color: highlight ? highlightColor[highlight] : undefined }}
      >
        {value}
        {unit && <span className="status-unit"> {unit}</span>}
      </td>
    </tr>
  )
}

function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <>
      <tr className="status-section-header">
        <td colSpan={2}>{title}</td>
      </tr>
      {children}
    </>
  )
}

export function StatusPanel() {
  const connected         = useConnected()
  const shooterLeft       = useNT4Number(TOPICS.shooter.leftVelocityRpm)
  const shooterRight      = useNT4Number(TOPICS.shooter.rightVelocityRpm)
  const shooterTargetRPS  = useNT4Number(TOPICS.shooter.targetVelocityRadPerSec)
  const shooterAtSetpoint = useNT4Boolean(TOPICS.shooter.atSetpoint)
  const hoodPos           = useNT4Number(TOPICS.hood.positionRad)
  const hoodTarget        = useNT4Number(TOPICS.hood.targetPositionRad)
  const hoodHomed         = useNT4Boolean(TOPICS.hood.homed)
  const pivotPos          = useNT4Number(TOPICS.intakePivot.positionRad)
  const pivotTarget       = useNT4Number(TOPICS.intakePivot.targetPositionRad)
  const intakeRpm         = useNT4Number(TOPICS.intakeRollers.velocityRpm)
  const indexerRpm        = useNT4Number(TOPICS.indexer.velocityRpm)
  const tunnelRpm         = useNT4Number(TOPICS.tunnel.velocityRpm)
  const beamBreak         = useNT4Boolean(TOPICS.tunnel.beamBreak)

  const camLeftClimbConn      = useNT4Boolean(TOPICS.vision.leftClimbConnected)
  const camRightClimbConn     = useNT4Boolean(TOPICS.vision.rightClimbConnected)
  const camLeftShooterConn    = useNT4Boolean(TOPICS.vision.leftShooterConnected)
  const camRightShooterConn   = useNT4Boolean(TOPICS.vision.rightShooterConnected)
  const camLeftClimbTags      = useNT4Number(TOPICS.vision.leftClimbTargetCount)
  const camRightClimbTags     = useNT4Number(TOPICS.vision.rightClimbTargetCount)
  const camLeftShooterTags    = useNT4Number(TOPICS.vision.leftShooterTargetCount)
  const camRightShooterTags   = useNT4Number(TOPICS.vision.rightShooterTargetCount)

  const canTof               = useNT4Boolean(TOPICS.canStatus.tof)
  const canShooterTop        = useNT4Boolean(TOPICS.canStatus.shooterTop)
  const canShooterBottom     = useNT4Boolean(TOPICS.canStatus.shooterBottom)
  const canHoodMotor         = useNT4Boolean(TOPICS.canStatus.hoodMotor)
  const canHoodCANdi         = useNT4Boolean(TOPICS.canStatus.hoodCANdi)
  const canIndexer           = useNT4Boolean(TOPICS.canStatus.indexer)
  const canIntakeRoller      = useNT4Boolean(TOPICS.canStatus.intakeRoller)
  const canTunnel            = useNT4Boolean(TOPICS.canStatus.tunnel)
  const canIntakePivot       = useNT4Boolean(TOPICS.canStatus.intakePivot)
  const canClimbMotor        = useNT4Boolean(TOPICS.canStatus.climbMotor)
  const canClimbPivotMotor   = useNT4Boolean(TOPICS.canStatus.climbPivotMotor)
  const canClimbEncoder      = useNT4Boolean(TOPICS.canStatus.climbEncoder)
  const canClimbPivotEncoder = useNT4Boolean(TOPICS.canStatus.climbPivotEncoder)

  const shooterFraction = ((shooterLeft + shooterRight) / 2) / SHOOTER_MAX_RPM

  return (
    <div className="status-panel">
      <table className="status-table">
        <tbody>
          <Section title="Network">
            <Row label="NT4" value={connected ? 'Connected' : 'Disconnected'}
              highlight={connected ? 'good' : 'bad'} />
          </Section>

          <Section title="Shooter">
            <Row label="Left"  value={Math.round(shooterLeft).toString()}  unit="RPM"
              highlight={shooterLeft > 100 ? 'good' : 'neutral'} />
            <Row label="Right" value={Math.round(shooterRight).toString()} unit="RPM"
              highlight={shooterRight > 100 ? 'good' : 'neutral'} />
            <Row label="Target" value={Math.round(shooterTargetRPS * 60 / (2 * Math.PI)).toString()} unit="RPM" />
            <Row label="At setpoint" value={shooterAtSetpoint ? 'TRUE' : 'FALSE'}
              highlight={shooterAtSetpoint ? 'good' : 'neutral'} />
            <Row label="Speed %" value={`${(shooterFraction * 100).toFixed(0)}%`}
              highlight={shooterFraction > 0.9 ? 'good' : shooterFraction > 0.5 ? 'warn' : 'neutral'} />
          </Section>

          <Section title="Hood">
            <Row label="Position" value={`${toDeg(hoodPos).toFixed(1)}°`} />
            <Row label="Target"   value={`${toDeg(hoodTarget).toFixed(1)}°`} />
            <Row label="Error"    value={`${toDeg(hoodPos - hoodTarget).toFixed(1)}°`}
              highlight={Math.abs(hoodPos - hoodTarget) < 0.02 ? 'good' : 'warn'} />
            <Row label="Homed"    value={hoodHomed ? 'TRUE' : 'FALSE'}
              highlight={hoodHomed ? 'good' : 'bad'} />
          </Section>

          <Section title="Intake Pivot">
            <Row label="Position" value={`${toDeg(pivotPos).toFixed(1)}°`} />
            <Row label="Target"   value={`${toDeg(pivotTarget).toFixed(1)}°`} />
            <Row label="Error"    value={`${toDeg(pivotPos - pivotTarget).toFixed(1)}°`}
              highlight={Math.abs(pivotPos - pivotTarget) < 0.02 ? 'good' : 'warn'} />
          </Section>

          <Section title="Game Piece">
            <Row label="Beam break" value={beamBreak ? 'BLOCKED' : 'Clear'}
              highlight={beamBreak ? 'good' : 'neutral'} />
            <Row label="Intake"  value={Math.round(intakeRpm).toString()}  unit="RPM" />
            <Row label="Indexer" value={Math.round(indexerRpm).toString()} unit="RPM" />
            <Row label="Tunnel"  value={Math.round(tunnelRpm).toString()}  unit="RPM" />
          </Section>

          <Section title="Cameras">
            <Row label="Left Climb"    value={camLeftClimbConn    ? `OK · ${Math.round(camLeftClimbTags)}`    : 'MISSING'} highlight={camLeftClimbConn    ? 'good' : 'bad'} unit={camLeftClimbConn    ? 'tags' : undefined} />
            <Row label="Right Climb"   value={camRightClimbConn   ? `OK · ${Math.round(camRightClimbTags)}`   : 'MISSING'} highlight={camRightClimbConn   ? 'good' : 'bad'} unit={camRightClimbConn   ? 'tags' : undefined} />
            <Row label="Left Shooter"  value={camLeftShooterConn  ? `OK · ${Math.round(camLeftShooterTags)}`  : 'MISSING'} highlight={camLeftShooterConn  ? 'good' : 'bad'} unit={camLeftShooterConn  ? 'tags' : undefined} />
            <Row label="Right Shooter" value={camRightShooterConn ? `OK · ${Math.round(camRightShooterTags)}` : 'MISSING'} highlight={camRightShooterConn ? 'good' : 'bad'} unit={camRightShooterConn ? 'tags' : undefined} />
          </Section>

          <Section title="CAN Devices">
            <Row label="ToF Sensor"         value={canTof               ? 'OK' : 'MISSING'} highlight={canTof               ? 'good' : 'bad'} />
            <Row label="Shooter Top"        value={canShooterTop        ? 'OK' : 'MISSING'} highlight={canShooterTop        ? 'good' : 'bad'} />
            <Row label="Shooter Bottom"     value={canShooterBottom     ? 'OK' : 'MISSING'} highlight={canShooterBottom     ? 'good' : 'bad'} />
            <Row label="Hood Motor"         value={canHoodMotor         ? 'OK' : 'MISSING'} highlight={canHoodMotor         ? 'good' : 'bad'} />
            <Row label="Hood CANdi"         value={canHoodCANdi         ? 'OK' : 'MISSING'} highlight={canHoodCANdi         ? 'good' : 'bad'} />
            <Row label="Indexer"            value={canIndexer           ? 'OK' : 'MISSING'} highlight={canIndexer           ? 'good' : 'bad'} />
            <Row label="Intake Roller"      value={canIntakeRoller      ? 'OK' : 'MISSING'} highlight={canIntakeRoller      ? 'good' : 'bad'} />
            <Row label="Tunnel"             value={canTunnel            ? 'OK' : 'MISSING'} highlight={canTunnel            ? 'good' : 'bad'} />
            <Row label="Intake Pivot"       value={canIntakePivot       ? 'OK' : 'MISSING'} highlight={canIntakePivot       ? 'good' : 'bad'} />
            <Row label="Climb Motor"        value={canClimbMotor        ? 'OK' : 'MISSING'} highlight={canClimbMotor        ? 'good' : 'bad'} />
            <Row label="Climb Pivot Motor"  value={canClimbPivotMotor   ? 'OK' : 'MISSING'} highlight={canClimbPivotMotor   ? 'good' : 'bad'} />
            <Row label="Climb Encoder"      value={canClimbEncoder      ? 'OK' : 'MISSING'} highlight={canClimbEncoder      ? 'good' : 'bad'} />
            <Row label="Climb Pivot Enc."   value={canClimbPivotEncoder ? 'OK' : 'MISSING'} highlight={canClimbPivotEncoder ? 'good' : 'bad'} />
          </Section>
        </tbody>
      </table>
    </div>
  )
}

function toDeg(rad: number) {
  return rad * (180 / Math.PI)
}
