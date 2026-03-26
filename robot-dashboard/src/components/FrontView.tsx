import { useCallback, useEffect, useRef, useState } from 'react'
import { useConnected, useNT4Boolean, useNT4Number } from '../hooks/useNT4'
import { CALIBRATE_ANCHORS, HOOD_ARM, TOPICS } from '../config'
import { boolColor } from '../utils/colorMap'
import { DotOverlay, TextOverlay } from './Overlays'
import { AnchorDef, AnchorHandles, AnchorMap, AnchorPanel } from './AnchorCalibrator'

// Overlay anchor positions as percentages of the ROBOT IMAGE (not the wrapper).
// Set these using CALIBRATE_ANCHORS mode, then paste the output here.
const ANCHORS = {
  shooterRpm: { x: 28.5, y: 22.3 },
  hoodPivot: { x: 46.6, y: 22.0 },
  hoodAngleLabel: { x: 80.4, y: 14.9 },
  intakeRoller: { x: 50.1, y: 89.0 },
  tunnelBeamBreak: { x: 56.8, y: 51.5 },
  tunnelRpm: { x: 80.7, y: 42.9 },
}

const ANCHOR_DEFS: AnchorDef[] = [
  { key: 'shooterRpm',      label: 'Shooter RPM',   color: '#22c55e' },
  { key: 'hoodPivot',       label: 'Hood Pivot',    color: '#f59e0b' },
  { key: 'hoodAngleLabel',  label: 'Hood Angle',    color: '#38bdf8' },
  { key: 'intakeRoller',    label: 'Intake Roller', color: '#a78bfa' },
  { key: 'tunnelBeamBreak', label: 'Beam Break',    color: '#f97316' },
  { key: 'tunnelRpm',       label: 'Tunnel RPM',    color: '#e879f9' },
]

interface Bounds { left: number; top: number; width: number; height: number }

export function FrontView() {
  const connected         = useConnected()
  const shooterLeftRpm    = useNT4Number(TOPICS.shooter.leftVelocityRpm)
  const shooterRightRpm   = useNT4Number(TOPICS.shooter.rightVelocityRpm)
  const shooterTargetRPS  = useNT4Number(TOPICS.shooter.targetVelocityRadPerSec)
  const shooterAtSetpoint = useNT4Boolean(TOPICS.shooter.atSetpoint)
  const hoodPositionRad   = useNT4Number(TOPICS.hood.positionRad)
  const hoodTargetRad     = useNT4Number(TOPICS.hood.targetPositionRad)
  const hoodHomed         = useNT4Boolean(TOPICS.hood.homed)
  const intakeRpm         = useNT4Number(TOPICS.intakeRollers.velocityRpm)
  const tunnelRpm         = useNT4Number(TOPICS.tunnel.velocityRpm)
  const beamBreak         = useNT4Boolean(TOPICS.tunnel.beamBreak)

  const avgShooterRpm    = (shooterLeftRpm + shooterRightRpm) / 2
  const hoodDeg          = HOOD_ARM.cssRotationAtZero + hoodPositionRad * HOOD_ARM.cssDegreesPerRad
  const shooterTargetRpm = shooterTargetRPS * 60 / (2 * Math.PI)
  const beamColor        = boolColor(beamBreak, '#22c55e', '#6b7280')

  const shooterValueColor = !connected ? '#ef4444'
    : shooterTargetRpm < 100 ? '#6b7280'
    : shooterAtSetpoint ? '#22c55e' : '#ef4444'

  const hoodValueColor = !connected ? '#ef4444'
    : !hoodHomed ? '#ef4444'
    : Math.abs(hoodPositionRad - hoodTargetRad) < 0.05 ? '#22c55e' : '#ef4444'

  const intakeValueColor = !connected ? '#ef4444'
    : Math.abs(intakeRpm) < 50 ? '#6b7280' : '#22c55e'

  const tunnelValueColor = !connected ? '#ef4444'
    : Math.abs(tunnelRpm) < 50 ? '#6b7280' : '#22c55e'

  const wrapperRef = useRef<HTMLDivElement>(null)
  const baseImgRef = useRef<HTMLImageElement>(null)
  const [imageBounds,  setImageBounds]  = useState<Bounds | null>(null)
  const [calibAnchors, setCalibAnchors] = useState<AnchorMap>(() => ({ ...ANCHORS }))

  const updateLayout = useCallback(() => {
    const img     = baseImgRef.current
    const wrapper = wrapperRef.current
    if (!img || !wrapper) return
    const iRect = img.getBoundingClientRect()
    const wRect = wrapper.getBoundingClientRect()
    setImageBounds({
      left: iRect.left - wRect.left, top:    iRect.top - wRect.top,
      width: iRect.width,            height: iRect.height,
    })
  }, [])

  useEffect(() => {
    window.addEventListener('resize', updateLayout)
    return () => window.removeEventListener('resize', updateLayout)
  }, [updateLayout])

  return (
    <div className="view-card">
      <h2 className="view-title">
        Front View
        {CALIBRATE_ANCHORS && <span className="calibrate-badge">CALIBRATING ANCHORS</span>}
      </h2>

      <div className="robot-image-wrapper" ref={wrapperRef}>
        <img ref={baseImgRef} src="/images/front-view.png" className="robot-base-image"
          alt="Robot — front view" draggable={false} onLoad={updateLayout} />

        {imageBounds && (
          // Image-bound container — all x/y are percentages of the image
          <div style={{
            position: 'absolute',
            left: imageBounds.left, top:    imageBounds.top,
            width: imageBounds.width, height: imageBounds.height,
          }}>
            {CALIBRATE_ANCHORS ? (
              <AnchorHandles
                anchors={calibAnchors}
                defs={ANCHOR_DEFS}
                onAnchorsChange={setCalibAnchors}
              />
            ) : (
              <>
                <TextOverlay
                  x={ANCHORS.shooterRpm.x} y={ANCHORS.shooterRpm.y}
                  label="Shooter" value={`${Math.round(avgShooterRpm)} RPM`} color={shooterValueColor}
                />
                {/* Hood bar — pivots from left edge at hoodPivot anchor */}
                <div style={{
                  position: 'absolute',
                  left: `${ANCHORS.hoodPivot.x}%`,
                  top: `${ANCHORS.hoodPivot.y}%`,
                  width: `${HOOD_ARM.widthPercent}%`,
                  height: 10,
                  background: hoodValueColor,
                  borderRadius: 5,
                  transformOrigin: '0% 50%',
                  transform: `rotate(${hoodDeg}deg)`,
                  transition: 'transform 0.08s linear',
                  pointerEvents: 'none',
                  boxShadow: `0 0 8px ${hoodValueColor}88`,
                }} />
                <TextOverlay
                  x={ANCHORS.hoodAngleLabel.x} y={ANCHORS.hoodAngleLabel.y}
                  label="Hood"
                  value={`${toDeg(hoodPositionRad).toFixed(1)}°`}
                  note={`Homed: ${hoodHomed ? 'TRUE' : 'FALSE'}`}
                  color={hoodValueColor}
                />
                <TextOverlay
                  x={ANCHORS.intakeRoller.x} y={ANCHORS.intakeRoller.y}
                  label="Intake" value={`${Math.round(intakeRpm)} RPM`} color={intakeValueColor}
                />
                <DotOverlay
                  x={ANCHORS.tunnelBeamBreak.x} y={ANCHORS.tunnelBeamBreak.y}
                  color={beamColor} tooltip={beamBreak ? 'Game piece present' : 'Tunnel empty'}
                />
                <TextOverlay
                  x={ANCHORS.tunnelRpm.x} y={ANCHORS.tunnelRpm.y}
                  label="Tunnel" value={`${Math.round(tunnelRpm)} RPM`} color={tunnelValueColor}
                />
              </>
            )}
          </div>
        )}
      </div>

      {/* Code panel — below the image, inside this view card */}
      {CALIBRATE_ANCHORS && (
        <AnchorPanel anchors={calibAnchors} defs={ANCHOR_DEFS} constName="ANCHORS" />
      )}
    </div>
  )
}

function toDeg(rad: number) { return rad * (180 / Math.PI) }
