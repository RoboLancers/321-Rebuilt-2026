import { useCallback, useEffect, useRef, useState } from 'react'
import { useConnected, useNT4Boolean, useNT4Number } from '../hooks/useNT4'
import {
  CALIBRATE_ANCHORS, CALIBRATE_ARM_PIVOT, CALIBRATE_PIVOT,
  INTAKE_ARM, TOPICS,
} from '../config'
import { boolColor } from '../utils/colorMap'
import { DotOverlay, TextOverlay } from './Overlays'
import { PivotCalibrator } from './PivotCalibrator'
import { AnchorDef, AnchorHandles, AnchorMap, AnchorPanel } from './AnchorCalibrator'

const ANCHORS = {
  shooterFlywheel: { x: 41.1, y: 22.4 },
  tunnelBeamBreak: { x: 22.6, y: 59.5 },
  indexerRoller: { x: 46.8, y: 68.7 },
  pivotAngleLabel: { x: 66.2, y: 68.2 },
  intakeRpmLabel: { x: 95.1, y: 85.5 },
}

const ANCHOR_DEFS: AnchorDef[] = [
  { key: 'shooterFlywheel', label: 'Shooter RPM', color: '#22c55e' },
  { key: 'tunnelBeamBreak', label: 'Beam Break',  color: '#f59e0b' },
  { key: 'indexerRoller',   label: 'Indexer RPM', color: '#f59e0b' },
  { key: 'pivotAngleLabel', label: 'Pivot Angle', color: '#38bdf8' },
  { key: 'intakeRpmLabel',  label: 'Intake RPM',  color: '#a78bfa' },
]

interface Bounds    { left: number; top: number; width: number; height: number }
interface ArmLayout { left: number; top: number; width: number; height: number }

const pivotCalibrating = CALIBRATE_PIVOT || CALIBRATE_ARM_PIVOT

export function SideView() {
  const connected         = useConnected()
  const shooterLeftRpm    = useNT4Number(TOPICS.shooter.leftVelocityRpm)
  const shooterRightRpm   = useNT4Number(TOPICS.shooter.rightVelocityRpm)
  const shooterTargetRPS  = useNT4Number(TOPICS.shooter.targetVelocityRadPerSec)
  const shooterAtSetpoint = useNT4Boolean(TOPICS.shooter.atSetpoint)
  const intakePivotRad    = useNT4Number(TOPICS.intakePivot.positionRad)
  const intakeTargetRad   = useNT4Number(TOPICS.intakePivot.targetPositionRad)
  const intakeRpm         = useNT4Number(TOPICS.intakeRollers.velocityRpm)
  const indexerRpm        = useNT4Number(TOPICS.indexer.velocityRpm)
  const beamBreak         = useNT4Boolean(TOPICS.tunnel.beamBreak)

  const avgShooterRpm    = (shooterLeftRpm + shooterRightRpm) / 2
  const shooterTargetRpm = shooterTargetRPS * 60 / (2 * Math.PI)
  const beamColor        = boolColor(beamBreak, '#22c55e', '#6b7280')

  const shooterValueColor = !connected ? '#ef4444'
    : shooterTargetRpm < 100 ? '#6b7280'
    : shooterAtSetpoint ? '#22c55e' : '#ef4444'

  const indexerValueColor = !connected ? '#ef4444'
    : Math.abs(indexerRpm) < 50 ? '#6b7280' : '#22c55e'

  const pivotValueColor = !connected ? '#ef4444'
    : Math.abs(intakePivotRad - intakeTargetRad) < 0.05 ? '#22c55e' : '#ef4444'

  const intakeValueColor = !connected ? '#ef4444'
    : Math.abs(intakeRpm) < 50 ? '#6b7280' : '#22c55e'
  const armDeg = INTAKE_ARM.cssRotationAtZero + intakePivotRad * INTAKE_ARM.cssDegreesPerRad
  const { pivotFraction: pf, armPivotFraction: apf } = INTAKE_ARM

  const wrapperRef = useRef<HTMLDivElement>(null)
  const baseImgRef = useRef<HTMLImageElement>(null)
  const armImgRef  = useRef<HTMLImageElement>(null)

  const [robotBounds,    setRobotBounds]    = useState<Bounds | null>(null)
  const [armLayout,      setArmLayout]      = useState<ArmLayout | null>(null)
  const [armCalibBounds, setArmCalibBounds] = useState<Bounds | null>(null)
  const [calibAnchors,   setCalibAnchors]   = useState<AnchorMap>(() => ({ ...ANCHORS }))

  const updateLayout = useCallback(() => {
    const wrapper = wrapperRef.current
    if (!wrapper) return
    const wRect = wrapper.getBoundingClientRect()

    if (CALIBRATE_ARM_PIVOT) {
      const armImg = armImgRef.current
      if (!armImg) return
      const aRect = armImg.getBoundingClientRect()
      setArmCalibBounds({
        left: aRect.left - wRect.left, top:  aRect.top - wRect.top,
        width: aRect.width,            height: aRect.height,
      })
      return
    }

    const robotImg = baseImgRef.current
    const armImg   = armImgRef.current
    if (!robotImg || !armImg) return

    const rRect     = robotImg.getBoundingClientRect()
    const robotLeft = rRect.left - wRect.left
    const robotTop  = rRect.top  - wRect.top
    const robotW    = rRect.width
    const robotH    = rRect.height

    setRobotBounds({ left: robotLeft, top: robotTop, width: robotW, height: robotH })

    const scale   = robotW / (robotImg.naturalWidth || 1)
    const armW    = (armImg.naturalWidth  || 1) * scale
    const armH    = (armImg.naturalHeight || 1) * scale
    const pivotWX = robotLeft + pf.x * robotW
    const pivotWY = robotTop  + pf.y * robotH

    setArmLayout({
      left: pivotWX - apf.x * armW, top:    pivotWY - apf.y * armH,
      width: armW,                   height: armH,
    })
  }, [pf, apf])

  useEffect(() => {
    window.addEventListener('resize', updateLayout)
    return () => window.removeEventListener('resize', updateLayout)
  }, [updateLayout])

  const calBadge = CALIBRATE_ARM_PIVOT ? 'CALIBRATING ARM PIVOT'
    : CALIBRATE_PIVOT                  ? 'CALIBRATING ROBOT PIVOT'
    : CALIBRATE_ANCHORS                ? 'CALIBRATING ANCHORS'
    : null

  return (
    <div className="view-card">
      <h2 className="view-title">
        Side View
        {calBadge && <span className="calibrate-badge">{calBadge}</span>}
      </h2>

      <div className="robot-image-wrapper" ref={wrapperRef}>

        {/* ── Arm-pivot calibration: arm image alone ── */}
        {CALIBRATE_ARM_PIVOT && (
          <>
            <img ref={armImgRef} src="/images/side-arm.png" className="robot-base-image"
              alt="Arm" draggable={false} onLoad={updateLayout} />
            {armCalibBounds && (
              <PivotCalibrator initialX={apf.x} initialY={apf.y}
                bounds={armCalibBounds} color="#38bdf8" configKey="armPivotFraction" />
            )}
          </>
        )}

        {/* ── Normal / pivot-calibration / anchor-calibration modes ── */}
        {!CALIBRATE_ARM_PIVOT && (
          <>
            <img ref={baseImgRef} src="/images/side-view.png" className="robot-base-image"
              alt="Robot — side view" draggable={false} onLoad={updateLayout} />

            <img ref={armImgRef} src="/images/side-arm.png" alt="" draggable={false}
              onLoad={updateLayout}
              style={{
                position:        'absolute',
                visibility:      armLayout ? 'visible' : 'hidden',
                left:            armLayout?.left   ?? 0,
                top:             armLayout?.top    ?? 0,
                width:           armLayout?.width  ?? 0,
                height:          armLayout?.height ?? 0,
                transformOrigin: `${apf.x * 100}% ${apf.y * 100}%`,
                transform:       `rotate(${pivotCalibrating || CALIBRATE_ANCHORS ? 0 : armDeg}deg)`,
                transition:      pivotCalibrating ? 'none' : 'transform 0.08s linear',
                pointerEvents:   'none',
              }}
            />

            {CALIBRATE_PIVOT && robotBounds && (
              <PivotCalibrator initialX={pf.x} initialY={pf.y}
                bounds={robotBounds} color="#f97316" configKey="pivotFraction" />
            )}

            {/* Anchor handles inside image-bound div */}
            {CALIBRATE_ANCHORS && robotBounds && (
              <div style={{
                position: 'absolute',
                left: robotBounds.left, top:    robotBounds.top,
                width: robotBounds.width, height: robotBounds.height,
              }}>
                <AnchorHandles
                  anchors={calibAnchors}
                  defs={ANCHOR_DEFS}
                  onAnchorsChange={setCalibAnchors}
                />
              </div>
            )}

            {/* Live overlays */}
            {!pivotCalibrating && !CALIBRATE_ANCHORS && robotBounds && (
              <div style={{
                position: 'absolute',
                left: robotBounds.left, top:    robotBounds.top,
                width: robotBounds.width, height: robotBounds.height,
                pointerEvents: 'none',
              }}>
                <TextOverlay
                  x={ANCHORS.shooterFlywheel.x} y={ANCHORS.shooterFlywheel.y}
                  label="Shooter" value={`${Math.round(avgShooterRpm)} RPM`} color={shooterValueColor}
                />
                <DotOverlay
                  x={ANCHORS.tunnelBeamBreak.x} y={ANCHORS.tunnelBeamBreak.y}
                  color={beamColor} tooltip={beamBreak ? 'Game piece present' : 'Tunnel empty'}
                />
                <TextOverlay
                  x={ANCHORS.indexerRoller.x} y={ANCHORS.indexerRoller.y}
                  label="Indexer" value={`${Math.round(indexerRpm)} RPM`} color={indexerValueColor}
                />
                <TextOverlay
                  x={ANCHORS.pivotAngleLabel.x} y={ANCHORS.pivotAngleLabel.y}
                  label="Pivot"
                  value={`${toDeg(intakePivotRad).toFixed(1)}°`}
                  color={pivotValueColor}
                />
                <TextOverlay
                  x={ANCHORS.intakeRpmLabel.x} y={ANCHORS.intakeRpmLabel.y}
                  label="Intake" value={`${Math.round(intakeRpm)} RPM`}
                  color={intakeValueColor} align="right"
                />
              </div>
            )}
          </>
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
