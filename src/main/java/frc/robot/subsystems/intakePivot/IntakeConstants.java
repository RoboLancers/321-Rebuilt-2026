/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
  public static final int kPivotMotorId = 16;
  public static final Current kCurrentLimit = Amps.of(60);
  public static final boolean kCurrentLimitEnable = true;
  public static final boolean kInverted = true;
  public static final double kG = 0;
  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kSensorToMechanismRatio = 72;
  public static final double kRelativeEncoderId = 0;
  public static final Angle kIntakePosition = Degrees.of(0);
  public static final Angle kTravelPosition = Degrees.of(30);
  public static final Angle kStowedPosition = Degrees.of(120);
  public static final int kEncoderID = 5;
  public static final Voltage kHomingVoltage = Volts.of(-2.0);
  public static final Angle kAngleTolerance = Degrees.of(1.5);
  public static final AngularVelocity kMaxVelocity = RadiansPerSecond.of(Math.PI);
  public static final AngularAcceleration kMaxAcceleration = RadiansPerSecondPerSecond.of(Math.PI);
  public static final Constraints kMaxPivotConstraints =
      new Constraints(
          kMaxVelocity.in(RadiansPerSecond), kMaxAcceleration.in(RadiansPerSecondPerSecond));
  public static final AngularVelocity kHomingVelocity = RPM.of(10);
  public static final Current kHomingCurrent = Amps.of(40);
}
