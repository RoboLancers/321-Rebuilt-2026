/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
  public static final int kPivotMotorId = 16;
  public static final Current kCurrentLimit = Amps.of(40);
  public static final boolean kCurrentLimitEnable = true;
  public static final boolean kInverted = true;
  public static double kG = 0;
  public static double kP = 0;
  public static double kD = 0;
  public static Angle angle;
  public static final double kSensorToMechanismRatio = 11 / 4;
  public static final double kRelativeEncoderId = 0;
  public static final Angle kIntakePosition = Degrees.of(0);
  public static final Angle kDefaultPosition = Degrees.of(0);
  public static final int kEncoderID = 0;
  public static final Angle kAngleTolerance = Degrees.of(0);
  public static final Voltage kHomingVoltage = Volts.of(-8.0);
  public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(0.125);
}
