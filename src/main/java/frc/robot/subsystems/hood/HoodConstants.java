/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HoodConstants {

  public static final double kP = 0.637;
  public static final double kD = 0;
  public static final double kG = 0.33;

  public static final int kHoodMotorId = 11;

  public static final int kHoodEncoderId = 0;

  public static final double kHoodGearRatio = 1000 / 7;

  public static final boolean kHoodMotorInverted = true;

  public static final Current kHoodStatorCurrentLimit = Amps.of(60);

  public static final Current kHoodSupplyCurrentLimit = Amps.of(60);

  public static final AngularVelocity kHoodMotorMaxVelocity = RotationsPerSecond.of(0.16);

  public static final AngularAcceleration kHoodMotorMaxAcceleration =
      RotationsPerSecondPerSecond.of(0.16);

  public static final Angle kStartingAngle = Degrees.of(0);

  public static final Angle kAngleTolerance = Degrees.of(0.5);

  public static final Angle kSetScoreAngle = Degrees.of(0);

  public static final Angle kRegion1ScoreAngle = Degrees.of(0);

  public static final Angle kRegion2ScoreAngle = Degrees.of(0);

  public static final Angle kRegion3ScoreAngle = Degrees.of(0);

  public static final Angle kNeutralFeedAngle = Degrees.of(15);

  public static final Angle kOppositeFeedAngle = Degrees.of(0);

  public static final Angle kReleaseAngle = Degrees.of(0);

  public static final Angle kTravelAngle = Degrees.of(0);

  public static final Voltage kHomingVoltage = Volts.of(-2);

  public static final Current kCurrentCeiling = Amps.of(20);

  public static final AngularVelocity kHomingVelocityFloor = RPM.of(1);

  public static final double kHomingVelocityTolerance = 2;

  public static final int kLimitSwitchID = 1;

  public static final Angle kZeroPosition = Degrees.of(0);
}
