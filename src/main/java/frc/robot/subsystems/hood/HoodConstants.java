/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class HoodConstants {

  public static final double kP = 0;
  public static final double kD = 0;
  public static final double kG = 0;

  public static final int kHoodMotorId = 1;

  public static final int kHoodEncoderId = 0;

  public static final boolean kHoodMotorInverted = false;

  public static final Current kHoodStatorCurrentLimit = Amps.of(40);

  public static final Current kHoodSupplyCurrentLimit = Amps.of(40);

  public static final LinearVelocity kHoodMotorMaxVelocity = MetersPerSecond.of(1);

  public static final LinearAcceleration kHoodMotorMaxAcceleration = MetersPerSecondPerSecond.of(1);

  public static final Angle kStartingAngle = Degrees.of(60);

  public static final Angle kAngleTolerance = Degrees.of(1);

  public static final Angle kSetScoreAngle = Degrees.of(0);

  public static final Angle kRegion1ScoreAngle = Degrees.of(0);

  public static final Angle kRegion2ScoreAngle = Degrees.of(0);

  public static final Angle kRegion3ScoreAngle = Degrees.of(0);

  public static final Angle kNeutralFeedAngle = Degrees.of(0);

  public static final Angle kOppositeFeedAngle = Degrees.of(0);

  public static final Angle kReleaseAngle = Degrees.of(0);

  public static final Angle kTravelAngle = Degrees.of(0);
}
