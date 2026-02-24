/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class OuttakeConstants {

  public static final double kP = 0;

  public static final double kD = 0;

  public static final double kV = 0;

  public static final int kMotorID = 0;

  public static final Current kStatorLimit = Amps.of(40);

  public static final Current kSupplyLimit = Amps.of(40);

  public static final boolean kInverted = false;

  public static final double kGearing = 2;

  public static final double kNominalVoltage = 12;

  public static final AngularVelocity kNeutralFeedRPM = RPM.of(0);

  public static final AngularVelocity kOppositeFeedRPM = RPM.of(0);

  public static final AngularVelocity kRegion1ScoreRPM = RPM.of(0);

  public static final AngularVelocity kRegion2ScoreRPM = RPM.of(0);

  public static final AngularVelocity kRegion3ScoreRPM = RPM.of(0);

  public static final AngularVelocity kSetPositionScoreRPM = RPM.of(0);

  public static final AngularVelocity kReleaseRPM = RPM.of(0);

  public static final AngularVelocity kMaxVelocity = RPM.of(3000);

  public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(20);

  public static final int kBeamBreakID = 0;
}
