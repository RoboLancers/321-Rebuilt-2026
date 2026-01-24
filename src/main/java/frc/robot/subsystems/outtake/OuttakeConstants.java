/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class OuttakeConstants {

  public static final int kMotorID = 0;

  public static final double kStatorLimit = 40;

  public static final boolean kInverted = false;

  public static final double kGearing = 1;

  public static final double kNominalVoltage = 12;

  public static final AngularVelocity kNeutralFeedRPM = RPM.of(0);

  public static final AngularVelocity kOppositeFeedRPM = RPM.of(0);

  public static final AngularVelocity kRegion1ScoreRPM = RPM.of(0);

  public static final AngularVelocity kRegion2ScoreRPM = RPM.of(0);

  public static final AngularVelocity kRegion3ScoreRPM = RPM.of(0);

  public static final AngularVelocity kSetPositionScoreRPM = RPM.of(0);

  public static final AngularVelocity kReleaseRPM = RPM.of(0);
}
