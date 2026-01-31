/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.Shooter;
import java.util.function.Supplier;

public class ShootFuel {

  public static Command outtakeWithVoltage(Shooter outtake, Supplier<Voltage> volts) {
    return outtake.runVolts(volts.get());
  }

  public static Command outtakeWithVelocity(Shooter outtake, Supplier<AngularVelocity> rpm) {
    return outtake.setControl(rpm.get());
  }

  public static Command feedNeutralZone(Shooter outtake) {
    return outtakeWithVelocity(outtake, () -> OuttakeConstants.kNeutralFeedRPM);
  }

  public static Command feedOppositeZone(Shooter outtake) {
    return outtakeWithVelocity(outtake, () -> OuttakeConstants.kOppositeFeedRPM);
  }

  public static Command scoreRegion1(Shooter outtake) {
    return outtakeWithVelocity(outtake, () -> OuttakeConstants.kRegion1ScoreRPM);
  }

  public static Command scoreRegion2(Shooter outtake) {
    return outtakeWithVelocity(outtake, () -> OuttakeConstants.kRegion2ScoreRPM);
  }

  public static Command scoreRegion3(Shooter outtake) {
    return outtakeWithVelocity(outtake, () -> OuttakeConstants.kRegion3ScoreRPM);
  }

  public static Command scoreSetPosition(Shooter outtake) {
    return outtakeWithVelocity(outtake, () -> OuttakeConstants.kSetPositionScoreRPM);
  }

  public static Command releaseFuel(Shooter outtake) {
    return outtakeWithVelocity(outtake, () -> OuttakeConstants.kReleaseRPM);
  }
}
