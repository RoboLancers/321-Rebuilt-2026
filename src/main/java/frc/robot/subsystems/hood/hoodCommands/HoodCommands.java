/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood.hoodCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import java.util.function.Supplier;

public class HoodCommands {

  public static Command goToAngle(Hood hood, Supplier<Angle> angle) {
    return Commands.run(() -> hood.goToAngle(angle.get()), hood)
        .until(
            () ->
                Math.abs(angle.get().in(Degrees) - hood.getAngle().in(Degrees))
                    <= HoodConstants.kAngleTolerance.in(Degrees));
  }

  public static Command runVolts(Hood hood, Supplier<Voltage> volts) {
    return Commands.run(() -> hood.runVolts(volts.get()), hood);
  }

  public static Command goToScoringAngle(Hood hood) {
    return goToAngle(hood, () -> HoodConstants.kSetScoreAngle);
  }

  public static Command goToRegion1Angle(Hood hood) {
    return goToAngle(hood, () -> HoodConstants.kRegion1ScoreAngle);
  }

  public static Command goToRegion2Angle(Hood hood) {
    return goToAngle(hood, () -> HoodConstants.kRegion2ScoreAngle);
  }

  public static Command goToRegion3Angle(Hood hood) {
    return goToAngle(hood, () -> HoodConstants.kRegion3ScoreAngle);
  }

  public static Command goToNeutralFeedAngle(Hood hood) {
    return goToAngle(hood, () -> HoodConstants.kNeutralFeedAngle);
  }

  public static Command goToOppositeFeedAngle(Hood hood) {
    return goToAngle(hood, () -> HoodConstants.kOppositeFeedAngle);
  }

  public static Command goToReleaseAngle(Hood hood) {
    return goToAngle(hood, () -> HoodConstants.kReleaseAngle);
  }

  public static Command goToTravelAngle(Hood hood) {
    return goToAngle(hood, () -> HoodConstants.kTravelAngle);
  }

  public static Command homeHoodVelocity(Hood hood) {
    return runVolts(hood, () -> HoodConstants.kHomingVoltage)
        .until(() -> hood.isHomedVelocity())
        .andThen(() -> hood.zeroEncoder(HoodConstants.kZeroPosition));
  }

  public static Command homeHoodCurrent(Hood hood) {
    return runVolts(hood, () -> HoodConstants.kHomingVoltage)
        .until(() -> hood.isHomedCurrent())
        .andThen(() -> hood.zeroEncoder(HoodConstants.kZeroPosition));
  }

  public static Command homeHoodMagnetic(Hood hood) {
    return runVolts(hood, () -> HoodConstants.kHomingVoltage)
        .until(() -> hood.getHoodAtHomedPosition())
        .andThen(() -> hood.zeroEncoder(HoodConstants.kZeroPosition));
  }
}
