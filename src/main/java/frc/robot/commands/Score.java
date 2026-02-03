/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.hoodCommands.HoodCommands;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.outtake.commands.ShootFuel;
import frc.robot.util.MyAlliance;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Score {

  private static final Pose2d redHubPose = new Pose2d(0, 0, Rotation2d.kZero);
  private static final Pose2d blueHubPose = new Pose2d(0, 0, Rotation2d.kZero);

  private static final Distance region1 = Meters.of(0);
  private static final Distance region2 = Meters.of(0);
  private static final Distance region3 = Meters.of(0);

  public static Command scoreFuelFromPose(
      Drivetrain drivetrain, Shooter shooter, Supplier<Pose2d> robotPose) {
    return Align.driveToHubScoringPose(drivetrain, robotPose)
        .andThen(ShootFuel.scoreSetPosition(shooter));
  }

  public static Distance getHubDistance(Supplier<Pose2d> robotPose) {
    Pose2d hubPose = null;

    if (MyAlliance.isRed()) {
      hubPose = redHubPose;
    } else {
      hubPose = blueHubPose;
    }

    return Meters.of(robotPose.get().getTranslation().getDistance(hubPose.getTranslation()));
  }

  public static AngularVelocity getScoreVelocity(Supplier<Pose2d> robotPose) {
    AngularVelocity velocity = RPM.of(0);
    if (0 < getHubDistance(robotPose).in(Meters)
        && getHubDistance(robotPose).in(Meters) < region1.in(Meters)) {
      velocity = OuttakeConstants.kRegion1ScoreRPM;
    } else if (region1.in(Meters) < getHubDistance(robotPose).in(Meters)
        && getHubDistance(robotPose).in(Meters) < region2.in(Meters)) {
      velocity = OuttakeConstants.kRegion2ScoreRPM;
    } else if (region2.in(Meters) < getHubDistance(robotPose).in(Meters)) {
      velocity = OuttakeConstants.kRegion3ScoreRPM;
    }

    return velocity;
  }

  public static Angle getScoreAngle(Supplier<Pose2d> robotPose) {
    Angle angle = Degrees.of(0);
    if (0 < getHubDistance(robotPose).in(Meters)
        && getHubDistance(robotPose).in(Meters) < region1.in(Meters)) {
      angle = HoodConstants.kRegion1ScoreAngle;
    } else if (region1.in(Meters) < getHubDistance(robotPose).in(Meters)
        && getHubDistance(robotPose).in(Meters) < region2.in(Meters)) {
      angle = HoodConstants.kRegion2ScoreAngle;
    } else if (region2.in(Meters) < getHubDistance(robotPose).in(Meters)) {
      angle = HoodConstants.kRegion3ScoreAngle;
    }

    return angle;
  }

  public static Command scoreFuelFromAnywhere(
      Drivetrain drivetrain,
      Shooter shooter,
      Hood hood,
      Supplier<Rotation2d> hubHeading,
      Supplier<Pose2d> robotPose) {
    return Align.rotateToHub(drivetrain, hubHeading, robotPose)
        .andThen(ShootFuel.outtakeWithVelocity(shooter, () -> getScoreVelocity(robotPose)))
        .alongWith(HoodCommands.goToAngle(hood, () -> getScoreAngle(robotPose)));
  }

  public static Command shootFuelFromAnywhere(
      Drivetrain drivetrain, Shooter shooter, Hood hood, Supplier<Pose2d> robotPose) {
    return ShootFuel.outtakeWithVelocity(shooter, () -> getScoreVelocity(robotPose))
        .alongWith(HoodCommands.goToAngle(hood, () -> getScoreAngle(robotPose)));
  }

  public static Command scoreFuelWhileDriving(
      Drivetrain drivetrain,
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      Shooter shooter,
      Hood hood,
      Supplier<Rotation2d> hubHeading,
      Supplier<Pose2d> robotPose) {
    return Align.rotateToHubWhileDriving(
            drivetrain, translationX, translationY, hubHeading, robotPose)
        .alongWith(shootFuelFromAnywhere(drivetrain, shooter, hood, robotPose));
  }
}
