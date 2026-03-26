/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.MyAlliance;
import frc.robot.util.RebuiltUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class Align {

  private static final Distance alignmentDistance = Inches.of(17);
  private static final Rotation2d alignmentRotation = new Rotation2d(Degrees.of(180));
  private static final Transform2d alignmentTransform =
      new Transform2d(
          Meters.of(alignmentDistance.in(Meters)),
          Meters.zero(),
          alignmentRotation); // TODO: check coordinate accuracy

  private static final Pose2d blueHubScoringPoseLeft =
      new Pose2d(Meters.zero(), Meters.zero(), new Rotation2d(Degrees.zero()));
  private static final Pose2d blueHubScoringPoseRight =
      new Pose2d(Meters.zero(), Meters.zero(), new Rotation2d(Degrees.zero()));
  private static final Pose2d redHubScoringPoseLeft =
      new Pose2d(Meters.zero(), Meters.zero(), new Rotation2d(Degrees.zero()));
  private static final Pose2d redHubScoringPoseRight =
      new Pose2d(Meters.zero(), Meters.zero(), new Rotation2d(Degrees.zero()));

  private static final Transform2d leftClimbAlign =
      new Transform2d(Meters.zero(), Meters.zero(), Rotation2d.kZero);

  private static final Transform2d rightClimbAlign =
      new Transform2d(Meters.zero(), Meters.zero(), Rotation2d.kZero);

  private static final Transform2d troughAlign =
      new Transform2d(Meters.zero(), Meters.zero(), Rotation2d.kZero);

  private static final Angle shooterFaceOffset = Degrees.of(90);

  public Command driveToPose(
      Drivetrain drivetrain, Supplier<Pose2d> pose, Supplier<Pose2d> robotPose) {
    return drivetrain.driveToFieldPoseCommand(pose, robotPose);
  }

  public static Command alignToNearestApriltag(Drivetrain drivetrain, Supplier<Pose2d> robotPose) {

    Pose2d apriltagPose = drivetrain.getNearestAllianceAprilTag().plus(alignmentTransform);

    return drivetrain.driveToFieldPoseCommand(() -> apriltagPose, robotPose);
  }

  public static Command alignToApriltag(
      Drivetrain drivetrain, Supplier<Integer> ID, Supplier<Pose2d> robotPose) {
    Pose2d targetPose =
        RobotConstants.kAprilTagLayout
            .getTagPose(ID.get())
            .map(pose -> pose.toPose2d())
            .orElse(robotPose.get())
            .plus(alignmentTransform);
    return drivetrain.driveToFieldPoseCommand(() -> targetPose, robotPose);
  }

  public static Command alignToPose(
      Drivetrain drivetrain, Supplier<Pose2d> pose, Supplier<Pose2d> robotPose) {
    return drivetrain.driveToFieldPoseCommand(() -> pose.get().plus(alignmentTransform), robotPose);
  }

  public static boolean robotOnRightSide(Supplier<Pose2d> robotPose) {
    return robotPose.get().getMeasureY().in(Meters)
        < 0.5 * VisionConstants.kAllowedFieldDistance.in(Meters);
  }

  public static Pose2d getHubScoringPose(Supplier<Pose2d> robotPose) {

    Pose2d hubScoringPose = robotPose.get();

    if (MyAlliance.isRed() && robotOnRightSide(robotPose)) {
      hubScoringPose = redHubScoringPoseRight;
    } else if (MyAlliance.isRed() && !robotOnRightSide(robotPose)) {
      hubScoringPose = redHubScoringPoseLeft;
    } else if (MyAlliance.isBlue() && robotOnRightSide(robotPose)) {
      hubScoringPose = blueHubScoringPoseLeft;
    } else if (MyAlliance.isBlue() && !robotOnRightSide(robotPose)) {
      hubScoringPose = blueHubScoringPoseRight;
    }

    return hubScoringPose;
  }

  public static Command driveToHubScoringPose(Drivetrain drivetrain, Supplier<Pose2d> robotPose) {
    return drivetrain.driveToFieldPoseCommand(() -> getHubScoringPose(robotPose), robotPose);
  }

  public static Command rotateToHub(
      Drivetrain drivetrain, DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Rotation2d> hubHeading, Supplier<Pose2d> robotPose) {
   return lockOnHub(drivetrain, translationX, translationY, hubHeading, robotPose).until(drivetrain.atHeading(hubHeading));
  }

  public static Command lockOnHub(
      Drivetrain drivetrain,
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      Supplier<Rotation2d> hubHeading,
      Supplier<Pose2d> robotPose) {
    return drivetrain.driveFixedHeading(
        translationX,
        translationY,
        () ->
            new Rotation2d(
                Degrees.of(hubHeading.get().getDegrees() + shooterFaceOffset.in(Degrees))));
  }

  

  public static Command alignLeftClimb(Drivetrain drivetrain, Supplier<Pose2d> robotPose) {
    Pose2d climbPose =
        (MyAlliance.isRed()
                ? RobotConstants.kAprilTagLayout.getTagPose(RebuiltUtil.redClimbTagID)
                : RobotConstants.kAprilTagLayout.getTagPose(RebuiltUtil.blueClimbTagID))
            .map(pose -> pose.toPose2d().plus(leftClimbAlign))
            .orElse(robotPose.get());

    return drivetrain.driveToFieldPoseCommand(() -> climbPose, robotPose);
  }

  public static Command alignRightClimb(Drivetrain drivetrain, Supplier<Pose2d> robotPose) {
    Pose2d climbPose =
        (MyAlliance.isRed()
                ? RobotConstants.kAprilTagLayout.getTagPose(RebuiltUtil.redClimbTagID)
                : RobotConstants.kAprilTagLayout.getTagPose(RebuiltUtil.blueClimbTagID))
            .map(pose -> pose.toPose2d().plus(rightClimbAlign))
            .orElse(robotPose.get())
            .plus(rightClimbAlign);

    return drivetrain.driveToFieldPoseCommand(() -> climbPose, robotPose);
  }

  public static Command alignToTrough(Drivetrain drivetrain, Supplier<Pose2d> robotPose) {
    Pose2d troughPose =
        (MyAlliance.isRed()
                ? RobotConstants.kAprilTagLayout.getTagPose(RebuiltUtil.redTroughTagID)
                : RobotConstants.kAprilTagLayout.getTagPose(RebuiltUtil.blueTroughTagID))
            .map(pose -> pose.toPose2d().plus(troughAlign))
            .orElse(robotPose.get());

    return drivetrain.driveToFieldPoseCommand(() -> troughPose, robotPose);
  }
}
