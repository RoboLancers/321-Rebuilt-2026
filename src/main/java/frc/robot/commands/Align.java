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
import frc.robot.util.AprilTagUtil;
import frc.robot.util.MyAlliance;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class Align {

  private static final List<Integer> redApriltagIDs =
      List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  private static final List<Integer> blueApriltagIDs =
      List.of(17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);

  private static final List<Pose2d> redTagPoses = AprilTagUtil.apriltagIDsToPoses(redApriltagIDs);
  private static final List<Pose2d> blueTagPoses = AprilTagUtil.apriltagIDsToPoses(blueApriltagIDs);

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

  private static final Pose2d redHubPose = new Pose2d(Meters.of(12.52 - Inches.of(23.5).in(Meters)),	Meters.of(4.03), Rotation2d.kZero);
  private static final Pose2d blueHubPose =  new Pose2d(Meters.of(4.02 + Inches.of(23.5).in(Meters)),	Meters.of(4.03) , new Rotation2d(Degrees.of(180)));

  private static final int redClimbTagID = 15;
  private static final int blueClimbTagID = 31;

  private static final int redTroughTagID = 13;
  private static final int blueTroughTagID = 29;

  private static final Transform2d leftClimbAlign =
      new Transform2d(Meters.zero(), Meters.zero(), Rotation2d.kZero);

  private static final Transform2d rightClimbAlign =
      new Transform2d(Meters.zero(), Meters.zero(), Rotation2d.kZero);

  private static final Transform2d troughAlign =
      new Transform2d(Meters.zero(), Meters.zero(), Rotation2d.kZero);

  private static final Angle shooterFaceOffset = Degrees.of(0);

  public Command driveToPose(Drivetrain drivetrain, Supplier<Pose2d> pose, Supplier<Pose2d> robotPose) {
    return drivetrain.driveToFieldPoseCommand(pose, robotPose);
  }

  // public static Command driveToPosePP(Drivetrain drivetrain, Supplier<Pose2d> pose) {
  //   return drivetrain.driveToPosePP(pose.get());
  // }

  public static Pose2d getNearestApriltag(Supplier<Pose2d> robotPose) {
    return MyAlliance.isRed()
        ? robotPose.get().nearest(redTagPoses)
        : robotPose.get().nearest(blueTagPoses);
  }

  public static Command alignToNearestApriltag(Drivetrain drivetrain, Supplier<Pose2d> robotPose) {

    Pose2d apriltagPose = getNearestApriltag(robotPose).plus(alignmentTransform);

    return drivetrain.driveToFieldPoseCommand(() -> apriltagPose, robotPose);
  }
  
  public static Command alignToApriltag(Drivetrain drivetrain, Supplier<Integer> ID, Supplier<Pose2d> robotPose) {
    Pose2d targetPose =
        RobotConstants.kAprilTagLayout
            .getTagPose(ID.get())
            .map(pose->pose.toPose2d())
            .orElse(robotPose.get())
            .plus(alignmentTransform);
    return drivetrain.driveToFieldPoseCommand(() -> targetPose, robotPose);
  }

  
  public static Command alignToPose(Drivetrain drivetrain, Supplier<Pose2d> pose, Supplier<Pose2d> robotPose) {
    return drivetrain.driveToFieldPoseCommand(() -> pose.get().plus(alignmentTransform), robotPose);
  }


  public static boolean robotOnRightSide(Supplier<Pose2d> robotPose) {
    return robotPose.get().getMeasureY().in(Meters)
        < 0.5 * VisionConstants.kAllowedFieldDistance.in(Meters);
  }

  public static Pose2d getHubScoringPose(Supplier<Pose2d> robotPose) {

    Pose2d hubScoringPose = robotPose.get();

    if (!MyAlliance.isRed() && robotOnRightSide(robotPose)) {
      hubScoringPose = blueHubScoringPoseRight;
    } else if (!MyAlliance.isRed() && !robotOnRightSide(robotPose)) {
      hubScoringPose = blueHubScoringPoseLeft;
    } else if (MyAlliance.isRed() && robotOnRightSide(robotPose)) {
      hubScoringPose = redHubScoringPoseLeft;
    } else if (MyAlliance.isRed() && !robotOnRightSide(robotPose)) {
      hubScoringPose = redHubScoringPoseRight;
    }

    return hubScoringPose;
  }

  public static Pose2d getHub() {
    return MyAlliance.isRed() ? redHubPose : blueHubPose;
  }

  public static Command driveToHubScoringPose(Drivetrain drivetrain, Supplier<Pose2d> robotPose) {
    return drivetrain.driveToFieldPoseCommand(
        () -> getHubScoringPose(robotPose), robotPose);
  }

  public static Command rotateToHub(Drivetrain drivetrain, Supplier<Rotation2d> hubHeading, Supplier<Pose2d> robotPose){
    return rotateToHubWhileDriving(drivetrain, ()->0, ()->0, hubHeading, robotPose);
  }

  public static Command rotateToHub1(Drivetrain drivetrain, Supplier<Rotation2d> hubHeading, Supplier<Pose2d> robotPose){
    return rotateToHubWhileDriving1(drivetrain, ()->0, ()->0, hubHeading, robotPose);
  }

  public static Command rotateToHub2(Drivetrain drivetrain, Supplier<Rotation2d> hubHeading, Supplier<Pose2d> robotPose){
    return rotateToHubWhileDriving2(drivetrain, ()->0, ()->0, hubHeading, robotPose);
  }

  public static Command rotateToHubWhileDriving(
      Drivetrain drivetrain, DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Rotation2d> hubHeading, Supplier<Pose2d> robotPose) {
     return drivetrain.teleopDriveWithHeading(translationX, translationY, hubHeading, robotPose);
  }

  public static Command rotateToHubWhileDriving1(
      Drivetrain drivetrain, DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Rotation2d> hubHeading, Supplier<Pose2d> robotPose) {
     return drivetrain.teleopDriveWithHeading(translationX, translationY, hubHeading, robotPose);
  }

  public static Command rotateToHubWhileDriving2(
      Drivetrain drivetrain, DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Rotation2d> hubHeading, Supplier<Pose2d> robotPose) {
     return drivetrain.driveFixedHeading(translationX, translationY, hubHeading);
  }

  public static Command alignLeftClimb(Drivetrain drivetrain, Supplier<Pose2d> robotPose) {
    Pose2d climbPose =
        (MyAlliance.isRed()
                ? RobotConstants.kAprilTagLayout.getTagPose(redClimbTagID)
                : RobotConstants.kAprilTagLayout.getTagPose(blueClimbTagID))
            .map(pose->pose.toPose2d().plus(leftClimbAlign))
            .orElse(robotPose.get());

    return drivetrain.driveToFieldPoseCommand(()->climbPose, robotPose);
  }

  public static Command alignRightClimb(Drivetrain drivetrain, Supplier<Pose2d> robotPose) {
    Pose2d climbPose =
        (MyAlliance.isRed()
                ? RobotConstants.kAprilTagLayout.getTagPose(redClimbTagID)
                : RobotConstants.kAprilTagLayout.getTagPose(blueClimbTagID))
            .map(pose->pose.toPose2d().plus(rightClimbAlign))
            .orElse(robotPose.get())
            .plus(rightClimbAlign);

    return drivetrain.driveToFieldPoseCommand(() -> climbPose, robotPose);
  }

  public static Command alignToTrough(Drivetrain drivetrain, Supplier<Pose2d> robotPose) {
    Pose2d troughPose =
        (MyAlliance.isRed()
                ? RobotConstants.kAprilTagLayout.getTagPose(redTroughTagID)
                : RobotConstants.kAprilTagLayout.getTagPose(blueTroughTagID))
            .map(pose->pose.toPose2d().plus(troughAlign))
            .orElse(robotPose.get());

    return drivetrain.driveToFieldPoseCommand(() -> troughPose, robotPose);
  }
}
