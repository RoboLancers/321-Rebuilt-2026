/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.AprilTagUtil;
import frc.robot.util.MyAlliance;
import java.util.List;
import java.util.function.Supplier;

public class AlignTest {

  private static final List<Integer> redApriltagIDs =
      List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  private static final List<Integer> blueApriltagIDs =
      List.of(17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);

  private static final List<Pose2d> redTagPoses = AprilTagUtil.apriltagIDsToPoses(redApriltagIDs);
  private static final List<Pose2d> blueTagPoses = AprilTagUtil.apriltagIDsToPoses(blueApriltagIDs);

  private static final List<Integer> redHubTags = List.of(9, 10);
  private static final List<Integer> blueHubTags = List.of(25, 26);

  private static final Distance alignmentDistance = Inches.of(24);
  private static final Rotation2d alignmentRotation = new Rotation2d(Degrees.of(180));
  private static final Transform2d alignmentTransform =
      new Transform2d(Meters.of(-alignmentDistance.in(Meters)), Meters.zero(), alignmentRotation);

  public static Command driveToPose(Drivetrain drivetrain, Supplier<Pose2d> pose) {
    return Commands.run(() -> drivetrain.driveToFieldPose(pose.get(), drivetrain.getPose()));
  }

//   public static Command driveToPosePP(Drivetrain drivetrain, Supplier<Pose2d> pose) {
//     return drivetrain.driveToPosePP(pose.get());
//   }

  public static Pose2d getNearestApriltag(Drivetrain drivetrain) {
    return MyAlliance.isRed()
        ? drivetrain.getPose().nearest(redTagPoses)
        : drivetrain.getPose().nearest(blueTagPoses);
  }

  public static Command alignToApriltag(Drivetrain drivetrain, Supplier<Integer> ID) {
    Pose2d targetPose =
        RobotConstants.kAprilTagLayout
            .getTagPose(ID.get())
            .orElse(null)
            .toPose2d()
            .plus(alignmentTransform);
    return driveToPose(drivetrain, ()->targetPose);
  }

//   public static Command alignToApriltagPP(Drivetrain drivetrain, Supplier<Integer> ID) {
//     Pose2d targetPose =
//         RobotConstants.kAprilTagLayout
//             .getTagPose(ID.get())
//             .orElse(null)
//             .toPose2d()
//             .plus(alignmentTransform);
//     return driveToPosePP(drivetrain, ()->targetPose);
//   }

  public static Command alignToPose(Drivetrain drivetrain, Supplier<Pose2d> pose) {
    return driveToPose(drivetrain, ()-> pose.get().plus(alignmentTransform));
  }

//   public static Command alignToPosePP(Drivetrain drivetrain, Supplier<Pose2d> pose){
//     return driveToPosePP(drivetrain, ()-> pose.get().plus(alignmentTransform));
//   }

  public static Command alignToNearestApriltag(Drivetrain drivetrain) {

    Pose2d apriltagPose = getNearestApriltag(drivetrain);

    return alignToPose(drivetrain, ()->apriltagPose);
  }

//   public static Command alignToNearestApriltagPP(Drivetrain drivetrain) {

//     Pose2d apriltagPose = getNearestApriltag(drivetrain);

//     return alignToPosePP(drivetrain, ()->apriltagPose);
//   }

}