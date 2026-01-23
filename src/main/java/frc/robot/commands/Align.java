/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.commands.OuttakeFuel;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.AprilTagUtil;
import frc.robot.util.MyAlliance;
import java.util.List;
import java.util.function.Supplier;

public class Align {

  private static final List<Integer> redApriltagIDs =
      List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  private static final List<Integer> blueApriltagIDs =
      List.of(17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);

  private static final List<Pose2d> redTagPoses = AprilTagUtil.apriltagIDsToPoses(redApriltagIDs);
  private static final List<Pose2d> blueTagPoses = AprilTagUtil.apriltagIDsToPoses(blueApriltagIDs);

  private static final List<Integer> redHubTags = List.of(9, 10);
  private static final List<Integer> blueHubTags = List.of(25, 26);

  private static final Distance alignmentDistance = Inches.of(18);
  private static final Rotation2d alignmentRotation = new Rotation2d(Degrees.of(180));
  private static final Transform2d alignmentTransform =
      new Transform2d(Meters.of(-alignmentDistance.in(Meters)), Meters.zero(), alignmentRotation);

  private static final Pose2d blueHubScoringPoseLeft = new Pose2d(Meters.zero(),Meters.zero(), new Rotation2d(Degrees.zero()));
  private static final Pose2d blueHubScoringPoseRight = new Pose2d(Meters.zero(),Meters.zero(), new Rotation2d(Degrees.zero()));
  private static final Pose2d redHubScoringPoseLeft = new Pose2d(Meters.zero(),Meters.zero(), new Rotation2d(Degrees.zero()));
  private static final Pose2d redHubScoringPoseRight = new Pose2d(Meters.zero(),Meters.zero(), new Rotation2d(Degrees.zero()));

  private static final Pose2d redHubPose = new Pose2d(0,0,Rotation2d.kZero);
  private static final Pose2d blueHubPose = new Pose2d(0,0,Rotation2d.kZero);

  private static final Distance region1 = Meters.of(0);
  private static final Distance region2 = Meters.of(0);
  private static final Distance region3 = Meters.of(0);

  public static Command driveToPose(Drivetrain drivetrain, Supplier<Pose2d> pose) {
    return Commands.run(() -> drivetrain.driveToFieldPose(pose.get(), drivetrain.getPose()));
  }

  public static Command driveToPosePP(Drivetrain drivetrain, Supplier<Pose2d> pose) {
    return drivetrain.driveToPosePP(pose.get());
  }

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

  public static Command alignToPose(Drivetrain drivetrain, Supplier<Pose2d> pose) {
    return driveToPose(drivetrain, ()-> pose.get().plus(alignmentTransform));
  }

  public static Command alignToNearestApriltag(Drivetrain drivetrain) {

    Pose2d apriltagPose = getNearestApriltag(drivetrain);

    return alignToPose(drivetrain, ()->apriltagPose);
  }

  public static boolean robotOnRightSide(Supplier<Pose2d> currentPose){
    return currentPose.get().getMeasureY().in(Meters) < 0.5 * VisionConstants.kAllowedFieldDistance.in(Meters);
  }

  public static Pose2d getHubScoringPose(Drivetrain drivetrain){

    Pose2d hubScoringPose = null;

    if (MyAlliance.isRed() && robotOnRightSide(()->drivetrain.getPose())){
        hubScoringPose = redHubScoringPoseRight;
    }

    else if(MyAlliance.isRed() && !robotOnRightSide(()->drivetrain.getPose())){
        hubScoringPose = redHubScoringPoseLeft;
    }

    else if(!MyAlliance.isRed() && robotOnRightSide(()->drivetrain.getPose())){
        hubScoringPose = blueHubScoringPoseRight;
    }

    else if(!MyAlliance.isRed() && !robotOnRightSide(()->drivetrain.getPose())){
        hubScoringPose = blueHubScoringPoseLeft;
    }

    return hubScoringPose;
    
  }

  public static Command driveToHubScoringPose(Drivetrain drivetrain){
    return driveToPose(drivetrain, ()->getHubScoringPose(drivetrain));
  }

  public static Command scoreFuelFromPose(Drivetrain drivetrain, Outtake outtake){
    return driveToHubScoringPose(drivetrain).andThen(OuttakeFuel.scoreSetPosition(outtake));
  }

  public static Distance getHubDistance(Drivetrain drivetrain){
    Pose2d hubPose = null;

    if (MyAlliance.isRed()){
      hubPose = redHubPose;
    } 
    else {
      hubPose = blueHubPose;}
      
      return Meters.of(drivetrain.getPose().getTranslation().getDistance(hubPose.getTranslation()));
  }

  public static AngularVelocity getScoreVelocity(Drivetrain drivetrain){
    AngularVelocity velocity = RPM.of(0);
    if(0<getHubDistance(drivetrain).in(Meters) && getHubDistance(drivetrain).in(Meters)<region1.in(Meters)){
      velocity = OuttakeConstants.kRegion1ScoreRPM;
    } 
    else if (region1.in(Meters)<getHubDistance(drivetrain).in(Meters) && getHubDistance(drivetrain).in(Meters)<region2.in(Meters)){
      velocity = OuttakeConstants.kRegion2ScoreRPM;
    }
    else if (region2.in(Meters)<getHubDistance(drivetrain).in(Meters)){
      velocity = OuttakeConstants.kRegion3ScoreRPM;
    }

    return velocity;
  }


}
