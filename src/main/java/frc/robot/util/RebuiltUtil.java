/* (C) RoboLancers 2026 */
package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.List;
import java.util.function.Supplier;

public class RebuiltUtil {

  public static final List<Integer> redApriltagIDs =
      List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  public static final List<Integer> blueApriltagIDs =
      List.of(17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);

  public static final List<Pose2d> redTagPoses = AprilTagUtil.aprilTagIDsToPoses(redApriltagIDs);
  public static final List<Pose2d> blueTagPoses = AprilTagUtil.aprilTagIDsToPoses(blueApriltagIDs);

  public static final Pose2d redHubPose =
      new Pose2d(Meters.of(11.9231), Meters.of(4.03), Rotation2d.kZero);
  public static final Pose2d blueHubPose =
      new Pose2d(Meters.of(4.6169), Meters.of(4.03), new Rotation2d(Degrees.of(180)));

  public static final Rectangle2d redAllianceZone =
      new Rectangle2d(new Translation2d(16.5, 0), new Translation2d(12.5, 8));
  public static final Rectangle2d blueAllianceZone =
      new Rectangle2d(new Translation2d(0, 0), new Translation2d(4, 8));

  public static final Rectangle2d redAllianceDefenseZone =
      new Rectangle2d(new Translation2d(16.5, 0), new Translation2d(13, 8));
  public static final Rectangle2d blueAllianceDefenseZone =
      new Rectangle2d(new Translation2d(0, 0), new Translation2d(3.5, 8));

  public static final Rectangle2d neutralZone = new Rectangle2d(new Translation2d(5.2,0), new Translation2d(11.4,8));
  public static final Rectangle2d blueNeutralZone = new Rectangle2d(new Translation2d(5.2,0), new Translation2d(8.3,8));
  public static final Rectangle2d redNeutralZone = new Rectangle2d(new Translation2d(8.3,0), new Translation2d(11.4,8));
  public static final Rectangle2d neutralDefenseZone = new Rectangle2d(new Translation2d(5.7,0), new Translation2d(10.9,8));
  public static final Rectangle2d blueNeutralDefenseZone = new Rectangle2d(new Translation2d(5.7,0), new Translation2d(8.3,8));
  public static final Rectangle2d redNeutralDefenseZone = new Rectangle2d(new Translation2d(8.3,0), new Translation2d(10.9,8));

  public static final int redClimbTagID = 15;
  public static final int blueClimbTagID = 31;

  public static final int redTroughTagID = 13;
  public static final int blueTroughTagID = 29;

  public static Pose2d getHubPose() {
    return MyAlliance.isRed() ? redHubPose : blueHubPose;
  }

  public static Rectangle2d getAllianceZone() {
    return MyAlliance.isRed() ? redAllianceZone : blueAllianceZone;
  }

  public static Rectangle2d getOppositeAllianceZone(){
    return MyAlliance.isRed() ? blueAllianceZone : redAllianceZone;
  }

  public static Rectangle2d getOppositeAllianceDefenseZone() {
    return MyAlliance.isRed() ? blueAllianceDefenseZone : redAllianceDefenseZone;
  }

  public static boolean inAllianceZone(Pose2d pose) {
    return getAllianceZone().contains(pose.getTranslation());
  }

  public static boolean inOppositeAllianceZone(Pose2d pose){
    return getOppositeAllianceZone().contains(pose.getTranslation());
  }

  public static boolean inRedAllianceZone(Pose2d pose) {
    return redAllianceZone.contains(pose.getTranslation());
  }

  public static boolean inBlueAllianceZone(Pose2d pose) {
    return blueAllianceZone.contains(pose.getTranslation());
  }

  public static boolean inBlueDefenseZone(Pose2d pose) {
    return blueAllianceDefenseZone.contains(pose.getTranslation());
  }

  public static boolean inRedDefenseZone(Pose2d pose) {
    return redAllianceDefenseZone.contains(pose.getTranslation());
  }

  public static boolean inOppositeAllianceDefenseZone(Pose2d pose) {
    return getOppositeAllianceDefenseZone().contains(pose.getTranslation());
  }

  public static boolean inNeutralZone(Pose2d pose) {
    return neutralZone.contains(pose.getTranslation());
  }

  public static boolean inNeutralDefenseZone(Pose2d pose) {
    return neutralDefenseZone.contains(pose.getTranslation());
  }

  public static boolean inRedNeutralDefenseZone(Pose2d pose) {
    return redNeutralDefenseZone.contains(pose.getTranslation());
  }

  public static boolean inBlueNeutralDefenseZone(Pose2d pose) {
    return blueNeutralDefenseZone.contains(pose.getTranslation());
  }

  public static boolean inBlueNeutralZone(Pose2d pose) {
    return blueNeutralZone.contains(pose.getTranslation());
  }

  public static boolean inRedNeutralZone(Pose2d pose) {
    return redNeutralZone.contains(pose.getTranslation());
  }

  public static TunableConstant xTransform = new TunableConstant("X Transform", 0);
  public static TunableConstant yTransform = new TunableConstant("Y Transform", 0);

  public static Rotation2d getHubHeading(Supplier<Pose2d> robotPose) {

    Rotation2d rotationToHub =
        new Rotation2d(
            Radians.of(
                Math.PI
                    + getHubPose().getRotation().getRadians()
                    + Math.atan(
                        (getHubPose().getY()
                                - (robotPose
                                        .get()
                                        .plus(
                                            new Transform2d(
                                                Inches.of(-8.0), Inches.of(2.5), Rotation2d.kZero)))
                                    .getY())
                            / (getHubPose().getX()
                                - (robotPose
                                        .get()
                                        .plus(
                                            new Transform2d(
                                                Inches.of(-8.0), Inches.of(2.5), Rotation2d.kZero)))
                                    .getX()))));

    return rotationToHub;
  }

  public static Distance getHubDistance(Supplier<Pose2d> robotPose) {
    Distance distance =
        Meters.of(robotPose.get().getTranslation().getDistance(getHubPose().getTranslation()));
    return distance;
  }
}
