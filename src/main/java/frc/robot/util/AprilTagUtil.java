/* (C) RoboLancers 2026 */
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotConstants;
import java.util.ArrayList;
import java.util.List;

public class AprilTagUtil {
  public static int kAprilTagCount = 32;

  public static List<Pose2d> aprilTagIDsToPoses(List<Integer> apriltagIDs) {

    List<Pose2d> poses = new ArrayList<>();

    for (int ID : apriltagIDs) {

      Pose2d pose = RobotConstants.kAprilTagLayout.getTagPose(ID).orElse(null).toPose2d();
      poses.add(pose);
    }

    return poses;
  }

  public static List<Pose2d> getAllAprilTagPoses() {
    List<Pose2d> poses = new ArrayList<>();
    for (int i = 1; i <= kAprilTagCount; i++) {
      Pose2d pose = RobotConstants.kAprilTagLayout.getTagPose(i).orElse(null).toPose2d();
      poses.add(pose);
    }

    return poses;
  }
}
