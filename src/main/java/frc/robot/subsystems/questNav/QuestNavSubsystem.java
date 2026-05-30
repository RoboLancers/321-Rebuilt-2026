/* (C) RoboLancers 2026 */
package frc.robot.subsystems.questNav;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem {

  private QuestNav questNav = new QuestNav();
  private Drivetrain drivetrain;

  private static final AprilTagFieldLayout AprilTagLayout = RobotConstants.kAprilTagLayout;

  // publishers for advantagescope
  private final StructArrayPublisher<Pose3d> allPublishedPoses;
  private final StructArrayPublisher<Pose3d> acceptedPoses;
  private final StructArrayPublisher<Pose3d> rejectedPoses;
  private final StructArrayPublisher<Pose3d> latestPose;

  public QuestNavSubsystem(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    var networkTables = NetworkTableInstance.getDefault();
    allPublishedPoses =
        networkTables.getStructArrayTopic("QuestNav/robotPoses", Pose3d.struct).publish();
    acceptedPoses =
        networkTables.getStructArrayTopic("QuestNav/rejectedPoses", Pose3d.struct).publish();
    rejectedPoses =
        networkTables.getStructArrayTopic("QuestNav/acceptedPoses", Pose3d.struct).publish();
    latestPose = networkTables.getStructArrayTopic("QuestNav/latestPose", Pose3d.struct).publish();

    questNav.setVersionCheckEnabled(QuestNavConstants.questVersionCheck);
  }
}
