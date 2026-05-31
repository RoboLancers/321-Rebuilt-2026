/* (C) RoboLancers 2026 */
package frc.robot.subsystems.questNav;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private double lastPoseTimestamp = -1;

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

    questNav.setVersionCheckEnabled(QuestNavConstants.kQuestVersionCheck);

    // later add more indepth logging features here
  }

  public void periodic() {
    questNav.commandPeriodic();

    boolean questConnected = questNav.isConnected();
    boolean questIsTracking = questNav.isTracking();

    SmartDashboard.putBoolean("QuestNav/Connected", questConnected);
    SmartDashboard.putBoolean("QuestNav/IsTracking", questIsTracking);
    SmartDashboard.putNumber("QuestNav/Latenyc", questNav.getLatency());

    questNav
        .getBatteryPercent()
        .ifPresent(
            percent -> {
              SmartDashboard.putNumber("QuestNav/BatteryPercent", percent);
              if (percent < QuestNavConstants.kQuestCriticalPercent) {
                DriverStation.reportWarning("Quest battery critical: " + percent + "%", false);
              }
            });
  }
}
