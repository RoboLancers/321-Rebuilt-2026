/* (C) RoboLancers 2026 */
package frc.robot.subsystems.questNav;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import java.util.List;

public class QuestNavSubsystem {

  private QuestNav questNav = new QuestNav();
  private Drivetrain drivetrain;

  private static final AprilTagFieldLayout AprilTagLayout = RobotConstants.kAprilTagLayout;

  // publishers for advantagescope
  private final StructArrayPublisher<Pose3d> allPublishedPoses;
  private final StructArrayPublisher<Pose3d> acceptedPoses;
  private final StructArrayPublisher<Pose3d> rejectedPoses;
  private final StructPublisher<Pose3d> latestPose;

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
    latestPose = networkTables.getStructTopic("QuestNav/latestPose", Pose3d.struct).publish();

    questNav.setVersionCheckEnabled(QuestNavConstants.kQuestVersionCheck);

    questNav.onConnected(() -> System.out.println("Quest Connected"));
    questNav.onDisconnected(() -> DriverStation.reportWarning("Quest Disconnected", false));
    questNav.onTrackingAcquired(() -> System.out.println("Quest Tracking Acquired"));
    questNav.onTrackingLost(() -> DriverStation.reportWarning("Quest Tracking Lost", false));
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

    questNav
        .getTrackingLostCounter()
        .ifPresent(
            count -> {
              SmartDashboard.putNumber("QuestNav/TrackingLostCount", count);
            });

    // Processes for reading unread pose frames

    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
    SmartDashboard.putNumber("QuestNav/UnreadPoseFrames", frames.length);

    List<Pose3d> allPoses = new ArrayList<>();
    List<Pose3d> acceptedPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    for (PoseFrame frame : frames) {
      Pose3d questPose = frame.questPose3d();
      Pose3d robotPose = questPose.transformBy(QuestNavConstants.kRobotToQuest.inverse());

      allPoses.add(robotPose);

      questNav.setPose(robotPose);

      if (rejectPose(robotPose)) {
        rejectedPoses.add(robotPose);
        continue;
      }

      acceptedPoses.add(robotPose);

      if (frame.isTracking()) {
        drivetrain.addVisionMeasurement(
            robotPose.toPose2d(), frame.dataTimestamp(), QuestNavConstants.kQuestStdDev);
      }
    }
  }

  public boolean rejectPose(Pose3d pose) {
    return pose.getX() < 0
        || pose.getX() > RobotConstants.kAprilTagLayout.getFieldLength()
        || pose.getY() < 0
        || pose.getY() > RobotConstants.kAprilTagLayout.getFieldWidth();
  }

  public void resetQuestPose(Pose3d robotPose) {
    Pose3d questPose = robotPose.transformBy(QuestNavConstants.kRobotToQuest);
    questNav.setPose(questPose);
  }
}
