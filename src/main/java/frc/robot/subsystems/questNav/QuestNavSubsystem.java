/* (C) RoboLancers 2026 */
package frc.robot.subsystems.questNav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class QuestNavSubsystem {

  private QuestNav questNav = new QuestNav();
  private Drivetrain drivetrain;

  public Pose3d latestQuestPose;

  // publishers for advantagescope
  private final StructArrayPublisher<Pose3d> allPublishedPosesPub;
  private final StructArrayPublisher<Pose3d> acceptedPosesPub;
  private final StructArrayPublisher<Pose3d> rejectedPosesPub;
  private final StructPublisher<Pose3d> latestPosePub;

  private double lastPoseTimestamp = -1;

  public QuestNavSubsystem(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    var networkTables = NetworkTableInstance.getDefault();
    allPublishedPosesPub =
        networkTables.getStructArrayTopic("QuestNav/robotPoses", Pose3d.struct).publish();
    rejectedPosesPub =
        networkTables.getStructArrayTopic("QuestNav/rejectedPoses", Pose3d.struct).publish();
    acceptedPosesPub =
        networkTables.getStructArrayTopic("QuestNav/acceptedPoses", Pose3d.struct).publish();
    latestPosePub = networkTables.getStructTopic("QuestNav/latestPose", Pose3d.struct).publish();

    questNav.setVersionCheckEnabled(QuestNavConstants.kQuestVersionCheck);

    questNav.onConnected(() -> System.out.println("Quest Connected"));
    questNav.onDisconnected(() -> DriverStation.reportWarning("Quest Disconnected", false));
    questNav.onTrackingAcquired(() -> System.out.println("Quest Tracking Acquired"));
    questNav.onTrackingLost(() -> DriverStation.reportWarning("Quest Tracking Lost", false));
  }

  public void questPeriodic() {
    questNav.commandPeriodic();

    boolean questConnected = questNav.isConnected();
    boolean questIsTracking = questNav.isTracking();

    SmartDashboard.putBoolean("QuestNav/Connected", questConnected);
    SmartDashboard.putBoolean("QuestNav/IsTracking", questIsTracking);
    SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());

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

      lastPoseTimestamp = frame.dataTimestamp();
    }

    // published pose arrays for advantagescope
    allPublishedPosesPub.set(allPoses.toArray(Pose3d[]::new));
    acceptedPosesPub.set(acceptedPoses.toArray(Pose3d[]::new));
    rejectedPosesPub.set(rejectedPoses.toArray(Pose3d[]::new));

    if (!allPoses.isEmpty()) {
      latestPosePub.set(allPoses.get(allPoses.size() - 1));
      latestQuestPose = allPoses.get(allPoses.size() - 1);
    }

    if (lastPoseTimestamp > 0) {
      SmartDashboard.putNumber(
          "QuestNav/TimeSinceLastPose", Timer.getTimestamp() - lastPoseTimestamp);
    }
  }

  public boolean rejectPose(Pose3d pose) {
    return pose.getX() < 0
        || pose.getX() > RobotConstants.kAprilTagLayout.getFieldLength()
        || pose.getY() < 0
        || pose.getY() > RobotConstants.kAprilTagLayout.getFieldWidth();
  }

  public void resetQuestPose3d(Pose3d robotPose) {
    Pose3d questPose = robotPose.transformBy(QuestNavConstants.kRobotToQuest);
    questNav.setPose(questPose);
  }

  public void resetQuestPose2d(Pose2d robotPose) {
    resetQuestPose3d(new Pose3d(robotPose));
  }

  public Pose3d getPoseFromSupplier(Supplier<Pose3d> bestPose) {
    return bestPose.get();
  }

  public Pose3d getLatestQuestPose() {
    return getPoseFromSupplier(() -> latestQuestPose);
  }
}
