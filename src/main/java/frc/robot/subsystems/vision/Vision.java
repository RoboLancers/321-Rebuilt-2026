/* (C) RoboLancers 2026 */
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  public double currentAmbiguity;

  public double getAmbiguityFromSupplier(DoubleSupplier ambiguity) {
    return ambiguity.getAsDouble();
  }

  public double getCurrentAmbiguity() {
    return getAmbiguityFromSupplier(() -> currentAmbiguity);
  }

  public Pose2d latestBestPose = new Pose2d(new Translation2d(0, 0), Rotation2d.kZero);
  public EstimatedRobotPose latestEstimatedRobotPose;

  public Pose2d getPoseFromSupplier(Supplier<Pose2d> bestPose) {
    return bestPose.get();
  }

  public Pose2d getLatestBestPose() {
    return getPoseFromSupplier(() -> latestBestPose);
  }

  public Consumer<VisionEstimate> visionEstConsumer;

  private PhotonCamera leftClimbCamera = new PhotonCamera(VisionConstants.kLeftClimbCameraName);

  private PhotonCamera rightClimbCamera = new PhotonCamera(VisionConstants.kRightClimbCameraName);

  private PhotonCamera leftShooterCamera = new PhotonCamera(VisionConstants.kLeftShooterCameraName);

  private PhotonCamera rightShooterCamera =
      new PhotonCamera(VisionConstants.kRightShooterCameraName);

  public List<PhotonCamera> cameras =
      List.of(leftClimbCamera, rightClimbCamera, leftShooterCamera, rightShooterCamera);

  private PhotonPoseEstimator leftClimbPoseEstimator =
      new PhotonPoseEstimator(
          RobotConstants.kAprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.kLeftClimbCameraTransform);

  private PhotonPoseEstimator rightClimbPoseEstimator =
      new PhotonPoseEstimator(
          RobotConstants.kAprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.kRightClimbCameraTransform);

  private PhotonPoseEstimator leftShooterPoseEstimator =
      new PhotonPoseEstimator(
          RobotConstants.kAprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.kLeftShooterCameraTransform);

  private PhotonPoseEstimator rightShooterPoseEstimator =
      new PhotonPoseEstimator(
          RobotConstants.kAprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.kRightShooterCameraTransform);

  public List<PhotonPoseEstimator> estimators =
      List.of(
          leftClimbPoseEstimator,
          rightClimbPoseEstimator,
          leftShooterPoseEstimator,
          rightShooterPoseEstimator);

  public static Vision create(Consumer<VisionEstimate> visionEstConsumer) {
    return new Vision(visionEstConsumer);
  }

  public Vision(Consumer<VisionEstimate> visionEstConsumer) {
    this.visionEstConsumer = visionEstConsumer;
  }

  private List<VisionEstimate> getVisionEstimates() {

    List<Double> ambiguities = new ArrayList<Double>();

    List<VisionEstimate> visionEstimates = new ArrayList<>();

    for (int i = 0; i < cameras.size(); i++) {

      if (cameras.get(i) == null || !cameras.get(i).isConnected()) continue;

      List<PhotonPipelineResult> unreadResults = cameras.get(i).getAllUnreadResults();

      if (unreadResults.isEmpty()) continue;

      PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);

      if (!latestResult.hasTargets()) continue;

      EstimatedRobotPose estimatedPose =
          estimators
              .get(i)
              .update(latestResult)
              .filter(
                  est ->
                      VisionConstants.kAllowedFieldArea.contains(
                              est.estimatedPose.getTranslation().toTranslation2d())
                          && est.estimatedPose
                              .getMeasureZ()
                              .isNear(Meters.of(0), VisionConstants.kAllowedFieldHeight))
              .orElse(null);

      if (estimatedPose == null) {
        continue;
      }

      double standardDeviation = calculateStdDevs(estimatedPose);

      VisionEstimate visionEstimate = new VisionEstimate(estimatedPose, standardDeviation);

      visionEstimates.add(visionEstimate);

      ambiguities.add(calculateAmbiguity(estimatedPose));
    }

    if (visionEstimates.size() == 0) {
      return null;
    }

    double highestConfidence = 1 - Collections.min(ambiguities);

    if (VisionConstants.kMinimumConfidence < highestConfidence) {

      this.latestBestPose =
          visionEstimates
              .get(ambiguities.indexOf(Collections.min(ambiguities)))
              .estimatedPose()
              .estimatedPose
              .toPose2d();

      this.latestEstimatedRobotPose =
          visionEstimates.get(ambiguities.indexOf(Collections.min(ambiguities))).estimatedPose();
      this.currentAmbiguity = Collections.min(ambiguities);
    }

    return visionEstimates;
  }

  private double calculateStdDevs(EstimatedRobotPose estimatedPose) {

    double distance = 0;

    if (estimatedPose == null) return 1;
    else {
      for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
        double targetDistance =
            target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
        distance = distance + targetDistance;
      }

      double averageDistance = distance / estimatedPose.targetsUsed.size();

      double standardDeviation = averageDistance / estimatedPose.targetsUsed.size();

      return standardDeviation;
    }
  }

  public double calculateAmbiguity(EstimatedRobotPose estimatedPose) {
    double totalAmbiguity = 0;
    if (estimatedPose == null) return 1;
    else {
      for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
        totalAmbiguity = totalAmbiguity + target.getPoseAmbiguity();
      }
      double averageAmbiguity = totalAmbiguity / estimatedPose.targetsUsed.size();
      return averageAmbiguity;
    }
  }

  public boolean areCamerasConnected = false;

  @Logged(name = "camerasAreConnected")
  public boolean getCamerasConnected() {
    return areCamerasConnected;
  }

  @Logged(name = "leftClimbCameraConnected")
  public boolean getLeftClimbCameraConnected() {
    return leftClimbCamera.isConnected();
  }

  @Logged(name = "rightClimbCameraConnected")
  public boolean getRightClimbCameraConnected() {
    return rightClimbCamera.isConnected();
  }

  @Logged(name = "leftShooterCameraConnected")
  public boolean getLeftShooterCameraConnected() {
    return leftShooterCamera.isConnected();
  }

  @Logged(name = "rightShooterCameraConnected")
  public boolean getRightShooterCameraConnected() {
    return rightShooterCamera.isConnected();
  }

  @Override
  public void periodic() {

    List<VisionEstimate> latestEstimates = getVisionEstimates();

    if (visionEstConsumer != null && latestEstimates != null) {
      for (VisionEstimate estimate : latestEstimates) {

        visionEstConsumer.accept(estimate);
      }
    }

    for (PhotonCamera camera : cameras) {
      if (camera.isConnected()) {
        areCamerasConnected = true;
      } else {
        areCamerasConnected = false;
        break;
      }
    }

    SmartDashboard.putBoolean("Cameras Are Connected", areCamerasConnected);

    SmartDashboard.putNumber("Vision Pose X", getLatestBestPose().getX());

    SmartDashboard.putNumber("Vision Pose Y", getLatestBestPose().getY());

    SmartDashboard.putNumber("Vision Pose Yaw", getLatestBestPose().getRotation().getDegrees());
  }
}
