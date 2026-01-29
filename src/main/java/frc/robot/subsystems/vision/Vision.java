/* (C) RoboLancers 2026 */
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

  public Pose2d bestPose = new Pose2d(0,0,Rotation2d.kZero);

  public Pose2d getPoseFromSupplier(Supplier<Pose2d> bestPose) {
    return bestPose.get();
  }

  public Pose2d getBestPose() {
    return getPoseFromSupplier(() -> bestPose);
  }

  public Consumer<VisionEstimate> visionEstConsumer;

  private PhotonCamera backLeftCamera = new PhotonCamera(VisionConstants.kBackLeftCameraName);

  private PhotonCamera topElevatorCamera = new PhotonCamera(VisionConstants.kTopElevatorCameraName);

  private PhotonCamera bottomElevatorCamera =
      new PhotonCamera(VisionConstants.kBottomElevatorCameraName);

  public List<PhotonCamera> cameras =
      List.of(backLeftCamera, topElevatorCamera, bottomElevatorCamera);

  private PhotonPoseEstimator backLeftPoseEstimator =
      new PhotonPoseEstimator(
          RobotConstants.kAprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.kBackLeftTransform);

  private PhotonPoseEstimator topElevatorPoseEstimator =
      new PhotonPoseEstimator(
          RobotConstants.kAprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.kTopElevatorTransform);

  private PhotonPoseEstimator bottomElevatorPoseEstimator =
      new PhotonPoseEstimator(
          RobotConstants.kAprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.kBottomElevatorTransform);

  public List<PhotonPoseEstimator> estimators =
      List.of(backLeftPoseEstimator, topElevatorPoseEstimator, bottomElevatorPoseEstimator);

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

      if (!cameras.get(i).isConnected()) return null;

      List<PhotonPipelineResult> unreadResults = cameras.get(i).getAllUnreadResults();

      if (unreadResults.isEmpty()) return null;

      PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);

      if (!latestResult.hasTargets()) return null;

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

      double standardDeviation = calculateStdDevs(estimatedPose);

      VisionEstimate visionEstimate = new VisionEstimate(estimatedPose, standardDeviation);

      visionEstimates.add(visionEstimate);

      ambiguities.add(calculateAmbiguity(estimatedPose));
    }

    double highestConfidence = 1 - Collections.min(ambiguities);

    if (VisionConstants.kMinimumConfidence < highestConfidence) {

      this.bestPose =
          visionEstimates.get(ambiguities.indexOf(Collections.min(ambiguities))).estimatedPose().estimatedPose.toPose2d();

      this.currentAmbiguity = Collections.min(ambiguities);
    }

    return visionEstimates;
  }

  private double calculateStdDevs(EstimatedRobotPose estimatedPose) {

    double distance = 0;

    if (estimatedPose == null) return 1;
else{
    for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
      double targetDistance =
          target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
      distance = distance + targetDistance;
    }

    double averageDistance = distance / estimatedPose.targetsUsed.size();

    double standardDeviation = averageDistance / estimatedPose.targetsUsed.size();

    return standardDeviation;}
  }

  public double calculateAmbiguity(EstimatedRobotPose estimatedPose) {
    double totalAmbiguity = 0;
if(estimatedPose == null) return 1;
else{
    for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
      totalAmbiguity = totalAmbiguity + target.getPoseAmbiguity();
    }
    double averageAmbiguity = totalAmbiguity / estimatedPose.targetsUsed.size();
    return averageAmbiguity;
  }}

  public boolean areCamerasConnected;

  @Override
  public void periodic() {

    List<VisionEstimate> latestEstimates = getVisionEstimates();

    if(!(latestEstimates == null)){for (VisionEstimate estimate : latestEstimates) {
      visionEstConsumer.accept(estimate);
    }}

    getVisionEstimates();

    for (PhotonCamera camera : cameras) {
      if (camera.isConnected()) {
        areCamerasConnected = true;
      } else {
        areCamerasConnected = false;
        break;
      }
    }
  }
}
