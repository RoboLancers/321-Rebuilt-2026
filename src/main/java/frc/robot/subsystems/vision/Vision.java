/* (C) RoboLancers 2026 */
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.vision.CameraStatusLED.StatusType;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Dictionary;
import java.util.Hashtable;
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

  private CANdle LEDCandle;
  public double currentAmbiguity;
  public final int candlePort = 0;
  public Color status;

  public double getAmbiguityFromSupplier(DoubleSupplier ambiguity) {
    return ambiguity.getAsDouble();
  }

  public double getCurrentAmbiguity() {
    return getAmbiguityFromSupplier(() -> currentAmbiguity);
  }

  public Pose3d latestBestPose = new Pose3d(new Translation3d(0, 0, 0), Rotation3d.kZero);
  public EstimatedRobotPose latestEstimatedRobotPose;

  public Pose3d getPoseFromSupplier(Supplier<Pose3d> bestPose) {
    return bestPose.get();
  }

  public Pose3d getLatestBestPose() {
    return getPoseFromSupplier(() -> latestBestPose);
  }

  public Consumer<VisionEstimate> visionEstConsumer;

  private Dictionary<PhotonCamera, CameraStatusLED> cameraStatusLEDs =
      new Hashtable<PhotonCamera, CameraStatusLED>(4);

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

  public static Vision create(Consumer<VisionEstimate> visionEstConsume, CANdle LEDCandle) {
    return new Vision(visionEstConsume, LEDCandle);
  }

  public Vision(Consumer<VisionEstimate> visionEstConsumer, CANdle LEDCandle) {
    this.visionEstConsumer = visionEstConsumer;
    this.LEDCandle = LEDCandle;

    cameraStatusLEDs.put(leftClimbCamera, new CameraStatusLED(LEDCandle, 0, 1));
    cameraStatusLEDs.put(rightClimbCamera, new CameraStatusLED(LEDCandle, 2, 3));
    cameraStatusLEDs.put(leftShooterCamera, new CameraStatusLED(LEDCandle, 4, 5));
    cameraStatusLEDs.put(rightShooterCamera, new CameraStatusLED(LEDCandle, 6, 7));
  }

  private List<VisionEstimate> getVisionEstimates() {

    List<Double> ambiguities = new ArrayList<Double>();

    List<VisionEstimate> visionEstimates = new ArrayList<>();

    for (int i = 0; i < cameras.size(); i++) {

      CameraStatusLED statusLED = cameraStatusLEDs.get(cameras.get(i));
      if (cameras.get(i) == null || !cameras.get(i).isConnected()) {
        statusLED.updateStatusColor(StatusType.Error);
        continue;
      }

      List<PhotonPipelineResult> unreadResults = cameras.get(i).getAllUnreadResults();
      if (unreadResults.size() == 0) {
        continue;
      }
      PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);

      if (!latestResult.hasTargets() || unreadResults.isEmpty()) {
        statusLED.updateStatusColor(StatusType.NotDetected);
        continue;
      }

      statusLED.updateStatusColor(StatusType.Detected);

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
              .estimatedPose;

      this.latestEstimatedRobotPose =
          visionEstimates.get(ambiguities.indexOf(Collections.min(ambiguities))).estimatedPose();
      this.currentAmbiguity = Collections.min(ambiguities);
    }

    return visionEstimates;
  }

  // TODO: Update Standard Deviations
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

  @Logged
  public boolean areCamerasConnected() {
    for (PhotonCamera camera : cameras) {
      if (!camera.isConnected()) {
        return false;
      }
    }
    return true;
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

    SmartDashboard.putString(
        "Left Shooter Cam Status Color",
        cameraStatusLEDs.get(leftShooterCamera).getStatusColorHex());
    SmartDashboard.putString(
        "Right Climb Cam Status Color", cameraStatusLEDs.get(rightClimbCamera).getStatusColorHex());
    SmartDashboard.putString(
        "Left Climb Cam Status Color", cameraStatusLEDs.get(leftClimbCamera).getStatusColorHex());
    SmartDashboard.putString(
        "Right Shooter Cam Status Color",
        cameraStatusLEDs.get(rightShooterCamera).getStatusColorHex());
  }
}
