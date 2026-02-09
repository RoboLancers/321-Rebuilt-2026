/* (C) RoboLancers 2026 */
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.leds.LedConstants;
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

  public double currentAmbiguity;
  public final int ledPort1 = 0;
  public final int ledPort2 = 1;
  public final int ledPort3 = 2;
  public final int ledPort4 = 3;
  public final int ledStart = 0;
  public final int ledEnd = 7;
  public final Color purple = new Color(191, 64, 191);
  public final Color red = new Color(255, 255, 255);
  public final Color white = new Color(255, 0, 0);
  public Color status;
  public CANdle candle = new CANdle(0);

  public void LedConfigs() {
    LEDConfigs configs = new LEDConfigs();
    CANdleFeaturesConfigs featuresConfigs = new CANdleFeaturesConfigs();
    configs.BrightnessScalar = LedConstants.brightnessScaler;
    featuresConfigs.VBatOutputMode = VBatOutputModeValue.On;
    candle.getConfigurator().apply(configs);
    candle.getConfigurator().apply(featuresConfigs);
  }

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

  private PhotonCamera backLeftCamera = new PhotonCamera(VisionConstants.kBackLeftCameraName);

  private PhotonCamera frontLeftCamera = new PhotonCamera(VisionConstants.kTopElevatorCameraName);

  private PhotonCamera frontRightCamera =
      new PhotonCamera(VisionConstants.kBottomElevatorCameraName);

  private PhotonCamera backRightCamera = new PhotonCamera(VisionConstants.backRightCameraName);

  private List<PhotonCamera> cameras =
      List.of(backLeftCamera, frontLeftCamera, frontRightCamera, backRightCamera);

  private Dictionary<PhotonCamera, CameraStatusLED> cameraStatusLEDs =
      new Hashtable<PhotonCamera, CameraStatusLED>(4);

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

    cameraStatusLEDs.put(backLeftCamera, new CameraStatusLED(ledPort1));
    cameraStatusLEDs.put(frontLeftCamera, new CameraStatusLED(ledPort2));
    cameraStatusLEDs.put(backRightCamera, new CameraStatusLED(ledPort3));
    cameraStatusLEDs.put(frontRightCamera, new CameraStatusLED(ledPort4));
  }

  private List<VisionEstimate> getVisionEstimates() {

    List<Double> ambiguities = new ArrayList<Double>();

    List<VisionEstimate> visionEstimates = new ArrayList<>();

    for (int i = 0; i < cameras.size(); i++) {

      CameraStatusLED statusLED = cameraStatusLEDs.get(cameras.get(i));
      if (cameras.get(i) == null || !cameras.get(i).isConnected()) {

        // set the corresponding color to red
        statusLED.updateStatusColor(StatusType.Error);
        continue;
      }

      List<PhotonPipelineResult> unreadResults = cameras.get(i).getAllUnreadResults();
      PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);

      if (!latestResult.hasTargets() || unreadResults.isEmpty()) {
        // set corresponding color to white
        statusLED.updateStatusColor(StatusType.NotDetected);
        continue;
      }

      // set correponding color to purple
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

  public boolean areCamerasConnected;

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

    SmartDashboard.putString("Camera 1", cameraStatusLEDs.get(frontLeftCamera).getStatusColorHex());
    SmartDashboard.putString(
        "Camera 2 ", cameraStatusLEDs.get(frontRightCamera).getStatusColorHex());
    SmartDashboard.putString("Camera 3", cameraStatusLEDs.get(backRightCamera).getStatusColorHex());
    SmartDashboard.putString("Camera 4", cameraStatusLEDs.get(backLeftCamera).getStatusColorHex());

    SmartDashboard.putBoolean("Cameras Are Connected", areCamerasConnected);

    SmartDashboard.putNumber("Vision Pose X", getLatestBestPose().getX());

    SmartDashboard.putNumber("Vision Pose Y", getLatestBestPose().getY());

    SmartDashboard.putNumber("Vision Pose Yaw", getLatestBestPose().getRotation().getDegrees());
  }
}
