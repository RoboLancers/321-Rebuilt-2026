/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class PoseEstimatorResolver extends SubsystemBase{

  public Vision vision;

  public Drivetrain drivetrain;

  public Pigeon2 pigeon = drivetrain.getPigeon2();

  public Pose2d visionPose = vision.getBestPose().estimatedPose.toPose2d();

  public Pose2d drivetrainPose = drivetrain.getSwerveDriveEstimatedPose();

  public Consumer<Pose2d> robotPoseConsumer;

  public PoseEstimatorResolver(Vision vision, Drivetrain drivetrain, Consumer<Pose2d> robotPoseConsumer) {
    this.vision = vision;
    this.drivetrain = drivetrain;

    this.robotPoseConsumer = robotPoseConsumer;
  }

  public double confidence = 1 - vision.getCurrentAmbiguity();

  public double visionWeight = confidence;

  public double drivetrainWeight = 0.90;

  public Rotation2d pigeonRotation = new Rotation2d(pigeon.getYaw().getValue());

  public Angle resolvedYaw =
      Degrees.of(
          ((visionWeight * visionPose.getRotation().getDegrees() + pigeonRotation.getDegrees())
              / (visionWeight + 1)));

  public Distance resolvedX =
      Meters.of(
          ((visionWeight * visionPose.getMeasureX().in(Meters)
                  + drivetrainWeight * drivetrainPose.getMeasureX().in(Meters))
              / (visionWeight + drivetrainWeight)));

  public Distance resolvedY =
      Meters.of(
          ((visionWeight * visionPose.getMeasureY().in(Meters)
                  + drivetrainWeight * drivetrainPose.getMeasureY().in(Meters))
              / (visionWeight + drivetrainWeight)));

  public Pose2d getPoseFromSupplier(
      Supplier<Distance> x, Supplier<Distance> y, Supplier<Angle> yaw) {
    return new Pose2d(x.get(), y.get(), new Rotation2d(yaw.get()));
  }

  public Pose2d getRobotPose() {
    return getPoseFromSupplier(() -> resolvedX, () -> resolvedY, () -> resolvedYaw);
  }

  @Override
  public void periodic(){
    robotPoseConsumer.accept(getRobotPose());
  }
}
