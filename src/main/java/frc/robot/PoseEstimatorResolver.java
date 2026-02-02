/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;

public class PoseEstimatorResolver extends SubsystemBase {
  public Drivetrain drivetrain;
  public Vision vision;

  public PoseEstimatorResolver(
      Vision vision, Drivetrain drivetrain) {
    this.vision = vision;
    this.drivetrain = drivetrain;
  }

  public Pose2d getRobotPose(){

   Pigeon2 pigeon = drivetrain.getPigeon2();

   Pose2d visionPose = vision.getLatestBestPose();

   Pose2d drivetrainPose = drivetrain.getPose();

   double confidence = 1 - vision.getCurrentAmbiguity();

   double visionWeight = confidence;

   double drivetrainWeight = 0.90;

   Rotation2d pigeonRotation = new Rotation2d(pigeon.getYaw().getValue());

   Angle resolvedYaw = 
      Degrees.of(
          ((visionWeight * visionPose.getRotation().getDegrees() + pigeonRotation.getDegrees())
              / (visionWeight + 1)));

   Distance resolvedX =
      Meters.of(
          ((visionWeight * visionPose.getMeasureX().in(Meters)
                  + drivetrainWeight * drivetrainPose.getMeasureX().in(Meters))
              / (visionWeight + drivetrainWeight)));

   Distance resolvedY =
      Meters.of(
          ((visionWeight * visionPose.getMeasureY().in(Meters)
                  + drivetrainWeight * drivetrainPose.getMeasureY().in(Meters))
              / (visionWeight + drivetrainWeight)));
    
    SmartDashboard.putNumber("Resolver Vision Pose X", visionPose.getX());

    SmartDashboard.putNumber("Resolver Vision Pose Y", visionPose.getY());
    
    SmartDashboard.putNumber("Resolver Vision Pose Yaw", visionPose.getRotation().getDegrees());
    
    SmartDashboard.putNumber("Resolver Drivetrain Pose X", drivetrainPose.getX());
    
    SmartDashboard.putNumber("Resolver Drivetrain Pose Y", drivetrainPose.getY());
    
    SmartDashboard.putNumber("Resolver Drivetrain Pose Yaw", drivetrainPose.getRotation().getDegrees());

    SmartDashboard.putNumber("Pigeon Yaw", pigeonRotation.getDegrees());

    SmartDashboard.putNumber("Resolver Robot Pose X", resolvedX.in(Meters));
    
    SmartDashboard.putNumber("Resolver Robot Pose Y", resolvedY.in(Meters));
    
    SmartDashboard.putNumber("Resolver Robot Pose Yaw", resolvedYaw.in(Degrees));
 
    return getPoseFromSupplier(() -> resolvedX, () -> resolvedY, () -> resolvedYaw);
  }

   public Pose2d getPoseFromSupplier(
      Supplier<Distance> x, Supplier<Distance> y, Supplier<Angle> yaw) {
    return new Pose2d(x.get(), y.get(), new Rotation2d(yaw.get()));
  }


  @Override
  public void periodic() {
    getRobotPose();
  }
}
