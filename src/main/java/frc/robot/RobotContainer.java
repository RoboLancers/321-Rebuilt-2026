/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;

public class RobotContainer {

  CommandXboxController driver = new CommandXboxController(0);
  Drivetrain drivetrain = Drivetrain.create();
  Vision vision =
      Vision.create(
          est ->
              drivetrain.addVisionMeasurement(
                  est.estimatedPose().estimatedPose.toPose2d(),
                  est.estimatedPose().timestampSeconds,
                  VecBuilder.fill(
                      est.standardDeviation(), est.standardDeviation(), est.standardDeviation())));

  private DoubleSupplier driverForward =
      () ->
          -MathUtil.applyDeadband(
                  Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 2),
                  DrivetrainConstants.kDriveDeadband)
              * Math.cos(Math.atan2(driver.getLeftX(), driver.getLeftY()))
              * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond);

  private DoubleSupplier driverStrafe =
      () ->
          -MathUtil.applyDeadband(
                  Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 2),
                  DrivetrainConstants.kDriveDeadband)
              * Math.sin(Math.atan2(driver.getLeftX(), driver.getLeftY()))
              * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond);

  private DoubleSupplier driverTurn =
      () ->
          -MathUtil.applyDeadband(driver.getRightX(), DrivetrainConstants.kRotationDeadband)
              * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond);

  public RobotContainer() {

    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
