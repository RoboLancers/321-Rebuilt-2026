/* (C) RoboLancers 2026 */
package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {

  public Drivetrain drivetrain = Drivetrain.create();

  public Vision vision =
      Vision.create(
          est ->
              drivetrain.addVisionMeasurement(
                  est.estimatedPose().estimatedPose.toPose2d(),
                  est.estimatedPose().timestampSeconds,
                  VecBuilder.fill(
                      est.standardDeviations(),
                      est.standardDeviations(),
                      est.standardDeviations())));

  public PoseEstimatorResolver poseResolver =
      new PoseEstimatorResolver(vision, drivetrain, pose -> drivetrain.addRobotPose(() -> pose));

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
