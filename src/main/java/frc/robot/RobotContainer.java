/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Align;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.RebuiltUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RobotContainer {

  public CommandXboxController driver = new CommandXboxController(0);
  public CommandXboxController manipulator = new CommandXboxController(1);

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

  //   public PoseEstimatorResolver poseEstimatorResolver =
  //       new PoseEstimatorResolver(vision, drivetrain);

  // public Hood hood = new Hood();
  // public Outtake outtake = new Outtake();
  // public IntakePivot intakePivot = new IntakePivot();
  // public IntakeRollers intakeRollers = new IntakeRollers();
  // public Indexer spindexer = new Indexer();
  // public Tunnel tunnel = new Tunnel();

  public Trigger slowMode = driver.b();

  private DoubleSupplier driverForward =
      () ->
          -MathUtil.applyDeadband(
                  Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 2),
                  DrivetrainConstants.kDriveDeadband)
              * Math.cos(Math.atan2(driver.getLeftX(), driver.getLeftY()))
              * (slowMode.getAsBoolean()
                  ? 1.5
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));

  private DoubleSupplier driverStrafe =
      () ->
          -MathUtil.applyDeadband(
                  Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 2),
                  DrivetrainConstants.kDriveDeadband)
              * Math.sin(Math.atan2(driver.getLeftX(), driver.getLeftY()))
              * (slowMode.getAsBoolean()
                  ? 1.5
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));

  private DoubleSupplier driverTurn =
      () ->
          -MathUtil.applyDeadband(driver.getRightX(), DrivetrainConstants.kRotationDeadband)
              * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond);

  public Supplier<Pose2d> currentRobotPose =
      () -> drivetrain.getPose(); // poseEstimatorResolver.getRobotPose();

  public Pose2d targetPose =
      new Pose2d(Meters.of(13.775), Meters.of(5.598), new Rotation2d(Degrees.of(-141.88)));

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn));
    // hood.setDefaultCommand(HoodCommands.goToTravelAngle(hood));
    // outtake.setDefaultCommand(OuttakeFuel.outtakeWithVelocity(outtake, ()->RPM.of(0)));
    // spindexer.setDefaultCommand(Index.goToVelocity(spindexer,()->RPM.of(0)));
    // tunnel.setDefaultCommand(new RunAtVelocity(tunnel,RPM.of(0)));
    // intakePivot.setDefaultCommand(new GoToDefaultPosition(intakePivot));

    driver
        .leftTrigger()
        .whileTrue(drivetrain.driveToFieldPoseCommand(() -> targetPose, currentRobotPose));
    driver
        .leftBumper()
        .whileTrue(
            drivetrain
                .driveToFieldPoseCommand(() -> targetPose, currentRobotPose)
                .until(
                    () ->
                        drivetrain.atPoseSetpoint(
                            Meters.of(0.02), Degrees.of(2), currentRobotPose)));
    driver
        .rightTrigger()
        .whileTrue(
            Align.rotateToHubWhileDriving2(
                drivetrain,
                driverForward,
                driverStrafe,
                () -> RebuiltUtil.getHubHeading(currentRobotPose),
                currentRobotPose));
    driver.rightBumper().whileTrue(Align.alignToApriltag(drivetrain, () -> 10, currentRobotPose));

    driver
        .y()
        .whileTrue(
            drivetrain.teleopDriveWithHeading(
                driverForward,
                driverStrafe,
                () -> RebuiltUtil.getHubHeading(currentRobotPose),
                currentRobotPose));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
