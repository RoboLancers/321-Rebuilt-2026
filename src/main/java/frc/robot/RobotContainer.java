/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Align;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.hoodCommands.HoodCommands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.indexerCommands.Index;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.intakePivotCommands.GoToDefaultPosition;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.commands.OuttakeFuel;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.tunnelCommands.RunAtVelocity;
import frc.robot.subsystems.vision.Vision;

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

  public PoseEstimatorResolver poseEstimatorResolver =
      new PoseEstimatorResolver(vision, drivetrain, pose -> drivetrain.addRobotPose(() -> pose));

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


  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("IntakeFuel", intakeFuel);
    NamedCommands.registerCommand("ShootFuel", shootFuel.releaseFuel(shooter));
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn));
    // hood.setDefaultCommand(HoodCommands.goToTravelAngle(hood));
    // outtake.setDefaultCommand(OuttakeFuel.outtakeWithVelocity(outtake, ()->RPM.of(0)));
    // spindexer.setDefaultCommand(Index.goToVelocity(spindexer,()->RPM.of(0)));
    // tunnel.setDefaultCommand(new RunAtVelocity(tunnel,RPM.of(0)));
    // intakePivot.setDefaultCommand(new GoToDefaultPosition(intakePivot));

    driver.leftTrigger().whileTrue(Align.rotateToHubWhileDriving(drivetrain, driverForward, driverTurn));
    driver.rightTrigger().whileTrue(Align.alignToApriltag(()->));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
