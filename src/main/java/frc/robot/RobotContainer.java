/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Align;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.RebuiltUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged
public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);

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

  @NotLogged private final SendableChooser<Command> autoChooser;
  // private final IntakeRollers intakeRollers = new IntakeRollers();
  // private final IntakeFuel intakeFuel = new IntakeFuel(intakeRollers);
  // private final Shooter shooter = new Shooter();
  // private final ShootFuel shootFuel = new ShootFuel();
  // private final Hood hood = new Hood();
  // private final IntakePivot intakePivot = new IntakePivot();
  // private final Indexer spindexer = new Indexer();
  // private final Tunnel tunnel = new Tunnel();

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

  private Supplier<Pose2d> currentRobotPose = () -> drivetrain.getPose();

  private Supplier<Rotation2d> hubHeading = () -> RebuiltUtil.getHubHeading(currentRobotPose);

  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // NamedCommands.registerCommand("IntakeFuel", intakeFuel);
    // NamedCommands.registerCommand("ShootFuel", shootFuel.releaseFuel(shooter));
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn));
    // intakeRollers.setDefaultCommand(
    //     Commands.run(() -> intakeRollers.setVoltage(Volts.of(0)), intakeRollers));
    // shooter.setDefaultCommand(ShootFuel.outtakeWithVoltage(shooter, () -> Volts.of(0)));
    // hood.setDefaultCommand(HoodCommands.goToTravelAngle(hood));
    // intakePivot.setDefaultCommand(new GoToDefaultPosition(intakePivot));
    // spindexer.setDefaultCommand(Index.setVoltage(spindexer, () -> Volts.of(0)));
    // tunnel.setDefaultCommand(new RunAtVelocity(tunnel, RPM.of(0)));

    // driver
    //     .leftBumper()
    //     .whileTrue(new GoToIntakePosition(intakePivot).andThen(new IntakeFuel(intakeRollers)));
    driver
        .leftTrigger()
        .whileTrue(
            Align.rotateToHubWhileDriving(
                drivetrain, driverForward, driverStrafe, hubHeading, currentRobotPose));
    // driver
    //     .rightTrigger()
    //     .whileTrue(
    //         Score.shootFuelFromAnywhere(
    //             drivetrain, shooter, hood, spindexer, tunnel, currentRobotPose));
    // driver.rightBumper().whileTrue(Score.feedFuel(shooter, hood, spindexer, tunnel));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
