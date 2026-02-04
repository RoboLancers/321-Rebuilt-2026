/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Align;
import frc.robot.commands.Score;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.hoodCommands.HoodCommands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.indexerCommands.Index;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.intakePivotCommands.GoToDefaultPosition;
import frc.robot.subsystems.intakePivot.intakePivotCommands.GoToIntakePosition;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.rolllercommands.IntakeFuel;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.outtake.commands.ShootFuel;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.tunnelCommands.RunAtVelocity;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.RebuiltUtil;

public class RobotContainer {

  public CommandXboxController driver = new CommandXboxController(0);
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

  private final SendableChooser<Command> autoChooser;
  private final IntakeRollers intakeRollers = new IntakeRollers();
  private final IntakeFuel intakeFuel = new IntakeFuel(intakeRollers);
  private final Shooter shooter = new Shooter();
  private final ShootFuel shootFuel = new ShootFuel();
  private final Hood hood = new Hood();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Indexer spindexer = new Indexer();
  private final Tunnel tunnel = new Tunnel();

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

    private Supplier<Pose2d> currentRobotPose = ()->drivetrain.getPose();

    private Supplier<Rotation2d> hubHeading = ()->RebuiltUtil.getHubHeading(currentRobotPose);

  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("IntakeFuel", intakeFuel);
    NamedCommands.registerCommand("ShootFuel", shootFuel.releaseFuel(shooter));
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn));
    intakeRollers.setDefaultCommand(Commands.run(()->intakeRollers.setVoltage(0)));
    shooter.setDefaultCommand(ShootFuel.outtakeWithVoltage(shooter, ()->Volts.of(0)));
    hood.setDefaultCommand(HoodCommands.goToTravelAngle(hood));
    intakePivot.setDefaultCommand(new GoToDefaultPosition(intakePivot));
    spindexer.setDefaultCommand(Index.setVoltage(spindexer, ()->Volts.of(0)));
    tunnel.setDefaultCommand(new RunAtVelocity(tunnel, RPM.of(0)));

    driver.leftBumper().whileTrue(new GoToIntakePosition(intakePivot).andThen(new IntakeFuel(intakeRollers)));
    driver.leftTrigger().whileTrue(Align.rotateToHubWhileDriving(drivetrain, driverForward, driverStrafe, hubHeading, currentRobotPose));
    driver.rightTrigger().whileTrue(Score.shootFuelFromAnywhere(drivetrain, shooter, hood, currentRobotPose));
    driver.rightBumper().whileTrue(HoodCommands.goToNeutralFeedAngle(hood).andThen(ShootFuel.feedNeutralZone(shooter)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
