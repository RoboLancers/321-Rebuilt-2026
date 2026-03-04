/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Align;
import frc.robot.commands.Feed;
import frc.robot.commands.ShootToHub;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.hoodCommands.HoodCommands;
import frc.robot.subsystems.hood.hoodCommands.SetHoodAngle;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.indexerCommands.SetIndexerVelocity;
import frc.robot.subsystems.intakePivot.IntakeConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.intakePivotCommands.GoToAngle;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.rolllercommands.IntakeDefaultVelocity;
import frc.robot.subsystems.intakerollers.rolllercommands.IntakeFuel;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.outtake.commands.SetShooterVelocity;
import frc.robot.subsystems.outtake.commands.ShootFuel;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.RebuiltUtil;
import frc.robot.util.TunableConstant;

public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);

  @Logged(name = "driverController")
  public XboxController getDriverController() {
    return driver.getHID();
  }

  public Tunnel tunnel = new Tunnel();
  public IntakeRollers intakeRollers = new IntakeRollers();
  public Indexer indexer = new Indexer();
  public IntakePivot intakePivot = new IntakePivot();
  public Hood hood = new Hood();
  public Shooter shooter = new Shooter();
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
  // private final IntakeRollers intakeRollers = new IntakeRollers();
  // private final IntakeFuel intakeFuel = new IntakeFuel(intakeRollers);
  // private final Shooter shooter = new Shooter();
  // private final ShootFuel shootFuel = new ShootFuel();
  // private final Hood hood = new Hood();
  // private final IntakePivot intakePivot = new IntakePivot();
  // private final Indexer spindexer = new Indexer();
  // private final Tunnel tunnel = new Tunnel();

  public Trigger slowMode = driver.b();

  @Logged(name = "driverForwardValue")
  public double getDriverForward() {
    double rawJoystick =
        -MathUtil.applyDeadband(
                Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 2),
                DrivetrainConstants.kDriveDeadband)
            * Math.cos(Math.atan2(driver.getLeftX(), driver.getLeftY()));

    return rawJoystick
        * (slowMode.getAsBoolean()
            ? DrivetrainConstants.kSlowModeLinearVelocity.in(MetersPerSecond)
            : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  }

  @Logged(name = "driverStrafeValue")
  public double getDriverStrafe() {
    double rawJoystick =
        -MathUtil.applyDeadband(
                Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 2),
                DrivetrainConstants.kDriveDeadband)
            * Math.sin(Math.atan2(driver.getLeftX(), driver.getLeftY()));

    return rawJoystick
        * (slowMode.getAsBoolean()
            ? DrivetrainConstants.kSlowModeLinearVelocity.in(MetersPerSecond)
            : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  }

  @Logged(name = "driverTurnValue")
  public double getDriverTurn() {
    double rawJoystick =
        -MathUtil.applyDeadband(driver.getRightX(), DrivetrainConstants.kRotationDeadband)
            * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond);

    return rawJoystick;
  }

  @Logged(name = "calculatedHubHeading")
  public Rotation2d getHubHeading() {
    return RebuiltUtil.getHubHeading(drivetrain::getPose);
  }

  @Logged(name = "calculatedHubDistance")
  public Distance getHubDistance() {
    return RebuiltUtil.getHubDistance(drivetrain::getPose);
  }

  @Logged(name = "calculatedScoringPitch")
  public Angle getScoringPitch() {
    return hood.getScoreAngle(getHubDistance());
  }

  @Logged(name = "calculatedScoringVelocity")
  public AngularVelocity getScoringVelocity() {
    return shooter.getScoreVelocity(getHubDistance());
  }

  public RobotContainer() {
    // configureBindings();
    configureTuningBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // NamedCommands.registerCommand("IntakeFuel", intakeFuel);
    // NamedCommands.registerCommand("ShootFuel", shootFuel.releaseFuel(shooter));
  }

  private void configureTuningBindings() {
    hood.setDefaultCommand(HoodCommands.runVolts(hood, () -> Volts.of(0)));
    shooter.setDefaultCommand(Commands.run(() -> shooter.setVelocity(RPM.of(0)), shooter));
    tunnel.setDefaultCommand(Commands.run(() -> tunnel.runAtVelocity(RPM.of(0)), tunnel));

    TunableConstant hoodPitch = new TunableConstant("RobotContainer/hoodPitch/", 0);
    TunableConstant shooterVelocity = new TunableConstant("RobotContainer/shooterVelocity/", 0);
    TunableConstant tunnelVelocity = new TunableConstant("RobotContainer/tunnelVelocity", 0);

    driver.y().onTrue(HoodCommands.homeHoodMagnetic(hood));

    driver
        .rightTrigger()
        .whileTrue(
            HoodCommands.goToAngle(hood, () -> Degrees.of(hoodPitch.get()))
                .alongWith(
                    // new RunAtVelocity(tunnel, () -> RPM.of(tunnelVelocity.get()))
                    Commands.run(() -> tunnel.runAtVelocity(RPM.of(600)), tunnel)
                        .alongWith(
                            ShootFuel.outtakeWithVelocity(
                                shooter, () -> RPM.of(shooterVelocity.get())))));

    driver.leftTrigger().whileTrue(new ShootToHub(tunnel, shooter, hood, this::getHubDistance));
  }

  private void configureBindings() {
    tunnel.setDefaultCommand(Commands.run(() -> tunnel.runAtVelocity(RPM.of(0)), tunnel));
    intakeRollers.setDefaultCommand(new IntakeDefaultVelocity(intakeRollers));
    indexer.setDefaultCommand(new SetIndexerVelocity(indexer, () -> RPM.of(0)));
    intakePivot.setDefaultCommand(new GoToAngle(intakePivot, intakePivot::getTargetAngle));
    hood.setDefaultCommand(new SetHoodAngle(hood, hood::getTargetAngle));
    shooter.setDefaultCommand(new SetShooterVelocity(shooter, () -> RPM.of(0)));

    drivetrain.setDefaultCommand(
        drivetrain.teleopDrive(this::getDriverForward, this::getDriverStrafe, this::getDriverTurn));

    // driver
    // .leftBumper()
    // .whileTrue(new GoToIntakePosition(intakePivot).andThen(new
    // IntakeFuel(intakeRollers)));

    driver.y().onTrue(new GoToAngle(intakePivot, () -> IntakeConstants.kIntakePosition));

    driver.a().onTrue(new GoToAngle(intakePivot, () -> IntakeConstants.kDefaultPosition));

    driver.leftBumper().whileTrue(new IntakeFuel(intakeRollers));

    driver
        .leftTrigger()
        .whileTrue(
            Align.rotateToHubWhileDriving(
                drivetrain,
                this::getDriverForward,
                this::getDriverStrafe,
                this::getHubHeading,
                drivetrain::getPose));
    // driver
    // .rightTrigger()
    // .whileTrue(
    // Score.shootFuelFromAnywhere(
    // drivetrain, shooter, hood, spindexer, tunnel, currentRobotPose));
    // driver.rightBumper().whileTrue(Score.feedFuel(shooter, hood, spindexer,
    // tunnel));

    driver
        .rightTrigger()
        .whileTrue(
            new SetIndexerVelocity(indexer, () -> IndexerConstants.kIndexVelocity)
                .alongWith(new ShootToHub(tunnel, shooter, hood, this::getHubDistance)));

    driver
        .rightBumper()
        .whileTrue(
            new SetIndexerVelocity(indexer, () -> IndexerConstants.kIndexVelocity)
                .alongWith(new Feed(tunnel, shooter, hood)));
  }

  @Logged(name = "autonomousCommand")
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
