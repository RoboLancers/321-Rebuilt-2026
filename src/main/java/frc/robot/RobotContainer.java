/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Align;
import frc.robot.commands.Feed;
import frc.robot.commands.Release;
import frc.robot.commands.ShootAndIndex;
import frc.robot.commands.StaticShoot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.indexerCommands.SetIndexerVelocity;
import frc.robot.subsystems.intakePivot.IntakeConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.intakePivotCommands.GoToAngle;
import frc.robot.subsystems.intakerollers.IntakeRollerConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.rolllercommands.IntakeFuel;
import frc.robot.subsystems.intakerollers.rolllercommands.SetIntakeVelocity;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.outtake.commands.SetShooterVelocity;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.tunnelCommands.RunAtVelocity;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.RebuiltUtil;

@Logged
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
  public Vision vision = Vision.create(drivetrain::addVisionMeasurement, null);

  public Field2d latestPoseField = new Field2d();

  @Logged(name = "latest robot pose")
  public Pose3d getLatestCameraPose() {
    return vision.getLatestBestPose();
  }

  private SendableChooser<Command> autoChooser;

  public Trigger slowMode = driver.b();

  @Logged(name = "driverForwardValue")
  public double getDriverForward() {
    double rawJoystick =
        -MathUtil.applyDeadband(
                Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 1),
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
                Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 1),
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

  @Logged(name = "hubAngle")
  public double getHubAngle() {
    return getHubHeading().getDegrees();
  }

  @Logged(name = "robotAngle")
  public double getRobotAngle() {
    return drivetrain.getPose().getRotation().getDegrees();
  }

  @Logged(name = "shooterAngle")
  public double getShooterAngle() {
    return getRobotAngle() - 90;
  }

  @Logged(name = "finalShooterAngle")
  public double getFinalShooterAngle() {
    return Math.floorMod(Math.round(getShooterAngle()), 360);
  }

  @Logged(name = "finalHubAngle")
  public double getFinalHubAngle() {
    return Math.floorMod(Math.round(getHubAngle()), 360);
  }

  @Logged(name = "shooterAtHeading")
  public boolean shooterAtHeading() {
    return drivetrain.shooterAtHeading(getHubHeading());
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
    configureBindings();
    // configureTuningBindings();
    configureNamedAutoCommands();
    configureAutoChooser();
  }

  private void configureNamedAutoCommands() {
    IntakeFuel intakeFuel = new IntakeFuel(intakeRollers, intakePivot);
    GoToAngle intakePivotStow = new GoToAngle(intakePivot, () -> IntakeConstants.kStowedPosition);
    GoToAngle intakePivotOut = new GoToAngle(intakePivot, () -> IntakeConstants.kIntakePosition);
    ParallelRaceGroup intakeInAuto = new ParallelRaceGroup(intakeFuel, intakePivotOut);
    Command align =
        Align.rotateToHub(drivetrain, () -> 0, () -> 0, this::getHubHeading, drivetrain::getPose);
    ParallelRaceGroup alignInAuto = new ParallelRaceGroup(align);
    ShootAndIndex shootInAuto =
        new ShootAndIndex(tunnel, shooter, hood, indexer, this::getHubDistance);

    NamedCommands.registerCommand("IntakePivotStow", intakePivotStow);
    NamedCommands.registerCommand("IntakeFuel", intakeInAuto);
    NamedCommands.registerCommand("IntakePivotOut", intakePivotOut);
    NamedCommands.registerCommand("ShootFuel", shootInAuto);
    NamedCommands.registerCommand("Align", alignInAuto);
  }

  private void configureAutoChooser() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureTuningBindings() {

    intakeRollers.setDefaultCommand(
        Commands.run(() -> intakeRollers.setVoltage(Volts.of(0)), intakeRollers));

    intakePivot.setDefaultCommand(
        Commands.run(() -> intakePivot.setVoltage(Volts.of(0)), intakePivot));

    indexer.setDefaultCommand(Commands.run(() -> indexer.setVoltage(Volts.of(0)), indexer));

    driver.rightTrigger().whileTrue(new Release(tunnel, shooter, indexer));
  }

  private void configureBindings() {
    tunnel.setDefaultCommand(new RunAtVelocity(tunnel, () -> RPM.of(0)));
    intakeRollers.setDefaultCommand(
        Commands.run(() -> intakeRollers.setVoltage(Volts.of(0)), intakeRollers));
    indexer.setDefaultCommand(new SetIndexerVelocity(indexer, () -> RPM.of(0)));
    intakePivot.setDefaultCommand(
        new GoToAngle(intakePivot, () -> IntakeConstants.kStowedPosition));
    hood.setDefaultCommand(Commands.run(() -> hood.runVolts(Volts.of(0)), hood));
    shooter.setDefaultCommand(new SetShooterVelocity(shooter, () -> OuttakeConstants.kDefaultRPM));

    drivetrain.setDefaultCommand(
        drivetrain.teleopDrive(this::getDriverForward, this::getDriverStrafe, this::getDriverTurn));

    driver
        .leftBumper()
        .whileTrue(
            new GoToAngle(intakePivot, () -> IntakeConstants.kIntakePosition)
                .alongWith(
                    new // TODO: change to and then once end criteria is reimplemented
                    IntakeFuel(intakeRollers, intakePivot)));

    driver.y().toggleOnTrue(new GoToAngle(intakePivot, () -> IntakeConstants.kIntakePosition));

    driver
        .rightTrigger()
        .whileTrue(
            (Align.rotateToHub(
                    drivetrain,
                    this::getDriverForward,
                    this::getDriverStrafe,
                    this::getHubHeading,
                    drivetrain::getPose))
                .andThen(
                    new ShootAndIndex(tunnel, shooter, hood, indexer, this::getHubDistance)
                        .alongWith(
                            Align.lockOnHub(
                                drivetrain,
                                this::getDriverForward,
                                this::getDriverStrafe,
                                this::getHubHeading,
                                drivetrain::getPose))));

    driver
        .leftTrigger()
        .whileTrue(
            Align.lockOnHub(
                drivetrain,
                this::getDriverForward,
                this::getDriverStrafe,
                this::getHubHeading,
                drivetrain::getPose));

    driver
        .rightBumper()
        .whileTrue(
            Align.faceAllianceZone(drivetrain, this::getDriverForward, this::getDriverStrafe)
                .alongWith(new Feed(tunnel, shooter, hood, indexer)));

    driver.a().whileTrue(new StaticShoot(tunnel, shooter, indexer));
    driver.b().whileTrue(new Feed(tunnel, shooter, hood, indexer));
    driver
        .x()
        .whileTrue(
            new SetIntakeVelocity(
                intakeRollers, intakePivot, () -> IntakeRollerConstants.kReleaseVelocity));
    driver.povLeft().whileTrue(new Release(tunnel, shooter, indexer));
  }

  @Logged(name = "autonomousCommand")
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}
