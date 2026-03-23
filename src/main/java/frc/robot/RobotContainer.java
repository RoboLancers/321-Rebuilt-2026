/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Align;
import frc.robot.commands.ShootAndIndex;
import frc.robot.commands.ShootTesting;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.hoodCommands.HomeHood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.indexerCommands.SetIndexerVelocity;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.intakePivotCommands.GoToAngle;
import frc.robot.subsystems.intakePivot.intakePivotCommands.HomeIntakePivot;
import frc.robot.subsystems.intakePivot.intakePivotCommands.Tune;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.rolllercommands.IntakeDefaultVelocity;
import frc.robot.subsystems.intakerollers.rolllercommands.IntakeWithVoltage;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.outtake.commands.SetShooterVelocity;
import frc.robot.subsystems.outtake.commands.ShootFuel;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.RebuiltUtil;
import frc.robot.util.TunableConstant;

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

  public Trigger driveInverted = driver.leftTrigger();
  public Trigger turnInverted = driver.rightTrigger();

  public double getDriverForward1() {

    double rawJoystick =
        MathUtil.applyDeadband(
                Math.pow(Math.hypot(driver.getLeftX(), driver.getLeftY()), 1),
                DrivetrainConstants.kDriveDeadband)
            * Math.sin(Math.atan2(-driver.getLeftY(), driver.getLeftX()));

    return rawJoystick
        * (slowMode.getAsBoolean()
            ? DrivetrainConstants.kSlowModeLinearVelocity.in(MetersPerSecond)
            : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  }

  public double getDriverStrafe1() {

    double rawJoystick =
        MathUtil.applyDeadband(
                Math.pow(Math.hypot(driver.getLeftX(), driver.getLeftY()), 1),
                DrivetrainConstants.kDriveDeadband)
            * Math.cos(Math.atan2(driver.getLeftY(), driver.getRightX()));

    return rawJoystick
        * (slowMode.getAsBoolean()
            ? DrivetrainConstants.kSlowModeLinearVelocity.in(MetersPerSecond)
            : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  }

  public double getDriverForward2() {
    double rawJoystick =
        -MathUtil.applyDeadband(
                Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 1),
                DrivetrainConstants.kDriveDeadband)
            * Math.cos(Math.atan2(driver.getLeftX(), driver.getLeftY()));

    double processedJoystick =
        rawJoystick
            * (slowMode.getAsBoolean()
                ? DrivetrainConstants.kSlowModeLinearVelocity.in(MetersPerSecond)
                : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
    return driveInverted.getAsBoolean() ? -processedJoystick : processedJoystick;
  }

  public double getDriverStrafe2() {
    double rawJoystick =
        -MathUtil.applyDeadband(
                Math.pow(Math.hypot(driver.getLeftY(), driver.getLeftX()), 1),
                DrivetrainConstants.kDriveDeadband)
            * Math.sin(Math.atan2(driver.getLeftX(), driver.getLeftY()));

    double processedJoystick =
        rawJoystick
            * (slowMode.getAsBoolean()
                ? DrivetrainConstants.kSlowModeLinearVelocity.in(MetersPerSecond)
                : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));

    return driveInverted.getAsBoolean() ? -processedJoystick : processedJoystick;
  }

  public double getDriverTurn2() {
    double processedJoystick =
        -MathUtil.applyDeadband(driver.getRightX(), DrivetrainConstants.kRotationDeadband)
            * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond);

    return turnInverted.getAsBoolean() ? -processedJoystick : processedJoystick;
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
    configureBindings();
    // configureTuningBindings();
    ShootFuel shootFuel = new ShootFuel();
    ShootAndIndex shootInAuto =
        new ShootAndIndex(tunnel, shooter, hood, indexer, this::getHubDistance);
    NamedCommands.registerCommand(
        "ShootFuel", Commands.run(() -> shooter.setVoltage(Volts.of(10))));
        
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // autoChooser.addOption("Test Auto", new PathPlannerAuto("Test Auto"));
    autoChooser.addOption("Disrupt Auto", new PathPlannerAuto("Disrupt Auto"));
    // NamedCommands.registerCommand("IntakeFuel", intakeFuel);
  }

  private void configureTuningBindings() {
    // drivetrain.setDefaultCommand(
    //     drivetrain.driveRobotCentric(this::getDriverForward, this::getDriverStrafe,
    // this::getDriverTurn));

    intakeRollers.setDefaultCommand(
        Commands.run(() -> intakeRollers.setVoltage(Volts.of(0)), intakeRollers));

    intakePivot.setDefaultCommand(
        Commands.run(() -> intakePivot.setVoltage(Volts.of(0)), intakePivot));

    // indexer.setDefaultCommand(Commands.run(() -> indexer.setVoltage(Volts.of(0)), indexer));

    driver.a().whileTrue(new Tune(intakePivot));

    driver.y().whileTrue(new HomeIntakePivot(intakePivot));

    driver.x().whileTrue(new IntakeWithVoltage(intakeRollers));

    // driver.x().whileTrue(new SetIndexerVelocity(indexer, () -> IndexerConstants.kIndexVelocity));

    // driver.rightTrigger().whileTrue(new Release(tunnel, shooter, hood));
  }

  public TunableConstant tuningHoodAngle = new TunableConstant("Robot Container/hood pitch", 0);

  private void configureBindings() {
    tunnel.setDefaultCommand(Commands.run(() -> tunnel.runAtVelocity(RPM.of(0)), tunnel));
    intakeRollers.setDefaultCommand(new IntakeDefaultVelocity(intakeRollers));
    indexer.setDefaultCommand(new SetIndexerVelocity(indexer, () -> RPM.of(0)));
    intakePivot.setDefaultCommand(new GoToAngle(intakePivot, intakePivot::getTargetAngle));
    hood.setDefaultCommand(Commands.run(() -> hood.runVolts(Volts.of(0)), hood));
    shooter.setDefaultCommand(new SetShooterVelocity(shooter, () -> RPM.of(0)));

    drivetrain.setDefaultCommand(
        drivetrain.teleopDrive(
            this::getDriverForward2, this::getDriverStrafe2, this::getDriverTurn2));

    // driver.a().onTrue(Commands.runOnce(()->drivetrain.getPigeon2().setYaw(Degrees.of(0))));

    driver.y().whileTrue(new HomeHood(hood));

    // driver
    // .leftBumper()
    // .whileTrue(new GoToIntakePosition(intakePivot).andThen(new
    // IntakeFuel(intakeRollers)));

    // driver.leftBumper().whileTrue(new IntakeWithVoltage(intakeRollers));

    // driver.y().whileTrue(new GoToVoltage(intakePivot, () -> Volts.of(-3)));

    // driver.a().onTrue(new GoToAngle(intakePivot, () -> IntakeConstants.kDefaultPosition));

    // driver.leftBumper().whileTrue(new IntakeFuel(intakeRollers));

    driver
        .leftTrigger()
        .whileTrue(
            Align.rotateToHubWhileDriving(
                drivetrain,
                this::getDriverForward,
                this::getDriverStrafe,
                this::getHubHeading,
                drivetrain::getPose));

    //  driver.rightTrigger().whileTrue((new ShootToHub(tunnel, shooter, hood,
    // this::getHubDistance)));

    //   driver.x().whileTrue(new SetIndexerVelocity(indexer, () ->
    // IndexerConstants.kIndexVelocity));

    driver
        .rightTrigger()
        .whileTrue(new ShootAndIndex(tunnel, shooter, hood, indexer, this::getHubDistance));

    driver
        .rightBumper()
        .whileTrue(
            new ShootTesting(
                tunnel,
                shooter,
                hood,
                indexer,
                this::getHubDistance,
                () -> Degrees.of(tuningHoodAngle.get())));

    // driver
    //     .rightBumper()
    //     .whileTrue(
    //         new Feed(tunnel, shooter, hood));

    // driver.rightTrigger().whileTrue(new ShootTest(tunnel, shooter, indexer));
  }

  @Logged(name = "autonomousCommand")
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}
