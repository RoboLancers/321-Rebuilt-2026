/* (C) RoboLancers 2026 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotConstants;
import frc.robot.util.MyAlliance;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged
public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

  // driveToPose PID controllers
  ProfiledPIDController xPoseController =
      new ProfiledPIDController(
          DrivetrainConstants.kTranslationGains.kP(),
          DrivetrainConstants.kTranslationGains.kI(),
          DrivetrainConstants.kTranslationGains.kD(),
          DrivetrainConstants.kTranslationConstraints);

  ProfiledPIDController yPoseController =
      new ProfiledPIDController(
          DrivetrainConstants.kTranslationGains.kP(),
          DrivetrainConstants.kTranslationGains.kI(),
          DrivetrainConstants.kTranslationGains.kD(),
          DrivetrainConstants.kTranslationConstraints);

  ProfiledPIDController thetaController =
      new ProfiledPIDController(
          DrivetrainConstants.kHeadingGains.kP(),
          DrivetrainConstants.kHeadingGains.kI(),
          DrivetrainConstants.kHeadingGains.kD(),
          DrivetrainConstants.kHeadingConstraints);

  public record AlignmentSetpoint(Pose2d pose, boolean isFinalSetpoint) {}

  private final SwerveRequest.FieldCentric fieldCentricRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  private final SwerveRequest.ApplyRobotSpeeds robotCentricRequest =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true)
          .withHeadingPID(
              DrivetrainConstants.kTuneHeadingGains.kP(),
              DrivetrainConstants.kTuneHeadingGains.kI(),
              DrivetrainConstants.kTuneHeadingGains.kD())
          .withRotationalDeadband(0.1);

  private final Field2d poseField = new Field2d();

  private AlignmentSetpoint alignmentSetpoint = new AlignmentSetpoint(Pose2d.kZero, true);

  public static Drivetrain create() {
    return new Drivetrain(
        TunerConstants.kTunerDrivetrain.getDriveTrainConstants(),
        TunerConstants.kTunerDrivetrain.getModuleConstants());
  }

  public Drivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    // create CTRE Swervedrivetrain
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    configNeutralMode(NeutralModeValue.Brake);
    configurePoseControllers();

    SmartDashboard.putData("Drivetrain Pose Field", poseField);
  }

  void configurePoseControllers() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  ChassisSpeeds flipFieldSpeeds(ChassisSpeeds speeds) {
    return MyAlliance.isRed()
        ? new ChassisSpeeds(
            -speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond)
        : speeds;
  }

  Rotation2d flipRotation(Rotation2d rotation) {
    return MyAlliance.isRed() ? rotation.plus(Rotation2d.k180deg) : rotation;
  }

  boolean atPoseSetpoint() {
    return atPoseSetpoint(
        DrivetrainConstants.kAlignmentSetpointTranslationTolerance,
        DrivetrainConstants.kAlignmentSetpointRotationTolerance);
  }

  boolean atFinalPoseSetpoint() {
    if (!getAlignmentSetpoint().isFinalSetpoint()) return false;
    return atPoseSetpoint();
  }

  // drive with heading controlled by PID
  Command driveFixedHeading(
      DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Rotation2d> rotation) {
    return run(
        () ->
            driveFixedHeading(
                translationX.getAsDouble(), translationY.getAsDouble(), rotation.get()));
  }

  Command driveToFieldPose(Supplier<AlignmentSetpoint> pose) {
    return driveToFieldPose(pose, this::getPose);
  }

  Command driveToFieldPose(Supplier<AlignmentSetpoint> pose, Supplier<Pose2d> currentPose) {
    return runOnce(
            () -> {
              ChassisSpeeds speeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      getChassisSpeeds(), currentPose.get().getRotation());

              xPoseController.reset(
                  currentPose.get().getTranslation().getX(), speeds.vxMetersPerSecond);

              yPoseController.reset(
                  currentPose.get().getTranslation().getY(), speeds.vyMetersPerSecond);

              thetaController.reset(
                  currentPose.get().getRotation().getRadians(), speeds.omegaRadiansPerSecond);
            })
        .andThen(
            run(
                () -> {
                  setAlignmentSetpoint(pose.get());
                  driveToFieldPose(pose.get().pose, currentPose.get());
                }));
  }

  public Command teleopDrive(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () -> {
          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble(),
                  RobotConstants.kRobotLoopPeriod.in(Seconds));

          setControl(
              fieldCentricRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond)
                  .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
        });
  }

  public Command teleopDriveFixedHeading(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY) {
    return run(
        () -> {
          Rotation2d desiredRotation =
              flipRotation(
                  Rotation2d.fromRadians(
                      Math.atan2(rotationX.getAsDouble(), rotationY.getAsDouble())));

          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  0,
                  RobotConstants.kRobotLoopPeriod.in(Seconds));

          setControl(
              fieldCentricFacingAngleRequest
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(desiredRotation)
                  .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
        });
  }

  public Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () -> {
          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble(),
                  RobotConstants.kRobotLoopPeriod.in(Seconds));

          setControl(
              fieldCentricRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
        });
  }
  ;

  public Command driveRobotCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () ->
            driveRobotCentric(
                translationX.getAsDouble(),
                translationY.getAsDouble(),
                rotation.getAsDouble(),
                DriveFeedforwards.zeros(4)));
  }

  public void driveRobotCentric(
      double translationX, double translationY, double rotation, DriveFeedforwards feedforwards) {

    var speeds = new ChassisSpeeds(translationX, translationY, rotation);

    setControl(
        robotCentricRequest
            .withSpeeds(speeds)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
  }

  public void driveToFieldPose(Pose2d pose, Pose2d currentPose) {

    double ffFactor = 0;

    ChassisSpeeds targetSpeeds =
        ChassisSpeeds.discretize(
            xPoseController.calculate(getPose().getX(), pose.getX())
                + xPoseController.getSetpoint().velocity * ffFactor,
            yPoseController.calculate(getPose().getY(), pose.getY())
                + yPoseController.getSetpoint().velocity * ffFactor,
            thetaController.calculate(
                    getPose().getRotation().getRadians(), pose.getRotation().getRadians())
                + thetaController.getSetpoint().velocity * ffFactor,
            RobotConstants.kRobotLoopPeriod.in(Seconds));

    if (currentPose.getTranslation().getDistance(alignmentSetpoint.pose().getTranslation())
        < DrivetrainConstants.kAlignmentSetpointTranslationTolerance.in(Meters))
      targetSpeeds = new ChassisSpeeds(0, 0, targetSpeeds.omegaRadiansPerSecond);

    if (Math.abs(
            currentPose.getRotation().minus(alignmentSetpoint.pose().getRotation()).getDegrees())
        < DrivetrainConstants.kAlignmentSetpointRotationTolerance.in(Degrees))
      targetSpeeds =
          new ChassisSpeeds(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, 0);

    setControl(
        fieldCentricRequest
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocityX(targetSpeeds.vxMetersPerSecond)
            .withVelocityY(targetSpeeds.vyMetersPerSecond)
            .withRotationalRate(targetSpeeds.omegaRadiansPerSecond)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance));
  }

  // public Command driveToPosePP(Pose2d pose) {
  //   PathConstraints constraints =
  //       new PathConstraints(
  //           DrivetrainConstants.kMaxPathVelocity,
  //           DrivetrainConstants.kMaxPathAcceleration,
  //           DrivetrainConstants.kMaxPathAngularVelocity,
  //           DrivetrainConstants.kMaxPathAngularAcceleration);
  //   return AutoBuilder.pathfindToPose(pose, constraints, 0.0);
  // }

  // drive with absolute heading control
  public void driveFixedHeading(double translationX, double translationY, Rotation2d rotation) {
    var speeds =
        ChassisSpeeds.discretize(
            translationX, translationY, 0, RobotConstants.kRobotLoopPeriod.in(Seconds));

    setControl(
        fieldCentricFacingAngleRequest
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withTargetDirection(
                MyAlliance.isRed() ? rotation.plus(Rotation2d.fromDegrees(180)) : rotation)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
  }

  public void setAlignmentSetpoint(AlignmentSetpoint setpoint) {
    alignmentSetpoint = setpoint;
  }

  public boolean atPoseSetpoint(Distance tranTol, Angle rotTol) {
    final var currentPose = getPose();
    return currentPose.getTranslation().getDistance(alignmentSetpoint.pose().getTranslation())
            < tranTol.in(Meters)
        && Math.abs(
                currentPose
                    .getRotation()
                    .minus(alignmentSetpoint.pose().getRotation())
                    .getDegrees())
            < rotTol.in(Degrees);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < super.getModules().length; i++) {
      super.getModule(i).apply(new ModuleRequest().withState(states[i]));
    }
  }

  @Logged(name = "MeasuredModuleStates")
  public SwerveModuleState[] getMeasuredModuleStates() {
    return super.getState().ModuleStates;
  }

  @Logged(name = "MeasuredModulePositions")
  public SwerveModulePosition[] getModulePositions() {
    return super.getState().ModulePositions;
  }

  @Logged(name = "TargetModuleStates")
  public SwerveModuleState[] getTargetModuleStates() {
    return super.getState().ModuleTargets;
  }

  @Logged(name = "MeasuredRobotPose")
  public Pose2d getPose() {
    return super.getState().Pose;
  }

  @Logged(name = "MeasuredRobotRelativeChassisSpeeds")
  public ChassisSpeeds getChassisSpeeds() {
    return super.getState().Speeds;
  }

  @Logged(name = "MeasuredHeadingRad")
  public Rotation2d getHeading() {
    return new Rotation2d(super.getPigeon2().getYaw().getValue().in(Radians));
  }

  public AlignmentSetpoint getAlignmentSetpoint() {
    return alignmentSetpoint;
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPose, double timeStampSeconds, Matrix<N3, N1> standardDeviations) {
    super.addVisionMeasurement(
        visionRobotPose, Utils.fpgaToCurrentTime(timeStampSeconds), standardDeviations);
  }

  @NotLogged private Alliance lastAlliance;

  public void periodic() {

    if (DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                if (lastAlliance == allianceColor) return;
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
                lastAlliance = allianceColor;
              });
    }

    poseField.setRobotPose(getPose());
  }

  @Logged(name = "AtPoseSetpoint")
  public boolean atSetpoint() {
    return atPoseSetpoint();
  }
}
