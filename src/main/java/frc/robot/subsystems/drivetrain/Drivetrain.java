/* (C) RoboLancers 2026 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotConstants;
import frc.robot.subsystems.vision.VisionEstimate;
import frc.robot.util.AprilTagUtil;
import frc.robot.util.DefenseMode;
import frc.robot.util.MyAlliance;
import frc.robot.util.RebuiltUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
  ChassisSpeeds speeds = new ChassisSpeeds();

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

  @Logged(name = "drivetrainPoseField")
  public Field2d poseField = new Field2d();

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

    // SmartDashboard.putData("Drivetrain Pose Field", poseField);

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds. Also optionally outputs individual module feedforwards
        ppHolonomicDriveController,
        config, // The robot configuration
        MyAlliance::isRed, // Red alliance means flip the direction
        this // Reference to this subsystem to set requirements
        );
  }

  void configurePoseControllers() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  boolean atPoseSetpoint(Supplier<Pose2d> currentRobotPose) {
    return atPoseSetpoint(
        DrivetrainConstants.kAlignmentSetpointTranslationTolerance,
        DrivetrainConstants.kAlignmentSetpointRotationTolerance,
        currentRobotPose);
  }

  boolean atFinalPoseSetpoint(Supplier<Pose2d> currentRobotPose) {
    if (!getAlignmentSetpoint().isFinalSetpoint()) return false;
    return atPoseSetpoint(currentRobotPose);
  }

  public Pose2d getNearestAllianceAprilTag() {
    return MyAlliance.isRed()
        ? getPose().nearest(RebuiltUtil.redTagPoses)
        : getPose().nearest(RebuiltUtil.blueTagPoses);
  }

  public Pose2d getNearestAprilTag() {
    return getPose().nearest(AprilTagUtil.getAllAprilTagPoses());
  }

  public List<Double> getDriveVelocities() {
    List<Double> velocityList = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      velocityList.add(
          i,
          Arrays.asList(super.getModules())
              .get(i)
              .getDriveMotor()
              .getVelocity()
              .getValue()
              .in(RPM));
    }
    return velocityList;
  }

  public List<Double> getSteerPositions() {
    List<Double> positionList = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      positionList.add(
          i,
          Arrays.asList(super.getModules())
              .get(i)
              .getEncoder()
              .getAbsolutePosition()
              .getValue()
              .in(Degrees));
    }
    return positionList;
  }

  public void logSwerveMotorStates() {
    for (double velocity : getDriveVelocities()) {
      SmartDashboard.putNumber(
          "Measured Velocity" + Integer.toString(getDriveVelocities().indexOf(velocity)), velocity);
    }
    for (double position : getSteerPositions()) {
      SmartDashboard.putNumber(
          "Measured Position" + Integer.toString(getSteerPositions().indexOf(position)), position);
    }
  }

  // drive with heading controlled by PID
  public Command driveFixedHeading(
      DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Rotation2d> rotation) {
    return run(
        () ->
            driveFixedHeading(
                translationX.getAsDouble(), translationY.getAsDouble(), rotation.get()));
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

  public Command driveToFieldPoseCommand(Supplier<Pose2d> pose, Supplier<Pose2d> currentRobotPose) {
    return driveToFieldPose(() -> (new AlignmentSetpoint(pose.get(), true)), currentRobotPose);
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

  public Command teleopDriveWithHeading(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      Supplier<Rotation2d> heading,
      Supplier<Pose2d> currentRobotPose) {
    return runOnce(
            () -> {
              ChassisSpeeds speeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      getChassisSpeeds(), currentRobotPose.get().getRotation());

              thetaController.reset(
                  currentRobotPose.get().getRotation().getRadians(), speeds.omegaRadiansPerSecond);
            })
        .andThen(
            run(
                () -> {
                  var speeds =
                      ChassisSpeeds.discretize(
                          translationX.getAsDouble(),
                          translationY.getAsDouble(),
                          thetaController.calculate(
                              currentRobotPose.get().getRotation().getRadians(),
                              heading.get().getRadians()),
                          RobotConstants.kRobotLoopPeriod.in(Seconds));

                  setControl(
                      fieldCentricRequest
                          .withVelocityX(speeds.vxMetersPerSecond)
                          .withVelocityY(speeds.vyMetersPerSecond)
                          .withRotationalRate(speeds.omegaRadiansPerSecond)
                          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
                }));
  }

  public Command teleopDriveFixedHeading(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY) {
    return run(
        () -> {
          Rotation2d desiredRotation =
              Rotation2d.fromRadians(Math.atan2(rotationX.getAsDouble(), rotationY.getAsDouble()));

          if (MyAlliance.isRed()) {
            desiredRotation = desiredRotation.plus(Rotation2d.k180deg);
          }

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

  public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {

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
            xPoseController.calculate(currentPose.getX(), pose.getX())
                + xPoseController.getSetpoint().velocity * ffFactor,
            yPoseController.calculate(currentPose.getY(), pose.getY())
                + yPoseController.getSetpoint().velocity * ffFactor,
            thetaController.calculate(
                    currentPose.getRotation().getRadians(), pose.getRotation().getRadians())
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

  @Logged(name = "Is defense line")
  public boolean isDefenseLine() {
    return DefenseMode.isDefenseLine(getPose());
  }

  double clampedVelocity = 0;

  @Logged(name = "Clamped velocity")
  public double clampedVelocity() {
    return clampedVelocity;
  }

  @Logged(name = "Is red alliance line")
  public boolean isRedAllianceLine() {
    return DefenseMode.isRedAllianceLine(getPose());
  }

  @Logged(name = "Is blue alliance line")
  public boolean isBlueAllianceLine() {
    return DefenseMode.isBlueAllianceLine(getPose());
  }

  @Logged(name = "Is red neutral line")
  public boolean isRedNeutralLine() {
    return DefenseMode.isRedNeutralLine(getPose());
  }

  @Logged(name = "Is blue neutral line")
  public boolean isBlueNeutralLine() {
    return DefenseMode.isBlueNeutralLine(getPose());
  }

  @Logged(name = "Alliance Based Line Type")
  public String AllianceBasedLineType() {
    return DefenseMode.getAllianceBasedLine(DefenseMode.getDefenseLine(getPose())).toString();
  }

  public Command defenseDrive(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotation,
      BooleanSupplier defenseMode) {
    return run(
        () -> {
          if (DefenseMode.isDefenseLine(getPose()) && defenseMode.getAsBoolean()) {
            driveFixedHeading(
                DefenseMode.defenseClampVelocity(translationX.getAsDouble(), getPose()),
                translationY.getAsDouble(),
                Rotation2d.kZero);
            clampedVelocity =
                DefenseMode.defenseClampVelocity(translationX.getAsDouble(), getPose());
          } else {
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
          }
        });
  }

  // public Command defenseDrive(
  //     DoubleSupplier translationX,
  //     DoubleSupplier translationY,
  //     DoubleSupplier rotation,
  //     BooleanSupplier defenseMode) {
  //   return run(
  //       () -> {
  //           var speeds =
  //               ChassisSpeeds.discretize(
  //                   translationX.getAsDouble(),
  //                   translationY.getAsDouble(),
  //                   rotation.getAsDouble(),
  //                   RobotConstants.kRobotLoopPeriod.in(Seconds));

  //           setControl(
  //               fieldCentricRequest
  //                   .withVelocityX(speeds.vxMetersPerSecond)
  //                   .withVelocityY(speeds.vyMetersPerSecond)
  //                   .withRotationalRate(speeds.omegaRadiansPerSecond)
  //                   .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));}
  //   );
  //       }

  public boolean atPoseSetpoint(Distance tranTol, Angle rotTol, Supplier<Pose2d> currentPose) {
    return currentPose.get().getTranslation().getDistance(alignmentSetpoint.pose().getTranslation())
            < tranTol.in(Meters)
        && Math.abs(
                currentPose
                    .get()
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

  public boolean shooterAtHeading(Rotation2d heading) {
    return Math.abs(
            Math.floorMod(
                    Math.round(
                        getPose().getRotation().getDegrees()
                            - RobotConstants.kShooterFaceOffset.in(Degrees)),
                    360)
                - Math.floorMod(Math.round(heading.getDegrees()), 360))
        <= DrivetrainConstants.kSpinupRange.in(Degrees);
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

  // PPHolonomicController is the built in controller for holonomic drive trains
  private PPHolonomicDriveController ppHolonomicDriveController =
      new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
          );

  private boolean hasAppliedOperatorPerspective;

  public void addVisionMeasurement(VisionEstimate estimate) {
    addVisionMeasurement(
        estimate.estimatedPose().estimatedPose.toPose2d(),
        estimate.estimatedPose().timestampSeconds,
        VecBuilder.fill(
            estimate.standardDeviations(),
            estimate.standardDeviations(),
            estimate.standardDeviations()));
  }

  public void driveTrainPeriodic() {

    SmartDashboard.putNumber("Drivetrain Pose X", getPose().getX());

    SmartDashboard.putNumber("Drivetrain Pose Y", getPose().getY());

    SmartDashboard.putNumber("Drivetrain Pose Yaw", getPose().getRotation().getDegrees());

    logSwerveMotorStates();
    poseField.setRobotPose(getPose());
    SmartDashboard.putData("Robot Pose Field", poseField);

    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    MyAlliance.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg);
              });
      hasAppliedOperatorPerspective = true;
    }
  }
}
