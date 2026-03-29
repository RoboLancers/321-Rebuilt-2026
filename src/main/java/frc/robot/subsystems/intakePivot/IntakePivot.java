/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class IntakePivot extends SubsystemBase {

  @Logged private TalonFX intakePivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);
  @Logged private CANcoder intakeEncoder = new CANcoder(IntakeConstants.kEncoderID);

  private MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
  private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
  private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

  public double targetVoltage = 0;
  public double loggedAngle = 0;

  private Angle targetAngle = IntakeConstants.kStowedPosition;

  public IntakePivot() {
    motorConfigurations();
    setPID(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kG);
  }

  private void motorConfigurations() {
    motorConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
    motorConfigs.withNeutralMode(NeutralModeValue.Brake);

    currentLimitsConfigs.withStatorCurrentLimitEnable(IntakeConstants.kCurrentLimitEnable);
    currentLimitsConfigs.withStatorCurrentLimit(IntakeConstants.kCurrentLimit);

    feedbackConfigs.withSensorToMechanismRatio(IntakeConstants.kSensorToMechanismRatio);

    motionMagicConfigs.withMotionMagicCruiseVelocity(IntakeConstants.kMaxVelocity);

    intakePivotMotor.getConfigurator().apply(motorConfigs);
    intakePivotMotor.getConfigurator().apply(currentLimitsConfigs);
    intakePivotMotor.getConfigurator().apply(feedbackConfigs);
    intakePivotMotor.getConfigurator().apply(motionMagicConfigs);
  }

  public ProfiledPIDController pivotController =
      new ProfiledPIDController(0, 0, 0, IntakeConstants.kMaxPivotConstraints);
  public ArmFeedforward pivotFeedforward = new ArmFeedforward(0, 0, 0);

  public void setTargetAngle(Angle angle) {
    this.targetAngle = angle;
  }

  public void zero() {
    intakePivotMotor.setPosition(Degrees.of(0));
  }

  public void goToAngle(Angle angle) {
    loggedAngle = angle.in(Degrees);
    double volts =
        pivotController.calculate(getAngle().in(Radians), angle.in(Radians))
            + pivotFeedforward.calculate(angle.in(Radians), 0);
    targetVoltage = volts;
    intakePivotMotor.setVoltage(volts);
  }

  public void setVoltage(Voltage volts) {
    intakePivotMotor.setVoltage(volts.in(Volts));
  }

  @Logged(name = "intakePivotAngle")
  public Angle getAngle() {
    return intakeEncoder.getAbsolutePosition().refresh().getValue();
  }

  public void zeroEncoder() {
    intakePivotMotor.setPosition(intakeEncoder.getAbsolutePosition().getValue());
  }

  @Logged(name = "intakePivotAtTargetAngle")
  public boolean atTargetAngle() {
    return atAngle(targetAngle);
  }

  public boolean atAngle(Angle angle) {
    return Math.abs(getAngle().in(Degrees) - angle.in(Degrees))
        < RobotConstants.kAngleTolerance.in(Degrees);
  }

  public void setPID(double kP, double kI, double kD, double kG) {
    pivotController.setP(kP);
    pivotController.setI(kI);
    pivotController.setD(kD);

    pivotFeedforward.setKg(kG);
  }

  public void tune(
      double kP,
      double kI,
      double kD,
      double kG,
      double angle,
      double maxVelocity,
      double maxAcceleration) {
    setPID(kP, kI, kD, kG);
    setConstraints(new Constraints(maxVelocity, maxAcceleration));
    goToAngle(Degrees.of(angle));
  }

  public void setConstraints(Constraints constraints) {
    pivotController.setConstraints(constraints);
  }

  @Logged(name = "intakePivotVOltage")
  public Voltage getVoltage() {
    return intakePivotMotor.getMotorVoltage().getValue();
  }

  @Logged(name = "intakePivotTargetAngle")
  public Angle getTargetAngle() {
    return targetAngle;
  }

  @Logged(name = "intakePivotVelocity")
  public double getVelocity() {
    return intakePivotMotor.getVelocity().getValueAsDouble();
  }

  @Logged(name = "intakePivotCurrent")
  public Current getCurrent() {
    return intakePivotMotor.getStatorCurrent().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Pivot Angle", getAngle().in(Degrees));
    SmartDashboard.putNumber("Intake Pivot Voltage", getVoltage().in(Volts));
    SmartDashboard.putNumber("Intake Pivot Current", getCurrent().in(Amps));
    SmartDashboard.putNumber("Pivot Target Voltage", targetVoltage);
    SmartDashboard.putNumber("Pivot Target Angle", loggedAngle);
  }
}
