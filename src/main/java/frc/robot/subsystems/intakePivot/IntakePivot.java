/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class IntakePivot extends SubsystemBase {

  @Logged private TalonFX intakePivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);
  @Logged private DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(IntakeConstants.kEncoderID);

  private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
  private MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
  private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
  private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  private VoltageConfigs voltageConfigs = new VoltageConfigs();
  private Slot0Configs slot0Configs = new Slot0Configs();
  private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

  private Angle targetAngle = IntakeConstants.kDefaultPosition;

  public IntakePivot() {
    motorConfigurations();
    setPID(IntakeConstants.kP, IntakeConstants.kD, IntakeConstants.kG);
  }

  private void motorConfigurations() {
  //   motorConfigs.withInverted(InvertedValue.Clockwise_Positive);
  //   motorConfigs.withNeutralMode(NeutralModeValue.Brake);

  //   currentLimitsConfigs.withStatorCurrentLimitEnable(IntakeConstants.kCurrentLimitEnable);
  //   currentLimitsConfigs.withStatorCurrentLimit(IntakeConstants.kCurrentLimit);


  //   feedbackConfigs.withSensorToMechanismRatio(IntakeConstants.kSensorToMechanismRatio);

  //   // motionMagicConfigs.withMotionMagicCruiseVelocity(IntakeConstants.kMaxVelocity);

  //   intakePivotMotor.getConfigurator().apply(motorConfigs);
  //   intakePivotMotor.getConfigurator().apply(currentLimitsConfigs);
  //   intakePivotMotor.getConfigurator().apply(feedbackConfigs);
  //   intakePivotMotor.getConfigurator().apply(motionMagicConfigs);
  }

  public PIDController pivotController = new PIDController(0,0,0);
  public ArmFeedforward pivotFeedforward = new ArmFeedforward(0,0,0);

  public void goToAngle(Angle angle) {
    this.targetAngle = angle;
   double volts = pivotController.calculate(getAngle().in(Degrees), angle.in(Degrees)) + pivotFeedforward.calculate(angle.in(Degrees), 0);
    intakePivotMotor.setVoltage(volts);
  }

  public void setVoltage(Voltage volts) {
    intakePivotMotor.setVoltage(volts.in(Volts));
  }

  @Logged(name = "intakePivotAngle")
  public Angle getAngle() {
    return intakePivotMotor.getPosition().getValue();
  }

  public void zeroEncoder() {
    intakePivotMotor.setPosition(Degrees.of(intakeEncoder.get()));
  }

  @Logged(name = "intakePivotAtTargetAngle")
  public boolean atTargetAngle() {
    return atAngle(targetAngle);
  }

  public boolean atAngle(Angle angle) {
    return Math.abs(getAngle().in(Degrees) - angle.in(Degrees))
        < RobotConstants.kAngleTolerance.in(Degrees);
  }

  public void setPID(double kP, double kD, double kG) {
    pivotController.setP(kP);
    pivotController.setD(kD);
    pivotFeedforward.setKg(kG);
  }

  public void tune(double kP, double kD, double kG, double angle) {
    setPID(kP, kD, kG);
    goToAngle(Degrees.of(angle));
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

  public boolean atHomedPosition() {
    return false;
  }

  public void periodic() {

    SmartDashboard.putNumber("Intake Pivot Angle", getAngle().in(Degrees));
    SmartDashboard.putNumber("Intake Pivot Voltage", getVoltage().in(Volts));
    SmartDashboard.putNumber("Intake Pivot Current", getCurrent().in(Amps));
  }
}
