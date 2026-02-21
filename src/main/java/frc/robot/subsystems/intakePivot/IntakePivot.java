/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(name = "Intake Pivot")
public class IntakePivot extends SubsystemBase {

  @Logged private TalonFX intakePivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);
  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(IntakeConstants.kEncoderID);

  private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
  private MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
  private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
  private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  private VoltageConfigs voltageConfigs = new VoltageConfigs();
  private Slot0Configs slot0Configs = new Slot0Configs();

  private Angle targetAngle = IntakeConstants.kDefaultPosition;
  private Voltage targetVoltage = Volts.of(0);
  private AngularVelocity targetVelocity = DegreesPerSecond.of(0);

  public IntakePivot() {
    motorConfigurations();
    setPID(IntakeConstants.kP, IntakeConstants.kD, IntakeConstants.kG);
  }

  private void motorConfigurations() {
    motorConfigs.withInverted(InvertedValue.Clockwise_Positive);
    motorConfigs.withNeutralMode(NeutralModeValue.Brake);

    currentLimitsConfigs.withStatorCurrentLimitEnable(IntakeConstants.kCurrentLimitEnable);
    currentLimitsConfigs.withStatorCurrentLimit(IntakeConstants.kCurrentLimit);

    slot0Configs.withGravityType(GravityTypeValue.Arm_Cosine);
    slot0Configs.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    feedbackConfigs.withSensorToMechanismRatio(IntakeConstants.kSensorToMechanismRatio);

    intakePivotMotor.getConfigurator().apply(motorConfigs);
    intakePivotMotor.getConfigurator().apply(currentLimitsConfigs);
    intakePivotMotor.getConfigurator().apply(slot0Configs);
    intakePivotMotor.getConfigurator().apply(feedbackConfigs);
  }

  public void goToAngle(Angle angle) {
    targetAngle = angle;
    MotionMagicVoltage intakeVoltage = new MotionMagicVoltage(angle);
    intakePivotMotor.setControl(intakeVoltage);
  }

  public Angle getAngle() {
    return Degrees.of(intakePivotMotor.getPosition().getValueAsDouble());
  }

  public void zeroEncoder() {
    intakePivotMotor.setPosition(Degrees.of(absoluteEncoder.get()));
  }

  @Logged(name = "atTargetAngle")
  public boolean atTargetAngle() {
    return getAngle() == targetAngle;
  }

  public void setPID(double kP, double kD, double kG) {
    intakePivotMotor.getConfigurator().apply(new Slot0Configs().withKP(kP).withKD(kD).withKG(kG));
  }

  public void tune(double kP, double kD, double kG, double angle) {
    setPID(kP, kD, kG);
    goToAngle(Degrees.of(angle));
  }

  public Voltage getVoltage() {
    return intakePivotMotor.getMotorVoltage().getValue();
  }

  public void setVoltage(Voltage targetVoltage) {
    this.targetVoltage = targetVoltage;
    intakePivotMotor.setVoltage(targetVoltage.in(Volts));
  }

  public double getVelocity() {
    return intakePivotMotor.getVelocity().getValueAsDouble();
  }

  public Current getCurrent() {
    return intakePivotMotor.getStatorCurrent().getValue();
  }

  @Logged(name = "AtTargetVelocity")
  public boolean atTargetVelocity() {
    return intakePivotMotor.getVelocity().getValue() == targetVelocity;
  }
}
