/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {

  @Logged private TalonFX rollerMotor = new TalonFX(IntakeRollerConstants.kRollerMotorId);
  private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  private MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
  private VoltageConfigs voltageConfigs = new VoltageConfigs();
  private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
  private Slot0Configs slot0Configs = new Slot0Configs();

  private AngularVelocity targetVelocity = RPM.of(0);

  public IntakeRollers() {
    motorConfigurations();
    setPID(
        IntakeRollerConstants.kP,
        IntakeRollerConstants.kD,
        IntakeRollerConstants.kV,
        IntakeRollerConstants.kG);
  }

  public void motorConfigurations() {
    motorConfigs.withInverted(InvertedValue.Clockwise_Positive);
    motorConfigs.withNeutralMode(NeutralModeValue.Brake);
    feedbackConfigs.withSensorToMechanismRatio(IntakeRollerConstants.kSensorToMechanismRatio);

    currentLimitsConfigs.withStatorCurrentLimitEnable(
        IntakeRollerConstants.kStatorCurrentLimitsEnable);
    currentLimitsConfigs.withStatorCurrentLimit(IntakeRollerConstants.kStatorCurrentLimit);
    currentLimitsConfigs.withSupplyCurrentLimitEnable(
        IntakeRollerConstants.kSupplyCurrentLimitsEnable);
    currentLimitsConfigs.withSupplyCurrentLimit(IntakeRollerConstants.kSupplyCurrentLimit);

    rollerMotor.getConfigurator().apply(motorConfigs);
    rollerMotor.getConfigurator().apply(currentLimitsConfigs);
    rollerMotor.getConfigurator().apply(feedbackConfigs);
  }

  public void setVelocity(AngularVelocity velocity) {
    this.targetVelocity = velocity;
    rollerMotor.setControl(new MotionMagicVelocityVoltage(velocity));
  }

  public void setPID(double kP, double kD, double kV, double kG) {
    slot0Configs.withKP(kP);
    slot0Configs.withKD(kD);
    slot0Configs.withKG(kG);
    slot0Configs.withKV(kV);

    rollerMotor.getConfigurator().apply(slot0Configs);
  }

  public void tune(
      double kP, double kD, double kV, double kG, AngularVelocity rollerTargetVelocity) {
    setPID(kP, kD, kV, kG);
    setVelocity(rollerTargetVelocity);
  }

  @Logged(name = "intakeRollersVelocity")
  public AngularVelocity getRollerVelocity() {
    return rollerMotor.getVelocity().getValue();
  }

  @Logged(name = "intakeRollersAtTargetVelocity")
  public boolean atTargetVelocity() {
    return rollerMotor.getVelocity().getValue() == targetVelocity;
  }

  @Logged(name = "intakeRollersTargetVelocity")
  public AngularVelocity getRollerTargetVelocity() {
    return targetVelocity;
  }

  @Logged(name = "intakeRollersVoltage")
  public Voltage getVoltage() {
    return rollerMotor.getMotorVoltage().getValue();
  }

  @Logged(name = "intakeRollersCurrent")
  public Current getCurrent() {
    return rollerMotor.getStatorCurrent().getValue();
  }
}
