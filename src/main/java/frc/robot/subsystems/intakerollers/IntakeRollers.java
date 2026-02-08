/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class IntakeRollers extends SubsystemBase {

  @Logged private TalonFX rollerMotor = new TalonFX(IntakeRollerConstants.kRollerMotorId);
  private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  private MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
  private VoltageConfigs voltageConfigs = new VoltageConfigs();
  private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
  private Slot0Configs slot0Configs = new Slot0Configs();

  private Velocity targetVelocity;
  private Voltage targetVoltage;

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

    currentLimitsConfigs.withStatorCurrentLimitEnable(IntakeRollerConstants.kCurrentLimitsEnable);
    currentLimitsConfigs.withStatorCurrentLimit(IntakeRollerConstants.kCurrentLimit);

    rollerMotor.getConfigurator().apply(motorConfigs);
    rollerMotor.getConfigurator().apply(currentLimitsConfigs);
    rollerMotor.getConfigurator().apply(feedbackConfigs);
  }

  public void setVoltage(Voltage targetVoltage) {
    rollerMotor.setVoltage(targetVoltage.in(Volts));
  }

  public void setVelocity(double targetVelocity) {
    rollerMotor.setControl(new MotionMagicVelocityVoltage(targetVelocity));
  }

  public void setPID(double kP, double kD, double kV, double kG) {
    slot0Configs.withKP(kP);
    slot0Configs.withKD(kD);
    slot0Configs.withKG(kG);
    slot0Configs.withKV(kV);

    rollerMotor.getConfigurator().apply(slot0Configs);
  }

  public void tune(double kP, double kD, double kV, double kG, double rollerTargetVelocity) {
    setPID(kP, kD, kV, kG);
    setVelocity(rollerTargetVelocity);
  }

  @Logged(name = "rollerVelocity")
  public double getRollerVelocity() {
    return rollerMotor.getVelocity().getValueAsDouble();
  }

  @Logged(name = "atTargetRollerVelocity")
  public boolean atTargetVoltage() {
    return rollerMotor.getVelocity().getValue() == targetVelocity;
  }
}
