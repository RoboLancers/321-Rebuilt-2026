/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeRollers {

  private TalonFX rollerMotor = new TalonFX(IntakeRollersConstants.rollerMotorId);
  private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  private MotorOutputConfigs motorConfigs = new MotorOutputConfigs();

  public IntakeRollers() {
    motorConfigurations();
  }

  public void motorConfigurations() {
    motorConfigs.withInverted(InvertedValue.Clockwise_Positive);
    motorConfigs.withNeutralMode(NeutralModeValue.Brake);

    currentLimitsConfigs.withStatorCurrentLimitEnable(IntakeRollersConstants.kCurrentLimitsEnable);
    currentLimitsConfigs.withStatorCurrentLimit(IntakeRollersConstants.currentLimit);
  }
}
