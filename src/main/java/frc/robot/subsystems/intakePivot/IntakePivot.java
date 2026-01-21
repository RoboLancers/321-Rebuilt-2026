/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {

  private TalonFX intakePivotMotorLeft = new TalonFX(IntakeConstants.intakePivotMotorLeftId);
  private TalonFX intakePivotMotorRight = new TalonFX(IntakeConstants.intakePivotMotorRightId);
  private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
  private MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
  private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
  private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  private VoltageConfigs voltageConfigs = new VoltageConfigs();
  private Slot0Configs slot0Configs = new Slot0Configs();

  public IntakePivot() {
    motorConfigurations();
  }

  private void motorConfigurations() {
    motorConfigs.withInverted(InvertedValue.Clockwise_Positive);
    motorConfigs.withNeutralMode(NeutralModeValue.Brake);

    currentLimitsConfigs.withStatorCurrentLimitEnable(IntakeConstants.currentLimitEnable);
    currentLimitsConfigs.withStatorCurrentLimit(IntakeConstants.currentLimit);

    slot0Configs.withKG(IntakeConstants.kG);
    slot0Configs.withKD(IntakeConstants.kD);
    slot0Configs.withKP(IntakeConstants.kP);

    feedbackConfigs.withSensorToMechanismRatio(IntakeConstants.sensorToMechanismRatio);

    intakePivotMotorLeft.getConfigurator().apply(motorConfigs);
    intakePivotMotorLeft.getConfigurator().apply(currentLimitsConfigs);
    intakePivotMotorLeft.getConfigurator().apply(slot0Configs);
    intakePivotMotorLeft.getConfigurator().apply(feedbackConfigs);

    Follower follower =
        new Follower(IntakeConstants.intakePivotMotorLeftId, MotorAlignmentValue.Aligned);
    intakePivotMotorRight.setControl(follower);
  }

  public void goToAngle(Angle angle) {
    MotionMagicVoltage intakeVoltage = new MotionMagicVoltage(angle);
    intakePivotMotorLeft.setControl(intakeVoltage);
  }

  public Angle getAngle() {
    return Degrees.of(intakePivotMotorLeft.getPosition().getValueAsDouble());
  }
}
