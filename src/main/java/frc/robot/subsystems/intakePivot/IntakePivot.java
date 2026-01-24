/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;

@Logged
public class IntakePivot extends SubsystemBase {

  private TalonFX intakePivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);
  private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
  private MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
  private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
  private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  private VoltageConfigs voltageConfigs = new VoltageConfigs();
  private Slot0Configs slot0Configs = new Slot0Configs();

  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(IntakeConstants.kEncoderID);

  public IntakePivot() {
    motorConfigurations();
  }

  private void motorConfigurations() {
    motorConfigs.withInverted(InvertedValue.Clockwise_Positive);
    motorConfigs.withNeutralMode(NeutralModeValue.Brake);

    currentLimitsConfigs.withStatorCurrentLimitEnable(IntakeConstants.kCurrentLimitEnable);
    currentLimitsConfigs.withStatorCurrentLimit(IntakeConstants.kCurrentLimit);

    slot0Configs.withKG(IntakeConstants.kG);
    slot0Configs.withKD(IntakeConstants.kD);
    slot0Configs.withKP(IntakeConstants.kP);

    feedbackConfigs.withSensorToMechanismRatio(IntakeConstants.kSensorToMechanismRatio);

    intakePivotMotor.getConfigurator().apply(motorConfigs);
    intakePivotMotor.getConfigurator().apply(currentLimitsConfigs);
    intakePivotMotor.getConfigurator().apply(slot0Configs);
    intakePivotMotor.getConfigurator().apply(feedbackConfigs);
  }

  public void goToAngle(Angle angle) {
    MotionMagicVoltage intakeVoltage = new MotionMagicVoltage(angle);
    intakePivotMotor.setControl(intakeVoltage);
  }

  public Angle getAngle() {
    return Degrees.of(intakePivotMotor.getPosition().getValueAsDouble());
  }

  public void zeroEncoder() {
    intakePivotMotor.setPosition(Degrees.of(absoluteEncoder.get()));
  }

  public void tune() {

    TunableConstant kP = new TunableConstant("/IntakePivot/kP", 0);
    TunableConstant kD = new TunableConstant("/IntakePivot/kD", 0);
    TunableConstant kG = new TunableConstant("/IntakePivot/kG", 0);
    TunableConstant angle = new TunableConstant("/IntakePivot/angle", 0);

    IntakeConstants.kP = kP.get();
    IntakeConstants.kD = kD.get();
    IntakeConstants.kG = kG.get();

    goToAngle(Degrees.of(angle.get()));
  }
}
