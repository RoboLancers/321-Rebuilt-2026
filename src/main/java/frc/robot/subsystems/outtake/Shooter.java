/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  @Logged private TalonFX topShooterMotor = new TalonFX(OuttakeConstants.kTopMotorID);
  @Logged private TalonFX bottomShooterMotor = new TalonFX(OuttakeConstants.kBottomMotorID);

  private AngularVelocity targetShooterVelocity = RPM.of(0);

  public Shooter() {

    configureMotors();
    setPID(OuttakeConstants.kP, OuttakeConstants.kD, OuttakeConstants.kV);
  }

  private void configureMotors() {

    TalonFXConfiguration topConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(OuttakeConstants.kStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(OuttakeConstants.kSupplyLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        OuttakeConstants.kInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(OuttakeConstants.kGearing)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(OuttakeConstants.kMaxVelocity)
                    .withMotionMagicAcceleration(OuttakeConstants.kMaxAcceleration));

    TalonFXConfiguration bottomConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(OuttakeConstants.kStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(OuttakeConstants.kSupplyLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        OuttakeConstants.kInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(OuttakeConstants.kGearing)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(OuttakeConstants.kMaxVelocity)
                    .withMotionMagicAcceleration(OuttakeConstants.kMaxAcceleration));

    topShooterMotor.getConfigurator().apply(topConfiguration);
    bottomShooterMotor.getConfigurator().apply(bottomConfiguration);
  }

  private void setPID(double kP, double kD, double kV) {

    Slot0Configs pid = new Slot0Configs().withKP(kP).withKD(kD).withKV(kV);

    topShooterMotor.getConfigurator().apply(pid);
    bottomShooterMotor.getConfigurator().apply(pid);
  }

  public void setVelocity(AngularVelocity rpm) {
    targetShooterVelocity = rpm;
    topShooterMotor.setControl(new MotionMagicVelocityVoltage(rpm.in(RPM)));
    bottomShooterMotor.setControl(new Follower())
  }

  public void tune(double kP, double kD, double kV, double targetRPM) {
    setPID(kP, kD, kV);
    topShooterMotor.setControl(new MotionMagicVelocityVoltage(RPM.of(targetRPM)));
  }

  @Logged(name = "shooterTargetVelocity")
  public AngularVelocity getTargetShooterVelocity() {
    return this.targetShooterVelocity;
  }

  @Logged(name = "shooterTopMotorVelocity")
  public AngularVelocity getTopVelocity() {
    return topShooterMotor.getVelocity().getValue();
  }

  @Logged(name = "shooterBottomMotorVelocity")
  public AngularVelocity getBottomVelocity() {
    return bottomShooterMotor.getVelocity().getValue();
  }

  @Logged(name = "topShooterVoltage")
  public Voltage getTopVoltage() {
    return topShooterMotor.getMotorVoltage().getValue();
  }

  @Logged(name = "bottomShooterVoltage")
  public Voltage getBottomVoltage() {
    return bottomShooterMotor.getMotorVoltage().getValue();
  }

  @Logged(name = "topShooterCurrent")
  public Current getTopCurrent() {
    return topShooterMotor.getStatorCurrent().getValue();
  }

  @Logged(name = "bottomShooterCurrent")
  public Current getCurrent() {
    return bottomShooterMotor.getStatorCurrent().getValue();
  }
}
