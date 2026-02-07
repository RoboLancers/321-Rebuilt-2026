/* (C) RoboLancers 2026 */
package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Indexer extends SubsystemBase {

  private TalonFX motor = new TalonFX(IndexerConstants.kMotorID);
  private Velocity targetVelocity;
  private Voltage targetVoltage;
  private Current targetCurrent;

  public Indexer() {
    configureMotors();
    setPID(IndexerConstants.kP, IndexerConstants.kD, IndexerConstants.kV);
  }

  public void configureMotors() {
    TalonFXConfiguration configurations =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(IndexerConstants.kCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(IndexerConstants.kCurrentLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        IndexerConstants.kInverted
                            ? InvertedValue.CounterClockwise_Positive
                            : InvertedValue.Clockwise_Positive))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(IndexerConstants.kGearing))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(IndexerConstants.kMaxVelocity)
                    .withMotionMagicAcceleration(IndexerConstants.kMaxAcceleration));

    motor.getConfigurator().apply(configurations);
  }

  public void setPID(double kP, double kD, double kV) {
    Slot0Configs pidConfigs = new Slot0Configs().withKP(kP).withKD(kD).withKV(kV);

    motor.getConfigurator().apply(pidConfigs);
  }

  public void goToVelocity(AngularVelocity velocity) {
    motor.setControl(new MotionMagicVelocityVoltage(velocity));
  }

  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage.in(Volts));
  }

  public void tune(double kP, double kD, double kV, double targetSpeed) {
    setPID(kP, kD, kV);
    goToVelocity(RPM.of(targetSpeed));
  }

  @Logged
  public Velocity getTargetVelocity() {
    return this.targetVelocity;
  }

  @Logged
  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Logged
  public Voltage getTargetVoltage() {
    return this.targetVoltage;
  }

  @Logged
  public Voltage getVoltage() {
    return motor.getMotorVoltage().getValue();
  }

  @Logged
  public Current getIndexerCurrent() {
    return motor.getStatorCurrent().getValue();
  }

  @Logged
  public Current getTargetCurrent() {
    return this.targetCurrent;
  }
}
