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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;

public class Indexer extends SubsystemBase {

  private TalonFX motor = new TalonFX(IndexerConstants.kMotorID);

  public double kP = 0;
  public double kD = 0;
  public double kV = 0;

  public Indexer() {
    configureMotors();
    setPID();
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
                    .withNeutralMode(IndexerConstants.kNeutralMode)
                    .withInverted(IndexerConstants.kInverted))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(IndexerConstants.kGearing))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(IndexerConstants.kMaxVelocity)
                    .withMotionMagicAcceleration(IndexerConstants.kMaxAcceleration));

    motor.getConfigurator().apply(configurations);
  }

  public void setPID() {
    Slot0Configs pidConfigs = new Slot0Configs().withKP(kP).withKD(kD).withKV(kV);

    motor.getConfigurator().apply(pidConfigs);
  }

  public void goToVelocity(AngularVelocity velocity) {
    motor.setControl(new MotionMagicVelocityVoltage(velocity));
  }

  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage.in(Volts));
  }

  public void tune() {
    TunableConstant kP = new TunableConstant("kP/Indexer/", 0);

    TunableConstant kD = new TunableConstant("kD/Indexer/", 0);

    TunableConstant kV = new TunableConstant("kV/Indexer/", 0);

    TunableConstant targetSpeed = new TunableConstant("targetSpeed/Indexer/", 0);

    this.kP = kP.get();
    this.kV = kV.get();
    this.kD = kD.get();

    goToVelocity(RPM.of(targetSpeed.get()));
  }
}
