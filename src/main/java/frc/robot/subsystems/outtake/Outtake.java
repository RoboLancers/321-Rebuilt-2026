/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake;

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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;

public class Outtake extends SubsystemBase {

  private TalonFX motor = new TalonFX(OuttakeConstants.kMotorID);

  public double kP = 0;

  public double kD = 0;

  public double kV = 0;

  public Outtake() {

    configureMotors();
    setPID();
  }

  private void configureMotors() {

    TalonFXConfiguration configuration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(OuttakeConstants.kStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(OuttakeConstants.kStatorLimit)
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

    motor.getConfigurator().apply(configuration);
  }

  private void setPID() {

    Slot0Configs pid = new Slot0Configs().withKP(kP).withKD(kD).withKV(kV);

    motor.getConfigurator().apply(pid);
  }

  public Command setControl(AngularVelocity rpm) {
    return run(() -> motor.setControl(new MotionMagicVelocityVoltage(rpm.in(RPM))));
  }

  public Command runVolts(Voltage volts) {
    return run(() -> motor.setVoltage(volts.in(Volts)));
  }

  public Command tune() {

    TunableConstant kP = new TunableConstant("/Outtake/kP", 0);
    TunableConstant kD = new TunableConstant("/Outtake/kD", 0);
    TunableConstant kV = new TunableConstant("/Outtake/kV", 0);
    TunableConstant targetRPM = new TunableConstant("/Outtake/targetRPM", 0);

    this.kP = kP.get();
    this.kD = kD.get();
    this.kV = kV.get();

    return run(() -> setControl(RPM.of(targetRPM.get())));
  }
}
