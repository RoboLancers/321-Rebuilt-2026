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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  @Logged private TalonFX shooterMotor = new TalonFX(OuttakeConstants.kMotorID);

  @Logged private DigitalInput shooterBeamBreak = new DigitalInput(OuttakeConstants.kBeamBreakID);

  private AngularVelocity targetShooterVelocity = RPM.of(0);

  public Shooter() {

    configureMotors();
    setPID(OuttakeConstants.kP, OuttakeConstants.kD, OuttakeConstants.kV);
  }

  private void configureMotors() {

    TalonFXConfiguration configuration =
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

    shooterMotor.getConfigurator().apply(configuration);
  }

  private void setPID(double kP, double kD, double kV) {

    Slot0Configs pid = new Slot0Configs().withKP(kP).withKD(kD).withKV(kV);

    shooterMotor.getConfigurator().apply(pid);
  }

  public void setVelocity(AngularVelocity rpm) {
    targetShooterVelocity = rpm;
    shooterMotor.setControl(new MotionMagicVelocityVoltage(rpm.in(RPM)));
  }

  public void tune(double kP, double kD, double kV, double targetRPM) {
    setPID(kP, kD, kV);
    shooterMotor.setControl(new MotionMagicVelocityVoltage(RPM.of(targetRPM)));
  }

  @Logged(name = "shooterTargetVelocity")
  public AngularVelocity getTargetShooterVelocity() {
    return this.targetShooterVelocity;
  }

  @Logged(name = "shooterVelocity")
  public AngularVelocity getVelocity() {
    return shooterMotor.getVelocity().getValue();
  }

  @Logged(name = "shooterVoltage")
  public Voltage getVoltage() {
    return shooterMotor.getMotorVoltage().getValue();
  }

  @Logged(name = "shooterCurrent")
  public Current getCurrent() {
    return shooterMotor.getStatorCurrent().getValue();
  }

  @Logged(name = "shooterHasFuel")
  public boolean getFuelInShooter(){
    return shooterBeamBreak.get();
  }
}
