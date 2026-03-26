/* (C) RoboLancers 2026 */
package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  @Logged private TalonFX motor = new TalonFX(IndexerConstants.kMotorID);
  private AngularVelocity targetVelocity = DegreesPerSecond.of(0);

  private PIDController indexerController = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward indexerFeedforward = new SimpleMotorFeedforward(0.50, 0.00795);

  public Indexer() {
    configureMotors();
    setPID(IndexerConstants.kP, IndexerConstants.kD, IndexerConstants.kV, IndexerConstants.kS);
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
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(IndexerConstants.kGearing))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(IndexerConstants.kMaxVelocity)
                    .withMotionMagicAcceleration(IndexerConstants.kMaxAcceleration));

    motor.getConfigurator().apply(configurations);
  }

  public void setPID(double kP, double kD, double kV, double kS) {
    // Slot0Configs pidConfigs = new Slot0Configs().withKP(kP).withKD(kD).withKV(kV);

    // motor.getConfigurator().apply(pidConfigs);

    indexerController.setP(kP);

    indexerController.setD(kD);

    indexerFeedforward.setKv(kV);

    indexerFeedforward.setKs(kS);
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    this.targetVelocity = velocity;
  }

  public AngularVelocity getOscillationVelocity() {
    double velocity =
        IndexerConstants.kIndexVelocity.in(RPM)
            + IndexerConstants.kOscillationAmplitude
                * Math.sin(2 * Math.PI * Timer.getFPGATimestamp());
    return RPM.of(velocity);
  }

  public void goToVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
    double volts =
        indexerController.calculate(getVelocity().in(RPM), targetVelocity.in(RPM))
            + indexerFeedforward.calculateWithVelocities(
                getVelocity().in(RPM), targetVelocity.in(RPM));

    motor.setVoltage(volts);
  }

  public void setVoltage(Voltage volts) {
    motor.setVoltage(volts.in(Volts));
  }

  public void tune(double kP, double kD, double kV, double kS, AngularVelocity targetSpeed) {
    setPID(kP, kD, kV, kS);
    goToVelocity(targetSpeed);
  }

  @Logged(name = "indexerTargetVelocity")
  public AngularVelocity getTargetVelocity() {
    return this.targetVelocity;
  }

  @Logged(name = "indexerVelocity")
  public AngularVelocity getVelocity() {
    return motor.getVelocity().getValue();
  }

  @Logged(name = "indexerVoltage")
  public Voltage getVoltage() {
    return motor.getMotorVoltage().getValue();
  }

  @Logged(name = "indexerCurrent")
  public Current getIndexerCurrent() {
    return motor.getStatorCurrent().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Indexer Velocity", motor.getVelocity().getValue().in(RPM));
    SmartDashboard.putNumber("Indexer Voltage", motor.getMotorVoltage().getValue().in(Volts));
  }
}
