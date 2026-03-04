/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tunnel extends SubsystemBase {

  private PIDController tunnelController = new PIDController(TunnelConstants.kP, 0, 0);

  private SimpleMotorFeedforward tunnelFeedforward =
      new SimpleMotorFeedforward(0, TunnelConstants.kV, 0);

  @Logged private TalonFX tunnelMotor = new TalonFX(TunnelConstants.kTunnelMotorId);

  private AngularVelocity targetVelocity = RPM.of(0);

  public Tunnel() {
    tunnelMotorConfiguration();
    setTunnelPID(TunnelConstants.kP, 0, 0, TunnelConstants.kV);
  }

  private void tunnelMotorConfiguration() {

    TalonFXConfiguration tunnelMotorConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(TunnelConstants.kTunnelStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(TunnelConstants.kTunnelSupplyLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(TunnelConstants.kTunnelNeutralMode)
                    .withInverted(
                        TunnelConstants.kTunnelInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        TunnelConstants.kTunnelMaxVelocity.in(DegreesPerSecond))
                    .withMotionMagicAcceleration(
                        TunnelConstants.kTunnelMaxAcceleration.in(DegreesPerSecondPerSecond)))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(TunnelConstants.kTunnelGearRatio));

    tunnelMotor.getConfigurator().apply(tunnelMotorConfiguration);
  }

  @Logged(name = "tunnelVelocity")
  public AngularVelocity getVelocity() {
    AngularVelocity velocity = tunnelMotor.getVelocity().getValue();
    return velocity;
  }

  @Logged(name = "tunnelTargetVelocity")
  public AngularVelocity getTargetVelocity() {
    return this.targetVelocity;
  }

  public void runAtVelocity(AngularVelocity velocity) {
    this.targetVelocity = velocity;
    Voltage volts =
        Volts.of(
            tunnelController.calculate(getVelocity().in(RPM), velocity.in(RPM))
                + tunnelFeedforward.calculateWithVelocities(
                    getVelocity().in(RPM), velocity.in(RPM)));
    tunnelMotor.setVoltage(volts.in(Volts));
  }

  public void setTunnelPID(double kP, double kI, double kD, double kV) {
    tunnelController.setPID(kP, kI, kD);
    tunnelFeedforward.setKv(kV);
  }

  public void tuneTunnel(double kP, double kD, double kV, double targetVelocity) {

    tunnelController.setPID(kP, 0, kD);

    tunnelFeedforward.setKv(kV);

    runAtVelocity(RPM.of(targetVelocity));
  }

  @Logged(name = "tunnelVoltage")
  public Voltage getVoltage() {
    return tunnelMotor.getMotorVoltage().getValue();
  }

  @Logged(name = "tunnelCurrent")
  public Current getCurrent() {
    return tunnelMotor.getStatorCurrent().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tunnel Velocity", tunnelMotor.getVelocity().getValue().in(RPM));
    SmartDashboard.putNumber("Tunnel Voltage", getVoltage().in(Volts));
  }
}
