/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;

@Logged
public class Tunnel extends SubsystemBase {

  PIDController tunnelController = new PIDController(TunnelConstants.kP, 0, 0);

  SimpleMotorFeedforward tunnelFeedforward = new SimpleMotorFeedforward(0, TunnelConstants.kV, 0);

  @Logged TalonFX tunnelMotor = new TalonFX(TunnelConstants.kTunnelMotorId);

  public Tunnel() {
    tunnelMotorConfiguration();
    setTunnelPID(TunnelConstants.kP, 0, TunnelConstants.kV);
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
                        TunnelConstants.kTunnelMaxAcceleration.in(DegreesPerSecondPerSecond)));

    tunnelMotor.getConfigurator().apply(tunnelMotorConfiguration);
  }

  @Logged(name = "tunnelVelocity")
  public AngularVelocity getVelocity() {
    AngularVelocity velocity = RPM.of(tunnelMotor.getVelocity().getValueAsDouble());
    return velocity;
  }

  public void runAtVelocity(AngularVelocity velocity) {
    Voltage volts =
        Volts.of(
            tunnelController.calculate(getVelocity().in(RPM), velocity.in(RPM))
                + tunnelFeedforward.calculateWithVelocities(
                    getVelocity().in(RPM), velocity.in(RPM)));
    tunnelMotor.setVoltage(volts.in(Volts));
  }

  public void setTunnelPID(double kP, double kI, double kD) {
    tunnelController.setPID(kP, kI, kD);
  }

  public void tuneTunnel() {
    TunableConstant kP = new TunableConstant("/Tunnel/kP", 0);

    TunableConstant kD = new TunableConstant("/Tunnel/kD", 0);

    TunableConstant kV = new TunableConstant("/Tunnel/kV", 0);

    TunableConstant targetVelocity = new TunableConstant("/Tunnel/targetVelocity", 0);

    tunnelController.setPID(kP.get(), 0, kD.get());

    tunnelFeedforward.setKv(kV.get());

    runAtVelocity(RPM.of(targetVelocity.get()));
  }
}
