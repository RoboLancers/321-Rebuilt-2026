package frc.robot.subsystems.tunnel;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;

public class Tunnel {

    double kP;

    double kV;

    PIDController tunnelController = new PIDController(kP, 0, 0);

    SimpleMotorFeedforward tunnelFeedforward = new SimpleMotorFeedforward(0, kV, 0);

    TalonFX tunnelMotor = new TalonFX(TunnelConstants.kTunnelMotorId);

    public Tunnel(){
        tunnelMotorConfiguration();
        setTunnelPID(kP, 0, 0, 0, kV, 0, 0);
    }

    private void tunnelMotorConfiguration() {
        
        TalonFXConfiguration tunnelMotorConfiguration =
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(TunnelConstants.kTunnelStatorLimit)
                        .withSupplyCurrentLimit(TunnelConstants.kTunnelSupplyLimit))
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(TunnelConstants.kTunnelNeutralMode)
                        .withInverted(TunnelConstants.kTunnelInverted
                            ?InvertedValue.Clockwise_Positive
                            :InvertedValue.CounterClockwise_Positive))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(TunnelConstants.kTunnelMaxVelocity.in(MetersPerSecond))
                        .withMotionMagicAcceleration(TunnelConstants.kTunnelMaxAcceleration.in(MetersPerSecondPerSecond)));
        
        tunnelMotor.getConfigurator().apply(tunnelMotorConfiguration);
    }

    public AngularVelocity getVelocity(){
        AngularVelocity velocity = RPM.of(tunnelMotor.getVelocity().getValueAsDouble());
        return velocity;
    }

    public void runAtVelocity(AngularVelocity velocity){
        velocity =
        RPM.of(
            tunnelController.calculate(getVelocity().in(RPM), velocity.in(RPM)) +
            tunnelFeedforward.calculateWithVelocities(getVelocity().in(RPM), velocity.in(RPM)));
        tunnelMotor.setVoltage(velocity.in(RPM));
    }

    public void setTunnelPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG){
        tunnelMotor.getConfigurator().apply(
            new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withKG(kG)
        );
    }
}
