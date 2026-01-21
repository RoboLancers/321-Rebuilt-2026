package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{
    
TalonFX pivotMotor = new TalonFX(PivotConstants.kPivotMotorId);

public void configureMotors() {

    TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(PivotConstants.kPivotMotorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(
                    PivotConstants.kPivotMotorInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(
                    PivotConstants.kPivotMotorMaxVelocity.in(MetersPerSecond))
                .withMotionMagicAcceleration(
                    PivotConstants.kPivotMotorMaxAcceleration.in(MetersPerSecondPerSecond)
                ))
        .withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        );
                
    pivotMotor.getConfigurator().apply(pivotMotorConfigs);
}

public void setUpPID() {
    pivotMotor.getConfigurator()
        .apply(new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKG(0)
            .withKS(0)
            .withKA(0)
            .withKV(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        );
}

public void goToAngle(Angle targetAngle){
    pivotMotor.setControl(new MotionMagicVoltage(targetAngle));
}

public Angle getAngle() {
    Angle angle = Degrees.of(pivotMotor.getPosition().getValueAsDouble());
    return angle;
}

public void resetEncoder(){
    pivotMotor.setPosition(PivotConstants.kStartingAngle);
}

public boolean atTargetAngle(Angle targetAngle){
    boolean atTargetAngle =
        Math.abs(getAngle().in(Degrees) - targetAngle.in(Degrees)) 
        < PivotConstants.kAngleTolerance.in(Degrees);
    return atTargetAngle;
}

}
