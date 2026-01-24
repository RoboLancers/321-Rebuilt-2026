/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood;

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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;

@Logged
public class Hood extends SubsystemBase {

  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(HoodConstants.kHoodEncoderId);

  private TalonFX hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);

  public Hood() {
    configureMotors();
    setHoodPID(HoodConstants.kP, HoodConstants.kD, HoodConstants.kG);
    zeroEncoder();
  }

  public void configureMotors() {

    TalonFXConfiguration hoodMotorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(HoodConstants.kHoodMotorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        HoodConstants.kHoodMotorInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        HoodConstants.kHoodMotorMaxVelocity.in(MetersPerSecond))
                    .withMotionMagicAcceleration(
                        HoodConstants.kHoodMotorMaxAcceleration.in(MetersPerSecondPerSecond)))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

    hoodMotor.getConfigurator().apply(hoodMotorConfigs);
  }

  public void setHoodPID(double kP, double kD, double kG) {
    hoodMotor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(kP)
                .withKD(kD)
                .withKG(kG)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));
  }

  public void goToAngle(Angle targetAngle) {
    hoodMotor.setControl(new MotionMagicVoltage(targetAngle));
  }

  public Angle getAngle() {
    Angle angle = Degrees.of(hoodMotor.getPosition().getValueAsDouble());
    return angle;
  }

  public void zeroEncoder() {
    hoodMotor.setPosition(absoluteEncoder.get());
  }

  public boolean atTargetAngle(Angle targetAngle) {
    boolean atTargetAngle =
        Math.abs(getAngle().in(Degrees) - targetAngle.in(Degrees))
            < HoodConstants.kAngleTolerance.in(Degrees);
    return atTargetAngle;
  }

  public void tune() {
    TunableConstant kG = new TunableConstant("Hood/kG/", 0);
    TunableConstant kD = new TunableConstant("Hood/kD/", 0);
    TunableConstant kP = new TunableConstant("Hood/kP/", 0);
    TunableConstant targetAngle = new TunableConstant("Hood/targetAngle/", 0);

    setHoodPID(kP.get(), kD.get(), kG.get());

    goToAngle(Degrees.of(targetAngle.get()));
  }
}
