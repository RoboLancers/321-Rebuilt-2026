/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

  @Logged private TalonFX hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
  @Logged private DigitalInput hoodLimitSwitch = new DigitalInput(HoodConstants.kLimitSwitchID);
  private Angle targetAngle = HoodConstants.kStartingAngle;

  public Hood() {
    configureMotors();
    setHoodPID(HoodConstants.kP, HoodConstants.kD, HoodConstants.kG);
  }

  public void configureMotors() {

    TalonFXConfiguration hoodMotorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(HoodConstants.kHoodStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(HoodConstants.kHoodSupplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true))
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
    hoodMotor.getConfigurator().apply(new Slot0Configs().withKP(kP).withKD(kD).withKG(kG));
  }

  public void goToAngle(Angle angle) {
    this.targetAngle = angle;
    hoodMotor.setControl(new MotionMagicVoltage(angle));
  }

  @Logged(name = "hoodPitch")
  public Angle getAngle() {
    Angle angle = Degrees.of(hoodMotor.getPosition().getValueAsDouble());
    return angle;
  }

  @Logged(name = "hoodVelocity")
  public AngularVelocity getVelocity() {
    return hoodMotor.getVelocity().getValue();
  }

  public boolean isHomedVelocity() {
    return Math.abs(getVelocity().in(RPM) - HoodConstants.kHomingVelocityFloor.in(RPM))
        <= HoodConstants.kHomingVelocityTolerance;
  }

  public boolean isHomedCurrent() {
    return getCurrent().in(Amps) >= HoodConstants.kCurrentCeiling.in(Amps);
  }

  @Logged(name = "hoodAtHomedPosition")
  public boolean getHoodAtHomedPosition() {
    return hoodLimitSwitch.get();
  }

  public void zeroEncoder() {
    hoodMotor.setPosition(HoodConstants.kZeroPosition);
  }

  public void runVolts(Voltage volts) {
    hoodMotor.setVoltage(volts.in(Volts));
  }

  @Logged(name = "hoodAtTargetAngle")
  public boolean atTargetAngle() {
    boolean atTargetAngle =
        Math.abs(getAngle().in(Degrees) - targetAngle.in(Degrees))
            < HoodConstants.kAngleTolerance.in(Degrees);
    return atTargetAngle;
  }

  public void tune(double kP, double kD, double kG, double targetAngle) {
    setHoodPID(kP, kD, kG);
    goToAngle(Degrees.of(targetAngle));
  }

  @Logged(name = "hoodTargetAngle")
  public Angle getTargetAngle() {
    return this.targetAngle;
  }

  @Logged(name = "hoodVoltage")
  public Voltage getVoltage() {
    return hoodMotor.getMotorVoltage().getValue();
  }

  @Logged(name = "hoodCurrent")
  public Current getCurrent() {
    return hoodMotor.getStatorCurrent().getValue();
  }
}
