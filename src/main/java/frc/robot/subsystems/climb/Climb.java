/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

  private TalonFX climbMotor = new TalonFX(ClimbConstants.kClimbMotorId);

  private ArmFeedforward climbFeedforward = new ArmFeedforward(0, ClimbConstants.kG, 0, 0, 0);

  private PIDController climbController =
      new PIDController(ClimbConstants.kP, 0, ClimbConstants.kD);

  public Climb create() {
    return new Climb();
  }

  public Climb() {
    configureClimbMotors();
    setClimbPID(ClimbConstants.kP, 0.0, ClimbConstants.kD, 0.0, 0.0, 0.0, ClimbConstants.kG);
  }

  public void configureClimbMotors() {

    TalonFXConfiguration climbMotorConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ClimbConstants.kClimbStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ClimbConstants.kClimbSupplyLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(ClimbConstants.kClimbNeutralMode)
                    .withInverted(
                        ClimbConstants.kClimbInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ClimbConstants.kClimbMaxVelocity)
                    .withMotionMagicAcceleration(ClimbConstants.kClimbMaxAcceleration))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(ClimbConstants.kClimbGravityType)
                    .withStaticFeedforwardSign(ClimbConstants.kClimbFeedForwardSign));

    climbMotor.getConfigurator().apply(climbMotorConfiguration);
  }

  @Logged(name = "/climb/currentVelocity")
  public AngularVelocity getClimbVelocity() {
    AngularVelocity velocity = climbMotor.getVelocity().getValue();
    return velocity;
  }

  @Logged(name = "/climb/currentAngle")
  public Angle getAngle() {
    Angle angle = Degrees.of(climbMotor.getPosition().getValueAsDouble());
    return angle;
  }

  @Logged(name = "/climb/atTargetAngle")
  public boolean atTargetAngle(Angle targetAngle) {
    boolean atAngle =
        Math.abs(getAngle().in(Degrees) - targetAngle.in(Degrees))
            > ClimbConstants.kClimbAngleTolerance.in(Degrees);
    return atAngle;
  }

  public void goToAngle(Angle angle) {
    Voltage volts =
        Volts.of(
            climbController.calculate(getAngle().in(Degrees), angle.in(Degrees))
                + climbFeedforward.calculate(getAngle().in(Degrees), getClimbVelocity().in(RPM)));
    climbMotor.setVoltage(volts.in(Volts));
  }

  public void setClimbPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    climbMotor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withKG(kG));
  }
}
