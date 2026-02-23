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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Climb extends SubsystemBase {

  private TalonFX climbMotor = new TalonFX(ClimbConstants.kClimbMotorId);

  private TalonFX pivotClimbMotor = new TalonFX(ClimbConstants.kPivotClimbMotorId);

  private Relay magnetRelay = new Relay(ClimbConstants.kMagnetId);

  private CANcoder clawEncoder = new CANcoder(ClimbConstants.kEncoderId);

  private ArmFeedforward climbFeedforward = new ArmFeedforward(0, ClimbConstants.kG, 0, 0, 0);

  private PIDController climbController =
      new PIDController(ClimbConstants.kP, 0, ClimbConstants.kD);

  private PIDController pivotClimbController =
      new PIDController(ClimbConstants.kPivotP, 0, ClimbConstants.kPivotD);

  public Climb create() {
    return new Climb();
  }

  public Climb() {
    configureClimbMotors();
    setClimbPID(ClimbConstants.kP, 0.0, ClimbConstants.kD);
    setPivotClimbPID(ClimbConstants.kPivotP, 0.0, ClimbConstants.kPivotD);
    setClimbFeedforward(ClimbConstants.kG);
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

    TalonFXConfiguration pivotClimbMotorConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ClimbConstants.kPivotClimbStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ClimbConstants.kPivotClimbSupplyLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(ClimbConstants.kPivotClimbNeutralMode)
                    .withInverted(
                        ClimbConstants.kPivotClimbInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ClimbConstants.kPivotClimbMaxVelocity)
                    .withMotionMagicAcceleration(ClimbConstants.kPivotClimbMaxAcceleration));

    climbMotor.getConfigurator().apply(climbMotorConfiguration);
    pivotClimbMotor.getConfigurator().apply(pivotClimbMotorConfiguration);
  }

  @Logged(name = "climbVelocity")
  public AngularVelocity getClimbVelocity() {
    AngularVelocity velocity = climbMotor.getVelocity().getValue();
    return velocity;
  }

  @Logged(name = "pivotClimbVelocity")
  public AngularVelocity getPivotClimbVelocity() {
    AngularVelocity velocity = pivotClimbMotor.getVelocity().getValue();
    return velocity;
  }

  @Logged(name = "climbAngle")
  public Angle getClimbAngle() {
    Angle angle = Degrees.of(clawEncoder.getAbsolutePosition().getValueAsDouble());
    return angle;
  }

  @Logged(name = "climbPivotAngle")
  public Angle getPivotAngle() {
    Angle angle = Degrees.of(pivotClimbMotor.getPosition().getValueAsDouble());
    return angle;
  }

  @Logged(name = "climbPivotVoltage")
  public Voltage getPivotVoltage(){
    Voltage voltage = pivotClimbMotor.getMotorVoltage().getValue();
    return voltage;
  }

  @Logged(name = "climbPivotCurrent")
  public Current getPivotCurrent(){
    Current current = pivotClimbMotor.getStatorCurrent().getValue();
    return current;
  }

  @Logged(name = "climbVoltage")
  public Voltage getClimbVoltage(){
    Voltage voltage = climbMotor.getMotorVoltage().getValue();
    return voltage;
  }

  @Logged(name = "climbCurrent")
  public Current getClimbCurrent(){
    Current current = climbMotor.getStatorCurrent().getValue();
    return current;
  }

  @Logged(name = "magnetOn")
    public boolean getMagnetActivationStatus(){
      boolean magnetOn = (magnetRelay.get() == Value.kOn) ? true : false;
      return magnetOn;
    }
  

  @Logged
  public boolean atTargetAngle() {
    boolean atAngle =
        Math.abs(getClimbAngle().in(Degrees) - ClimbConstants.kClimbTargetAngle.in(Degrees))
            < ClimbConstants.kClimbAngleTolerance.in(Degrees);
    return atAngle;
  }

  @Logged
  public boolean atPivotTargetAngle() {
    boolean atAngle =
        Math.abs(getPivotAngle().in(Degrees) - ClimbConstants.kPivotTargetAngle.in(Degrees))
            < ClimbConstants.kPivotClimbAngleTolerance.in(Degrees);
    return atAngle;
  }

  public void goToAngle(Angle angle) {
    Voltage volts =
        Volts.of(
            climbController.calculate(getClimbAngle().in(Degrees), angle.in(Degrees))
                + climbFeedforward.calculate(getClimbAngle().in(Degrees), getClimbVelocity().in(RPM)));
    climbMotor.setVoltage(volts.in(Volts));
  }

  public void goToPivotAngle(Angle angle) {
    Voltage volts =
        Volts.of(pivotClimbController.calculate(getPivotAngle().in(Degrees), angle.in(Degrees)));
    pivotClimbMotor.setVoltage(volts.in(Volts));
  }

  public boolean atClimbVoltage(Voltage voltage) {
    boolean atVoltage = climbMotor.getMotorVoltage() == voltage;
    return atVoltage;
  }

  public boolean atPivotClimbVoltage(Voltage voltage) {
    boolean atVoltage = pivotClimbMotor.getMotorVoltage() == voltage;
    return atVoltage;
  }

  public void setClimbPID(double kP, double kI, double kD) {
    climbController.setPID(kP, kI, kD);
  }

  public void setPivotClimbPID(double kPivotP, double kPivotI, double kPivotD) {
    pivotClimbController.setPID(kPivotP, kPivotI, kPivotD);
  }

  public void tuneClimb(double kP, double kD, double kG, double kTargetAngle) {
    climbController.setPID(kP, 0, kD);
    climbFeedforward.setKg(kG);
    goToAngle(Degrees.of(kTargetAngle));
  }

  public void setClimbFeedforward(double kG) {
    climbFeedforward.setKg(kG);
  }

  public void tunePivotClimb(double kPivotP, double kPivotD, double kPivotTargetAngle) {
    pivotClimbController.setPID(kPivotP, 0, kPivotD);
    goToPivotAngle(Degrees.of(kPivotTargetAngle));
  }

  public void turnOnMagnet() {
    magnetRelay.set(Value.kOn);
  }

  public void turnOffMagnet() {
    magnetRelay.set(Value.kOff);
  }
}
