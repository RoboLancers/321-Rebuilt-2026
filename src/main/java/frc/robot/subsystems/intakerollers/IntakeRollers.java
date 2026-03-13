/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {

  @Logged private TalonFX rollerMotor = new TalonFX(IntakeRollerConstants.kRollerMotorId);
  private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  private MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
  private VoltageConfigs voltageConfigs = new VoltageConfigs();
  private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
  private Slot0Configs slot0Configs = new Slot0Configs();
  private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

  private AngularVelocity targetVelocity = RPM.of(0);

  public IntakeRollers() {
    motorConfigurations();
    setPID(IntakeRollerConstants.kP, IntakeRollerConstants.kD, IntakeRollerConstants.kV);
  }

  public void motorConfigurations() {
    motorConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
    motorConfigs.withNeutralMode(NeutralModeValue.Brake);
    feedbackConfigs.withSensorToMechanismRatio(IntakeRollerConstants.kIntakeRollerGearRatio);

    currentLimitsConfigs.withStatorCurrentLimitEnable(
        IntakeRollerConstants.kStatorCurrentLimitsEnable);
    currentLimitsConfigs.withStatorCurrentLimit(IntakeRollerConstants.kStatorCurrentLimit);
    currentLimitsConfigs.withSupplyCurrentLimitEnable(
        IntakeRollerConstants.kSupplyCurrentLimitsEnable);
    currentLimitsConfigs.withSupplyCurrentLimit(IntakeRollerConstants.kSupplyCurrentLimit);
    motionMagicConfigs
        .withMotionMagicCruiseVelocity(IntakeRollerConstants.kMaxVelocity)
        .withMotionMagicAcceleration(IntakeRollerConstants.kMaxAcceleration);

    rollerMotor.getConfigurator().apply(motorConfigs);
    rollerMotor.getConfigurator().apply(currentLimitsConfigs);
    rollerMotor.getConfigurator().apply(feedbackConfigs);
    rollerMotor.getConfigurator().apply(motionMagicConfigs);
  }

  public PIDController rollerController = new PIDController(0, 0, 0);
  public SimpleMotorFeedforward rollerFF = new SimpleMotorFeedforward(0, 0);

  public void setVelocity(AngularVelocity velocity) {
    this.targetVelocity = velocity;
    double volts =
        rollerController.calculate(getRollerVelocity().in(RPM), velocity.in(RPM))
            + rollerFF.calculateWithVelocities(getRollerVelocity().in(RPM), velocity.in(RPM));
    rollerMotor.setVoltage(volts);
  }

  public void setPID(double kP, double kD, double kV) {
    rollerController.setP(kP);
    rollerController.setD(kD);
    rollerFF.setKv(kV);
  }

  public void setVoltage(Voltage volts) {
    rollerMotor.setVoltage(volts.in(Volts));
  }

  public void tune(double kP, double kD, double kV, AngularVelocity rollerTargetVelocity) {
    setPID(kP, kD, kV);
    setVelocity(rollerTargetVelocity);
  }

  @Logged(name = "intakeRollersVelocity")
  public AngularVelocity getRollerVelocity() {
    return rollerMotor.getVelocity().getValue();
  }

  @Logged(name = "intakeRollersAtTargetVelocity")
  public boolean atTargetVelocity() {
    return rollerMotor.getVelocity().getValue() == targetVelocity;
  }

  @Logged(name = "intakeRollersTargetVelocity")
  public AngularVelocity getRollerTargetVelocity() {
    return targetVelocity;
  }

  @Logged(name = "intakeRollersVoltage")
  public Voltage getVoltage() {
    return rollerMotor.getMotorVoltage().getValue();
  }

  @Logged(name = "intakeRollersCurrent")
  public Current getCurrent() {
    return rollerMotor.getStatorCurrent().getValue();
  }

  public void periodic() {
    SmartDashboard.putNumber("Intake Roller Velocity", getRollerVelocity().in(RPM));
    SmartDashboard.putNumber("Intake Roller Voltage", getVoltage().in(Volts));
  }
}
