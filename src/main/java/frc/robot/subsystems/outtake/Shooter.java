/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  @Logged private TalonFX topShooterMotor = new TalonFX(OuttakeConstants.kTopMotorID);
  @Logged private TalonFX bottomShooterMotor = new TalonFX(OuttakeConstants.kBottomMotorID);

  @Logged private TimeOfFlight distanceSensor = new TimeOfFlight(OuttakeConstants.kTimeOfFlightID);

  public PIDController pidController = new PIDController(0, 0, 0);
  public SimpleMotorFeedforward simpleMotorFeedForward = new SimpleMotorFeedforward(0, 0, 0);

  private AngularVelocity targetShooterVelocity = RPM.of(0);

  public Shooter() {

    configureMotors();
    setPID(OuttakeConstants.kP, OuttakeConstants.kD, OuttakeConstants.kV);
  }

  private void configureMotors() {

    TalonFXConfiguration topConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(OuttakeConstants.kStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(OuttakeConstants.kSupplyLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        OuttakeConstants.kInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(OuttakeConstants.kGearing)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(OuttakeConstants.kMaxVelocity)
                    .withMotionMagicAcceleration(OuttakeConstants.kMaxAcceleration));

    TalonFXConfiguration bottomConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(OuttakeConstants.kStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(OuttakeConstants.kSupplyLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        OuttakeConstants.kInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(OuttakeConstants.kGearing)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(OuttakeConstants.kMaxVelocity)
                    .withMotionMagicAcceleration(OuttakeConstants.kMaxAcceleration));

    topShooterMotor.getConfigurator().apply(topConfiguration);
    bottomShooterMotor.getConfigurator().apply(bottomConfiguration);
  }

  private void setPID(double kP, double kD, double kV) {

    pidController.setP(kP);
    pidController.setD(kD);
    simpleMotorFeedForward.setKv(kV);
  }

  public void setTargetVelocity(AngularVelocity rpm) {
    targetShooterVelocity = rpm;
  }

  public void goToVelocity(AngularVelocity rpm) {
    double volts =
        pidController.calculate(getTopVelocity().in(RPM), rpm.in(RPM))
            + simpleMotorFeedForward.calculateWithVelocities(getTopVelocity().in(RPM), rpm.in(RPM));
    topShooterMotor.setVoltage(volts);
    bottomShooterMotor.setControl(
        new Follower(OuttakeConstants.kTopMotorID, OuttakeConstants.kFollowerReversed));
  }

  public void setVoltage(Voltage volts) {
    topShooterMotor.setVoltage(volts.in(Volts));
    bottomShooterMotor.setVoltage(volts.in(Volts));
  }

  public AngularVelocity getScoreVelocity(Distance hubDistance) {
    double rpm = 0;
    rpm = 522.6426 * Math.pow(1.00266, hubDistance.in(Inches));
    return RPM.of(rpm);
  }

  public void tune(double kP, double kD, double kV, double targetRPM) {
    setPID(kP, kD, kV);
    goToVelocity(RPM.of(targetRPM));
  }

  @Logged(name = "shooterTargetVelocity")
  public AngularVelocity getTargetShooterVelocity() {
    return this.targetShooterVelocity;
  }

  @Logged(name = "shooterTopMotorVelocity")
  public AngularVelocity getTopVelocity() {
    return topShooterMotor.getVelocity().getValue();
  }

  @Logged(name = "shooterBottomMotorVelocity")
  public AngularVelocity getBottomVelocity() {
    return bottomShooterMotor.getVelocity().getValue();
  }

  @Logged(name = "topShooterVoltage")
  public Voltage getTopVoltage() {
    return topShooterMotor.getMotorVoltage().getValue();
  }

  @Logged(name = "bottomShooterVoltage")
  public Voltage getBottomVoltage() {
    return bottomShooterMotor.getMotorVoltage().getValue();
  }

  @Logged(name = "topShooterCurrent")
  public Current getTopCurrent() {
    return topShooterMotor.getStatorCurrent().getValue();
  }

  @Logged(name = "bottomShooterCurrent")
  public Current getCurrent() {
    return bottomShooterMotor.getStatorCurrent().getValue();
  }

  @Logged(name = "shooterHasFuel")
  public boolean getFuelInShooter() {
    return distanceSensor.getRange() < OuttakeConstants.kFuelRange;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Top Motor Velocity", topShooterMotor.getVelocity().getValue().in(RPM));
    SmartDashboard.putNumber(
        "Bottom Motor Velocity", bottomShooterMotor.getVelocity().getValue().in(RPM));
    SmartDashboard.putNumber(
        "Top Shooter Motor Voltage", topShooterMotor.getMotorVoltage().getValue().in(Volts));
    SmartDashboard.putNumber(
        "Bottom Shooter Motor Voltage", bottomShooterMotor.getMotorVoltage().getValue().in(Volts));
    SmartDashboard.putNumber(
        "Top Rotor Velocity", topShooterMotor.getRotorVelocity().getValue().in(RPM));
    SmartDashboard.putNumber(
        "Bottom Rotor Velocity", bottomShooterMotor.getRotorVelocity().getValue().in(RPM));
  }
}
