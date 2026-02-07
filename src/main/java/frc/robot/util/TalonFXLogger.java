/* (C) Robolancers 2026 */
package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

// For creating a custom logger, refer to
// https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html#logging-third-party-data
@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  protected void update(EpilogueBackend dataLogger, TalonFX object) {
    dataLogger.log("CAN ID", object.getDeviceID());
    dataLogger.log("Requested Speed (Duty Cycle)", object.get());
    dataLogger.log("Supply Voltage (V)", object.getSupplyVoltage().getValueAsDouble());
    dataLogger.log("Motor Voltage (V)", object.getMotorVoltage().getValueAsDouble());
    dataLogger.log("Stator Current (A)", object.getStatorCurrent().getValueAsDouble());
    dataLogger.log("Supply Current (A)", object.getSupplyCurrent().getValueAsDouble());
    dataLogger.log("Motor Temperature (C)", object.getDeviceTemp().getValueAsDouble());
    dataLogger.log("Encoder Position", object.getPosition().getValueAsDouble());
    dataLogger.log("Encoder Velocity", object.getVelocity().getValueAsDouble());
  }
}