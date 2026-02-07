/* (C) Robolancers 2026 */
package frc.robot.util;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.measure.AngularVelocity;

// For creating a custom logger, refer to
// https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html#logging-third-party-data
@CustomLoggerFor(AngularVelocity.class)
public class AngularVelocityLogger extends ClassSpecificLogger<AngularVelocity> {

  public AngularVelocityLogger() {
    super(AngularVelocity.class);
  }

  @Override
  protected void update(EpilogueBackend dataLogger, AngularVelocity object) {
    dataLogger.log("", object.in(RPM));
  }
}