/* (C) RoboLancers 2026 */
package frc.robot.util;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

// For creating a custom logger, refer to
// https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html#logging-third-party-data
@CustomLoggerFor(TimeOfFlight.class)
public class TimeOfFlightLogger extends ClassSpecificLogger<TimeOfFlight> {

  public TimeOfFlightLogger() {
    super(TimeOfFlight.class);
  }

  @Override
  protected void update(EpilogueBackend dataLogger, TimeOfFlight object) {
    dataLogger.log("Range (mm)", object.getRange());
    dataLogger.log("Range STDDEV (mm)", object.getRangeSigma());
    dataLogger.log("Light Level", object.getAmbientLightLevel());
  }
}
