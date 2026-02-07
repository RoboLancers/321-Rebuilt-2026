/* (C) Robolancers 2026 */
package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.measure.Angle;

@CustomLoggerFor(Angle.class)
public class AngleLogger extends ClassSpecificLogger<Angle> {

  public AngleLogger() {
    super(Angle.class);
  }

  @Override
  protected void update(EpilogueBackend dataLogger, Angle object) {
    dataLogger.log("", object, Degrees);
  }
}