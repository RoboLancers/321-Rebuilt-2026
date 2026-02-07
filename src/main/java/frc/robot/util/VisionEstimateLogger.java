/* (C) RoboLancers 2026 */
package frc.robot.util;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import frc.robot.subsystems.vision.VisionEstimate;

@CustomLoggerFor(VisionEstimate.class)
public class VisionEstimateLogger extends ClassSpecificLogger<VisionEstimate> {
  public VisionEstimateLogger() {
    super(VisionEstimate.class);
  }

  @Override
  public void update(EpilogueBackend backend, VisionEstimate object) {
    backend.log("Standard Deviation", object.standardDeviations());
  }
}
