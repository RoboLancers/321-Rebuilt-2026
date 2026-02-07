/* (C) Robolancers 2025 */
package frc.robot.util;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import frc.robot.subsystems.vision.VisionEstimate;

@CustomLoggerFor(VisionEstimate.class)
public class VisionEstimateLogger extends ClassSpecificLogger<VisionEstimate> {
  public VisionEstimateLogger() {
    super(VisionEstimate.class);
  }

  @Override
  public void update(EpilogueBackend backend, VisionEstimate object) {
    backend.log("", object.stdDevs(), Matrix.getStruct(Nat.N3(), Nat.N1()));
  }
}
