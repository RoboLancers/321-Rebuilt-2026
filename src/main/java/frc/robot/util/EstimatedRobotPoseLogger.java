/* (C) RoboLancers 2026 */
package frc.robot.util;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.EstimatedRobotPose;

@CustomLoggerFor(EstimatedRobotPose.class)
public class EstimatedRobotPoseLogger extends ClassSpecificLogger<EstimatedRobotPose> {

  public EstimatedRobotPoseLogger() {
    super(EstimatedRobotPose.class);
  }

  @Override
  protected void update(EpilogueBackend backend, EstimatedRobotPose object) {
    backend.log("3D Vision Pose Estimate (m, rad)", object.estimatedPose, Pose3d.struct);
    backend.log("Pose Estimate Timestamp (s)", object.timestampSeconds);
  }
}
