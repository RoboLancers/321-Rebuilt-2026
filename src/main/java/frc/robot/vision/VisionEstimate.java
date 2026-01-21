package frc.robot.vision;

import org.photonvision.EstimatedRobotPose;

public record VisionEstimate (
EstimatedRobotPose estimatedPose,
double standardDeviations
) {}
