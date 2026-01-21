package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;

public record VisionEstimate (
EstimatedRobotPose estimatedPose,
double standardDeviations
) {}
