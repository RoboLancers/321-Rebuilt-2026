/* (C) RoboLancers 2026 */
package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;

public record VisionEstimate(EstimatedRobotPose estimatedPose, double standardDeviation) {}
