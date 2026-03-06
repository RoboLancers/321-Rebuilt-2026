/* (C) RoboLancers 2026 */
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotConstants;

public class VisionConstants {

  public static final String kLeftClimbCameraName = "Left Climb Camera";

  public static final String kRightClimbCameraName = "Right Climb Camera";

  public static final String kLeftShooterCameraName = "Left Shooter Camera";

  public static final String kRightShooterCameraName = "Right Shooter Camera";

  public static final double kMinimumConfidence = 0.85;

  public static final double kTargetConfidence = 0.95;

  public static final Transform3d kLeftClimbCameraTransform =
      new Transform3d(
          Inches.of(-11.841),
          Inches.of(10.010),
          Inches.of(5.797),
          new Rotation3d(Degrees.zero(), Degrees.of(15), Degrees.of(130)));

  public static final Transform3d kRightClimbCameraTransform =
      new Transform3d(
          Inches.of(-8.591),
          Inches.of(11.797),
          Inches.of(5.815),
          new Rotation3d(Degrees.of(0), Degrees.of(20), Degrees.of(30)));

  public static final Transform3d kLeftShooterCameraTransform =
      new Transform3d(
          Inches.of(-9.026),
          Inches.of(-11.559),
          Inches.of(6.062),
          new Rotation3d(Degrees.of(0), Degrees.of(45), Degrees.of(15))); //-75

  public static final Transform3d kRightShooterCameraTransform =
      new Transform3d(
          Inches.of(-11.123),
          Inches.of(-9.808),
          Inches.of(6.084),
          new Rotation3d(Degrees.zero(), Degrees.of(45), Degrees.of(-15))); //-105

  public static final Distance kAllowedFieldDistance = Meters.of(2.5);
  public static final Distance kAllowedFieldHeight = Meters.of(1.25);
  public static final Rectangle2d kAllowedFieldArea =
      new Rectangle2d(
          new Translation2d(-kAllowedFieldDistance.in(Meters), -kAllowedFieldDistance.in(Meters)),
          new Translation2d(
              RobotConstants.kAprilTagLayout.getFieldLength() + kAllowedFieldDistance.in(Meters),
              RobotConstants.kAprilTagLayout.getFieldWidth() + kAllowedFieldDistance.in(Meters)));

  public static final int brightnessScaler = 0;
}
