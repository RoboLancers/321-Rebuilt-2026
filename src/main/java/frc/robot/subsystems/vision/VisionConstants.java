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
          Inches.of(-11.113),
          Inches.of(9.233),
          Inches.of(15.160),
          new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(48)));

  public static final Transform3d kRightClimbCameraTransform =
      new Transform3d(
          Inches.of(-5.119),
          Inches.of(9.064),
          Inches.of(15.723),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(154)));

  public static final Transform3d kLeftShooterCameraTransform =
      new Transform3d(
          Inches.of(-4.310),
          Inches.of(-3.166),
          Inches.of(16.330),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-115)));

  public static final Transform3d kRightShooterCameraTransform =
      new Transform3d(
          Inches.of(-12.000),
          Inches.of(-3.173),
          Inches.of(12.948),
          new Rotation3d(Degrees.zero(), Degrees.of(-15), Degrees.of(-90)));

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
