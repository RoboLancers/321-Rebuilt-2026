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

  public static final String kBackLeftCameraName = "Back Left Swerve Module Camera";

  public static final String kTopElevatorCameraName = "Top Elevator Camera";

  public static final String kBottomElevatorCameraName = "Bottom Elevator Camera";

  public static final String backRightCameraName = "Back Right Camera";

  public static final double kMinimumConfidence = 0.85;

  public static final double kTargetConfidence = 0.95;

  public static final Transform3d kTopElevatorTransform =
      new Transform3d(
          Meters.of(0.2314956),
          Meters.of(-0.16764),
          Meters.of(0.3103626),
          new Rotation3d(Degrees.zero(), Degrees.of(-1), Degrees.of(48)));

  public static final Transform3d kBottomElevatorTransform =
      new Transform3d(
          Meters.of(0.2280412),
          Meters.of(-0.1723644),
          Meters.of(0.2151634),
          new Rotation3d(Degrees.zero(), Degrees.of(-18), Degrees.of(-10)));

  public static final Transform3d kBackLeftTransform =
      new Transform3d(
          Inches.of(-11.1),
          Inches.of(8.7),
          Inches.of(7.82 - 1.575),
          new Rotation3d(Degrees.of(0), Degrees.of(-5.5), Degrees.of(-20)));

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
