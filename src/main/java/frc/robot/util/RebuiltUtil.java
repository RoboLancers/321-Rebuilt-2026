/* (C) RoboLancers 2026 */
package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

public class RebuiltUtil {

  private static final Pose2d redHubPose =
      new Pose2d(Meters.of(11.9231), Meters.of(4.03), Rotation2d.kZero);
  private static final Pose2d blueHubPose =
      new Pose2d(Meters.of(4.6169), Meters.of(4.03), new Rotation2d(Degrees.of(180)));

  public static Pose2d getHub() {
    return MyAlliance.isRed() ? redHubPose : blueHubPose;
  }

  public static Rotation2d getHubHeading(Supplier<Pose2d> robotPose) {

    Rotation2d rotationToHub =
        new Rotation2d(
            Radians.of(
                Math.PI
                    + getHub().getRotation().getRadians()
                    + Math.atan(
                        (getHub().getY() - robotPose.get().getY())
                            / (getHub().getX() - robotPose.get().getX()))));

    return rotationToHub;
  }
}
