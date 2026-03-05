/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;

public class RobotConstants {
  public static final Time kRobotLoopPeriod = Seconds.of(TimedRobot.kDefaultPeriod);
  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final Angle kAngleTolerance = Degrees.of(0.5);
}
