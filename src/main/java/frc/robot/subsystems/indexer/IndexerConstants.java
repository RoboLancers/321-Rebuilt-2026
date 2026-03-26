/* (C) RoboLancers 2026 */
package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {

  public static int kMotorID = 17;

  public static double kCurrentLimit = 60;

  public static boolean kInverted = false;

  public static double kGearing = 9;

  public static AngularVelocity kMaxVelocity = RPM.of(615);

  public static AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(18);

  public static AngularVelocity kIndexVelocity = RPM.of(560);

  public static double kOscillationAmplitude = 40;

  public static AngularVelocity kReleaseVelocity = RPM.of(0);

  public static double kP = 0;
  public static double kD = 0;
  public static double kV = 0.018;
  public static double kS = 0;
}
