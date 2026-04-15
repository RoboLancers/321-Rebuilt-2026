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

  public static double kGearing = 3;

  public static AngularVelocity kMaxVelocity = RPM.of(1800);

  public static AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(70);

  public static AngularVelocity kIndexVelocity = RPM.of(1500);

  public static double kOscillationAmplitude = 300;

  public static AngularVelocity kReleaseVelocity = RPM.of(-600);

  public static double kP = 0;
  public static double kD = 0;
  public static double kV = 0.0065;
  public static double kS = 0;
}
