/* (C) RoboLancers 2026 */
package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {

  public static int kMotorID = 17;

  public static double kCurrentLimit = 40;

  public static boolean kInverted = true;

  public static double kGearing = 4;

  public static AngularVelocity kMaxVelocity = RPM.of(1000);

  public static AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(16);

  public static AngularVelocity kIndexVelocity = RPM.of(500);

  public static AngularVelocity kReleaseVelocity = RPM.of(0);

  public static double kP = 0;
  public static double kD = 0;
  public static double kV = 0.00795;
  public static double kS = 0.50;
}
