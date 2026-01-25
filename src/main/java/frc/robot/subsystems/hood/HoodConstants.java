/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class HoodConstants {

  public static final int kHoodMotorId = 1;

  public static final boolean kHoodMotorInverted = false;

  public static final Current kHoodMotorCurrentLimit = Amps.of(6);

  public static final LinearVelocity kHoodMotorMaxVelocity = MetersPerSecond.of(1);

  public static final LinearAcceleration kHoodMotorMaxAcceleration = MetersPerSecondPerSecond.of(1);

  public static final Angle kStartingAngle = Degrees.of(60);

  public static final Angle kAngleTolerance = Degrees.of(1);
}
