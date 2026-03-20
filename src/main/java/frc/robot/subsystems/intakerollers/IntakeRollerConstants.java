/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class IntakeRollerConstants {
  public static final int kRollerMotorId = 15;
  public static final String kIntakeRollerCANbus = "rio";
  public static final boolean kStatorCurrentLimitsEnable = true;
  public static final boolean kSupplyCurrentLimitsEnable = true;
  public static final Current kStatorCurrentLimit = Amps.of(60);
  public static final Current kSupplyCurrentLimit = Amps.of(60);
  public static final AngularVelocity kIntakeFuelVelocity = RPM.of(0);
  public static final AngularVelocity kOuttakeFuelVelocity = RPM.of(0);
  public static final AngularVelocity kMaxVelocity = RPM.of(1500);
  public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(25);
  public static final double kP = 0;
  public static final double kD = 0;
  public static final double kG = 0;
  public static final double kV = 0.00415;
  public static final double kIntakeRollerGearRatio = 2;
}
