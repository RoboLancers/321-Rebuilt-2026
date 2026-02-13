/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public class IntakeRollerConstants {
  public static final int kRollerMotorId = 0;
  public static final boolean kStatorCurrentLimitsEnable = true;
  public static final boolean kSupplyCurrentLimitsEnable = true;
  public static final Current kStatorCurrentLimit = Amps.of(40);
  public static final Current kSupplyCurrentLimit = Amps.of(40);
  public static final double kSensorToMechanismRatio = 0;
  public static final double kIntakeFuelVelocity = 0;
  public static final double kOuttakeFuelVelocity = 0;
  public static final double kP = 0;
  public static final double kD = 0;
  public static final double kG = 0;
  public static final double kV = 0;
  public static final double kIntakeRollerGearRatio = 2/1;
}
