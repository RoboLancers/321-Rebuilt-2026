/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class ClimbConstants {

  public static final double kP = 0;

  public static final double kD = 0;

  public static final double kG = 0;

  public static final int kClimbMotorId = 1;

  public static final double kClimbStatorLimit = 1;

  public static final double kClimbSupplyLimit = 1;

  public static final NeutralModeValue kClimbNeutralMode = NeutralModeValue.Brake;

  public static final boolean kClimbInverted = false;

  public static final AngularVelocity kClimbMaxVelocity = RPM.of(0);

  public static final AngularAcceleration kClimbMaxAcceleration = RotationsPerSecondPerSecond.of(0);

  public static final GravityTypeValue kClimbGravityType = GravityTypeValue.Elevator_Static;

  public static final StaticFeedforwardSignValue kClimbFeedForwardSign =
      StaticFeedforwardSignValue.UseClosedLoopSign;

  public static final Angle kClimbAngleTolerance = Degrees.of(1);
}
