/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ClimbConstants {

  public static final Angle kTargetAngle = Degrees.of(360);

  public static final Angle kTargetPivotAngle = Degrees.of(90);

  public static final double kP = 0;

  public static final double kD = 0;

  public static final double kG = 0;

  public static final double kPivotP = 0;

  public static final double kPivotD = 0;

  public static final int kClimbMotorId = 1;

  public static final int kPivotClimbMotorId = 2;

  public static final int kMagnetId = 0;

  public static final int kEncoderId = 0;

  public static final Current kClimbStatorLimit = Amps.of(40);

  public static final Current kClimbSupplyLimit = Amps.of(40);

  public static final Current kPivotClimbStatorLimit = Amps.of(40);

  public static final Current kPivotClimbSupplyLimit = Amps.of(40);

  public static final NeutralModeValue kClimbNeutralMode = NeutralModeValue.Brake;

  public static final boolean kClimbInverted = false;

  public static final NeutralModeValue kPivotClimbNeutralMode = NeutralModeValue.Brake;

  public static final boolean kPivotClimbInverted = false;

  public static final AngularVelocity kClimbMaxVelocity = RPM.of(0);

  public static final AngularAcceleration kClimbMaxAcceleration = RotationsPerSecondPerSecond.of(0);

  public static final AngularVelocity kPivotClimbMaxVelocity = RPM.of(0);

  public static final AngularAcceleration kPivotClimbMaxAcceleration =
      RotationsPerSecondPerSecond.of(0);

  public static final GravityTypeValue kClimbGravityType = GravityTypeValue.Arm_Cosine;

  public static final GravityTypeValue kPivotClimbGravityType = GravityTypeValue.Arm_Cosine;

  public static final StaticFeedforwardSignValue kClimbFeedForwardSign =
      StaticFeedforwardSignValue.UseClosedLoopSign;

  public static final StaticFeedforwardSignValue kPivotClimbFeedForwardSign =
      StaticFeedforwardSignValue.UseClosedLoopSign;

  public static final Angle kClimbAngleTolerance = Degrees.of(1);

  public static final Angle kPivotClimbAngleTolerance = Degrees.of(1);

  public static final double kClawGearRatio = 12 / 40;
  public static final double kSprocketGearRatio = 3 / 4;
  public static final double kClimbPivotGearRatio = 160;
}
