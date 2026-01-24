/* (C) RoboLancers 2026 */
package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {

  public static int kMotorID = 0;

  public static double kCurrentLimit = 40;

  public static NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

  public static InvertedValue kInverted = InvertedValue.Clockwise_Positive;

  public static double kGearing = 1;

  public static AngularVelocity kMaxVelocity = RPM.of(1500);

  public static AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(10);

  public static AngularVelocity kIndexVelocity = RPM.of(0);

  public static AngularVelocity kReleaseVelocity = RPM.of(0);
}
