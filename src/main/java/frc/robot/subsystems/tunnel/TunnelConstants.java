/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

@Logged
public class TunnelConstants {

  public static final double kP = 0;

  public static final double kD = 0;

  public static final double kV = 0.01815;

  public static final double kTunnelGearRatio = 9 / 1;

  public static final int kTunnelMotorId = 13;

  public static final Current kTunnelStatorLimit = Amps.of(40);

  public static final Current kTunnelSupplyLimit = Amps.of(40);

  public static final NeutralModeValue kTunnelNeutralMode = NeutralModeValue.Brake;

  public static final boolean kTunnelInverted = true;

  public static final AngularVelocity kPassFuelRPM = RPM.of(600);

  public static final AngularVelocity kTunnelMaxVelocity = RPM.of(700);

  public static final AngularAcceleration kTunnelMaxAcceleration =
      RotationsPerSecondPerSecond.of(1000);
}
