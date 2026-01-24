/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

@Logged
public class TunnelConstants {

  public static final double kP = 0;

  public static final double kD = 0;

  public static final double kV = 0;

  public static final int kTunnelMotorId = 1;

  public static final int kTunnelStatorLimit = 1;

  public static final int kTunnelSupplyLimit = 1;

  public static final NeutralModeValue kTunnelNeutralMode = NeutralModeValue.Brake;

  public static final boolean kTunnelInverted = false;

  public static final AngularVelocity kTunnelMaxVelocity = RPM.of(1);

  public static final AngularAcceleration kTunnelMaxAcceleration =
      RotationsPerSecondPerSecond.of(1);
}
