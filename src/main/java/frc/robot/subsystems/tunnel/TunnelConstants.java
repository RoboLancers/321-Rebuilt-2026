/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class TunnelConstants {

  public static final int kTunnelMotorId = 1;

  public static final int kTunnelStatorLimit = 1;

  public static final int kTunnelSupplyLimit = 1;

  public static final NeutralModeValue kTunnelNeutralMode = NeutralModeValue.Brake;

  public static final boolean kTunnelInverted = false;

  public static final LinearVelocity kTunnelMaxVelocity = MetersPerSecond.of(1);

  public static final LinearAcceleration kTunnelMaxAcceleration = MetersPerSecondPerSecond.of(1);
}
