/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel.tunnelCommands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.util.TunableConstant;

public class TuneTunnel {

  Tunnel tunnel;
  PIDController tunnelController;
  SimpleMotorFeedforward tunnelFeedforward;

  TunableConstant kP = new TunableConstant("/Tunnel/kP", 0);
  TunableConstant kV = new TunableConstant("/Tunnel/kV", 0);
  TunableConstant targetVelocity = new TunableConstant("/Tunnel/targetVelocity", 0);

  public TuneTunnel(
      Tunnel tunnel,
      PIDController tunnelController,
      SimpleMotorFeedforward tunnelFeedforward,
      TunableConstant kP,
      TunableConstant kV,
      TunableConstant targetVelocity) {
    this.tunnel = tunnel;
    this.tunnelController = tunnelController;
    this.tunnelFeedforward = tunnelFeedforward;
    this.kP = kP;
    this.kV = kV;
    this.targetVelocity = targetVelocity;
  }

  public void execute() {
    tunnelController.setPID(kP.get(), 0, 0);
    tunnelFeedforward.setKv(kV.get());
    tunnel.runAtVelocity(RPM.of(targetVelocity.get()));
  }
}
