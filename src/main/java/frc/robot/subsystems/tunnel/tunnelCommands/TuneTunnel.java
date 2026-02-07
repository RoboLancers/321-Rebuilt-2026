/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel.tunnelCommands;

import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.util.TunableConstant;

public class TuneTunnel {

  Tunnel tunnel;

  TunableConstant kP = new TunableConstant("/Tunnel/kP", 0);

  TunableConstant kD = new TunableConstant("/Tunnel/kD", 0);

  TunableConstant kV = new TunableConstant("/Tunnel/kV", 0);

  TunableConstant targetVelocity = new TunableConstant("/Tunnel/targetVelocity", 0);

  public TuneTunnel(Tunnel tunnel) {
    this.tunnel = tunnel;
  }

  public void execute() {
    tunnel.tuneTunnel(kP.get(), kD.get(), kV.get(), targetVelocity.get());
  }
}
