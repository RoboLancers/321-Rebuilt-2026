/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel.tunnelCommands;

import frc.robot.subsystems.tunnel.Tunnel;

public class TuneTunnel {

  Tunnel tunnel;

  public TuneTunnel(Tunnel tunnel) {
    this.tunnel = tunnel;
  }

  public void execute() {
    tunnel.tuneTunnel();
  }
}
