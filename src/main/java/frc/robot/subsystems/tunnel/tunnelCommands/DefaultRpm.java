/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel.tunnelCommands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tunnel.Tunnel;

public class DefaultRpm extends Command {

  Tunnel tunnel;

  public DefaultRpm(Tunnel tunnel) {
    this.tunnel = tunnel;
  }

  public void excute() {
    tunnel.runAtVelocity(RPM.of(0));
  }

  public boolean isFinished() {
    return tunnel.getVelocity() == (RPM.of(0));
  }
}
