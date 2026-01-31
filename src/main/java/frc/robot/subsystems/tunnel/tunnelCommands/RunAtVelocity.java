/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel.tunnelCommands;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.tunnel.Tunnel;
import java.util.function.Supplier;

public class RunAtVelocity {

  Tunnel tunnel;

  public RunAtVelocity(Tunnel tunnel) {
    this.tunnel = tunnel;
  }

  public void execute(Supplier<AngularVelocity> velocity) {
    tunnel.runAtVelocity(velocity.get());
  }
}
