/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel.tunnelCommands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tunnel.Tunnel;
import java.util.function.Supplier;

public class RunAtVelocity extends Command {

  Tunnel tunnel;
  Supplier<AngularVelocity> velocity;

  public RunAtVelocity(Tunnel tunnel, Supplier<AngularVelocity> velocity) {
    this.tunnel = tunnel;
    this.velocity = velocity;
  }

  public void init() {}

  public void execute() {
    tunnel.runAtVelocity(velocity.get());
  }

  public boolean isFinished() {
    return false;
  }
}
