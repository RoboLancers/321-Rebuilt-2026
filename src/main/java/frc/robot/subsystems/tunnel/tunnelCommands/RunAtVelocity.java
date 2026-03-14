/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel.tunnelCommands;

import static edu.wpi.first.units.Units.Volts;

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
    addRequirements(tunnel);
  }

  @Override
  public void execute() {
    tunnel.runAtVelocity(velocity.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    tunnel.setVoltage(Volts.of(0));
  }
}
