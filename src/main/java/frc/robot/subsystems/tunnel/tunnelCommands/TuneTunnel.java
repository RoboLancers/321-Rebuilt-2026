/* (C) RoboLancers 2026 */
package frc.robot.subsystems.tunnel.tunnelCommands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.util.TunableConstant;

public class TuneTunnel extends Command {

  Tunnel tunnel;

  TunableConstant kP = new TunableConstant("/Tunnel/kP", 0);

  TunableConstant kD = new TunableConstant("/Tunnel/kD", 0);

  TunableConstant kV = new TunableConstant("/Tunnel/kV", 0);

  TunableConstant targetVelocity = new TunableConstant("/Tunnel/targetVelocity", 0);

  public TuneTunnel(Tunnel tunnel) {
    this.tunnel = tunnel;
    addRequirements(tunnel);
  }

  @Override
  public void execute() {
    tunnel.tuneTunnel(kP.get(), kD.get(), kV.get(), targetVelocity.get());
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
