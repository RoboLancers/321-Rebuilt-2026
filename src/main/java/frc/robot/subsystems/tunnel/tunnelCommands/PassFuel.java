package frc.robot.subsystems.tunnel.tunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;

public class PassFuel extends Command{
    
Tunnel tunnel;

  public PassFuel(Tunnel tunnel) {
    this.tunnel = tunnel;
  }

  public void init(){}

  public void execute() {
    tunnel.runAtVelocity(TunnelConstants.kPassFuelRPM);
  }
}
