package frc.robot.subsystems.tunnel.tunnelCommands;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.tunnel.Tunnel;

public class RunAtVelocity {

    Tunnel tunnel;

    public RunAtVelocity(Tunnel tunnel){
        this.tunnel = tunnel;
    }
    
    public void execute(Supplier<AngularVelocity> velocity){
        tunnel.runAtVelocity(velocity.get());
    }

}
