/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;
import java.util.function.Supplier;

public class ShootToHub extends Command {

  Tunnel tunnel;
  Shooter shooter;
  Hood hood;
  Supplier<Distance> hubDistanceSupplier;

  public ShootToHub(
      Tunnel tunnel, Shooter shooter, Hood hood, Supplier<Distance> hubDistanceSupplier) {
    this.tunnel = tunnel;
    this.shooter = shooter;
    this.hood = hood;
    this.hubDistanceSupplier = hubDistanceSupplier;

    addRequirements(tunnel, shooter, hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Distance hubDistance = hubDistanceSupplier.get();
    tunnel.runAtVelocity(TunnelConstants.kPassFuelRPM);
    shooter.setVelocity(shooter.getScoreVelocity(hubDistance));
    hood.goToAngle(hood.getScoreAngle(hubDistance));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    hood.runVolts(Volts.of(0));
    shooter.setVelocity(RPM.of(0));
    tunnel.runAtVelocity(RPM.of(0));
  }
}
