/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;

public class ShootTest extends Command {

  Tunnel tunnel;
  Shooter shooter;
  Indexer indexer;

  public ShootTest(Tunnel tunnel, Shooter shooter, Indexer indexer) {
    this.tunnel = tunnel;
    this.shooter = shooter;
    this.indexer = indexer;
    addRequirements(tunnel, shooter, indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    shooter.setVelocity(RPM.of(1000));

    if (Math.abs(shooter.getTopVelocity().in(RPM) - 1000) < 25) {
      tunnel.runAtVelocity(TunnelConstants.kPassFuelRPM);
      indexer.goToVelocity(IndexerConstants.kIndexVelocity);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setVelocity(RPM.of(0));
    tunnel.runAtVelocity(RPM.of(0));
    indexer.setVoltage(Volts.of(0));
  }
}
