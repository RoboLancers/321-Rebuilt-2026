/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;

public class Release extends Command {

  Tunnel tunnel;
  Shooter shooter;
  Indexer indexer;

  public Release(Tunnel tunnel, Shooter shoote, Indexer indexer) {
    this.tunnel = tunnel;
    this.shooter = shooter;
    this.indexer = indexer;

    addRequirements(tunnel, shooter, indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    tunnel.runAtVelocity(TunnelConstants.kPassFuelRPM);
    shooter.setVelocity(OuttakeConstants.kReleaseRPM);
    indexer.goToVelocity(IndexerConstants.kIndexVelocity);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(Volts.of(0));
    tunnel.runAtVelocity(RPM.of(0));
    indexer.setVoltage(Volts.of(0));
  }
}
