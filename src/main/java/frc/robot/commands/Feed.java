/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;
import java.util.function.Supplier;

public class Feed extends Command {

  Tunnel tunnel;
  Shooter shooter;
  Hood hood;
  Indexer indexer;

  public Feed(
      Tunnel tunnel,
      Shooter shooter,
      Hood hood,
      Indexer indexer) {
    this.tunnel = tunnel;
    this.shooter = shooter;
    this.hood = hood;
    this.indexer = indexer;

    addRequirements(tunnel, shooter, hood, indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    shooter.setVelocity(OuttakeConstants.kNeutralFeedRPM);
    hood.goToAngle(HoodConstants.kNeutralFeedAngle);

    if (Math.abs(shooter.getTopVelocity().in(RPM) - OuttakeConstants.kNeutralFeedRPM.in(RPM))
        < 25) {
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
    hood.runVolts(Volts.of(0));
    shooter.setVelocity(RPM.of(0));
    tunnel.runAtVelocity(RPM.of(0));
    indexer.setVoltage(Volts.of(0));
  }
}