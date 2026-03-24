/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;

public class Feed extends Command {

  Tunnel tunnel;
  Shooter shooter;
  Hood hood;
  Indexer indexer;

  public Feed(Tunnel tunnel, Shooter shooter, Hood hood, Indexer indexer) {
    this.tunnel = tunnel;
    this.shooter = shooter;
    this.hood = hood;
    this.indexer = indexer;

    addRequirements(tunnel, shooter, hood, indexer);
  }

  @Override
  public void execute() {

    shooter.goToVelocity(OuttakeConstants.kNeutralFeedRPM);
    hood.goToAngle(HoodConstants.kNeutralFeedAngle);
    hood.setTargetAngle(HoodConstants.kNeutralFeedAngle);
    shooter.setTargetVelocity(OuttakeConstants.kNeutralFeedRPM);

    if (Math.abs(shooter.getTopVelocity().in(RPM) - OuttakeConstants.kNeutralFeedRPM.in(RPM))
        < 25) {
      indexer.setTargetVelocity(indexer.getOscillationVelocity());
      tunnel.setTargetVelocity(TunnelConstants.kPassFuelRPM);
      tunnel.goToVelocity(TunnelConstants.kPassFuelRPM);
      indexer.goToVelocity(indexer.getOscillationVelocity());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    hood.runVolts(Volts.of(0));
    shooter.setVoltage(Volts.of(0));
    tunnel.setVoltage(Volts.of(0));
    indexer.setVoltage(Volts.of(0));

    shooter.setTargetVelocity(RPM.of(0));
    tunnel.setTargetVelocity(RPM.of(0));
    indexer.setTargetVelocity(RPM.of(0));
  }
}
