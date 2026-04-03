/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;

public class StaticShoot extends Command {

  Hood hood;
  Tunnel tunnel;
  Shooter shooter;
  Indexer indexer;

  public StaticShoot(Tunnel tunnel, Shooter shooter, Indexer indexer, Hood hood) {
    this.hood = hood;
    this.tunnel = tunnel;
    this.shooter = shooter;
    this.indexer = indexer;

    addRequirements(tunnel, shooter, indexer);
  }

  @Override
  public void execute() {

    shooter.goToVelocity(OuttakeConstants.kStaticScoreRPM);
    shooter.setTargetVelocity(OuttakeConstants.kStaticScoreRPM);
    hood.goToAngle(Degrees.of(0));

    if (Math.abs(shooter.getTopVelocity().in(RPM) - OuttakeConstants.kStaticScoreRPM.in(RPM))
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
    shooter.setVoltage(Volts.of(0));
    tunnel.setVoltage(Volts.of(0));
    indexer.setVoltage(Volts.of(0));

    shooter.setTargetVelocity(RPM.of(0));
    tunnel.setTargetVelocity(RPM.of(0));
    indexer.setTargetVelocity(RPM.of(0));
  }
}
