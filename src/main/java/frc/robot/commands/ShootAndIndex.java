/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;
import java.util.function.Supplier;

public class ShootAndIndex extends Command {

  Tunnel tunnel;
  Shooter shooter;
  Hood hood;
  Indexer indexer;
  Supplier<Distance> hubDistanceSupplier;

  public ShootAndIndex(
      Tunnel tunnel, Shooter shooter, Hood hood, Indexer indexer, Supplier<Distance> hubDistanceSupplier) {
    this.tunnel = tunnel;
    this.shooter = shooter;
    this.hood = hood;
    this.indexer = indexer;
    this.hubDistanceSupplier = hubDistanceSupplier;

    addRequirements(tunnel, shooter, hood, indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
    Distance hubDistance = hubDistanceSupplier.get();
    shooter.setVelocity(shooter.getScoreVelocity(hubDistance));
    hood.goToAngle(hood.getScoreAngle(hubDistance));

    if(Math.abs(shooter.getTopVelocity().in(RPM) - shooter.getScoreVelocity(hubDistance).in(RPM)) < 25){
     tunnel.runAtVelocity(TunnelConstants.kPassFuelRPM);
    indexer.goToVelocity(IndexerConstants.kIndexVelocity);}
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
