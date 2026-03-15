/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;
import java.util.function.Supplier;

public class ShootTesting extends Command {

  Tunnel tunnel;
  Shooter shooter;
  Hood hood;
  Indexer indexer;
  Supplier<Distance> hubDistanceSupplier;
  Supplier<Angle> angleSupplier;

  public ShootTesting(
      Tunnel tunnel,
      Shooter shooter,
      Hood hood,
      Indexer indexer,
      Supplier<Distance> hubDistanceSupplier,
      Supplier<Angle> angleSupplier) {
    this.tunnel = tunnel;
    this.shooter = shooter;
    this.hood = hood;
    this.indexer = indexer;
    this.hubDistanceSupplier = hubDistanceSupplier;
    this.angleSupplier = angleSupplier;

    addRequirements(tunnel, shooter, hood, indexer);
  }

  Distance hubDistance = hubDistanceSupplier.get();

  @Override
  public void initialize() {
    hood.setTargetAngle(angleSupplier.get());
    shooter.setTargetVelocity(shooter.getScoreVelocity(hubDistance));
    indexer.setTargetVelocity(IndexerConstants.kIndexVelocity);
    tunnel.setTargetVelocity(TunnelConstants.kPassFuelRPM);
  }

  @Override
  public void execute() {

    shooter.goToVelocity(shooter.getScoreVelocity(hubDistance));
    hood.goToAngle(angleSupplier.get());

    if (Math.abs(shooter.getTopVelocity().in(RPM) - shooter.getScoreVelocity(hubDistance).in(RPM))
        < 25) {
      tunnel.goToVelocity(TunnelConstants.kPassFuelRPM);
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
    shooter.setVoltage(Volts.of(0));
    tunnel.setVoltage(Volts.of(0));
    indexer.setVoltage(Volts.of(0));

    shooter.setTargetVelocity(RPM.of(0));
    tunnel.setTargetVelocity(RPM.of(0));
    indexer.setTargetVelocity(RPM.of(0));
  }
}
