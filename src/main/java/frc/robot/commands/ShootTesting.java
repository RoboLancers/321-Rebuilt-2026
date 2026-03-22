/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;
import java.util.function.Supplier;

public class ShootTesting extends Command {

  Tunnel tunnel;
  Shooter shooter;
  Hood hood;
  Indexer indexer;
  Supplier<AngularVelocity> velocitySupplier;
  Supplier<Angle> angleSupplier;

  public ShootTesting(
      Tunnel tunnel,
      Shooter shooter,
      Hood hood,
      Indexer indexer,
      Supplier<AngularVelocity> velocitySupplier,
      Supplier<Angle> angleSupplier) {
    this.tunnel = tunnel;
    this.shooter = shooter;
    this.hood = hood;
    this.indexer = indexer;
    this.velocitySupplier = velocitySupplier;
    this.angleSupplier = angleSupplier;

    addRequirements(tunnel, shooter, hood, indexer);
  }

  @Override
  public void execute() {

    shooter.goToVelocity(velocitySupplier.get());
    hood.goToAngle(angleSupplier.get());
    hood.setTargetAngle(angleSupplier.get());
    shooter.setTargetVelocity(velocitySupplier.get());

    if (Math.abs(shooter.getTopVelocity().in(RPM) - velocitySupplier.get().in(RPM)) < 25) {
      tunnel.goToVelocity(TunnelConstants.kPassFuelRPM);
      indexer.setTargetVelocity(RPM.of(1400 + indexer.getOscillationVelocity().in(RPM)));
      indexer.goToVelocity(RPM.of(1400 + indexer.getOscillationVelocity().in(RPM)));
      tunnel.setTargetVelocity(TunnelConstants.kPassFuelRPM);
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
