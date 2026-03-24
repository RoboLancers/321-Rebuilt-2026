/* (C) RoboLancers 2026 */
package frc.robot.subsystems.indexer.indexerCommands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import java.util.function.Supplier;

public class SetIndexerVelocity extends Command {

  Indexer indexer;
  Supplier<AngularVelocity> rpmSupplier;

  public SetIndexerVelocity(Indexer indexer, Supplier<AngularVelocity> rpmSupplier) {
    this.indexer = indexer;
    this.rpmSupplier = rpmSupplier;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    indexer.setTargetVelocity(rpmSupplier.get());
  }

  @Override
  public void execute() {
    indexer.setTargetVelocity(rpmSupplier.get());
    indexer.goToVelocity(rpmSupplier.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setVoltage(Volts.of(0));
    indexer.setTargetVelocity(RPM.of(0));
  }
}
