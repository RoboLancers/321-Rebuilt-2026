/* (C) RoboLancers 2026 */
package frc.robot.subsystems.indexer.indexerCommands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerDefaultVelocity extends Command {

  Indexer indexer;

  public IndexerDefaultVelocity(Indexer indexer) {
    this.indexer = indexer;
  }

  public void execute() {
    indexer.goToVelocity(RPM.of(0));
  }

  public boolean isFinished() {
    return indexer.getVelocity() == (RPM.of(0));
  }
}
