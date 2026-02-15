/* (C) RoboLancers 2026 */
package frc.robot.subsystems.indexer.indexerCommands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import java.util.function.Supplier;

public class Index {

  public static Command setVoltage(Indexer indexer, Supplier<Voltage> voltage) {
    return Commands.run(() -> indexer.setVoltage(voltage.get()), indexer);
  }

  public static Command goToVelocity(Indexer indexer, Supplier<AngularVelocity> velocity) {
    return Commands.run(() -> indexer.goToVelocity(velocity.get()), indexer);
  }

  public static Command index(Indexer indexer) {
    return goToVelocity(indexer, () -> IndexerConstants.kIndexVelocity);
  }

  public static Command release(Indexer indexer) {
    return goToVelocity(indexer, () -> IndexerConstants.kReleaseVelocity);
  }
}
