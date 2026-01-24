/* (C) RoboLancers 2026 */
package frc.robot.subsystems.indexer.indexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;

public class Tune {

  public static Command tune(Indexer indexer) {
    return Commands.run(() -> indexer.tune());
  }
}
