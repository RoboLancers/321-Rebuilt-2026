/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood.hoodCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;

public class TuneHood {
  public static Command tune(Hood hood) {
    return Commands.run(() -> hood.tune());
  }
}
