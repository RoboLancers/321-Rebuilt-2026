/* (C) RoboLancers 2026 */
package frc.robot.commands.climbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class turnOffMagnet extends Command {

  Climb climb;

  public turnOffMagnet(Climb climb) {
    this.climb = climb;
  }

  public void execute() {
    climb.turnOffMagnet();
  }
}
