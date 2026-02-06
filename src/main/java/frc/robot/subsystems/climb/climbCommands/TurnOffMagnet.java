/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb.climbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class TurnOffMagnet extends Command {

  Climb climb;

  public TurnOffMagnet(Climb climb) {
    this.climb = climb;
  }

  public void execute() {
    climb.turnOffMagnet();
  }

  public boolean isFinished() {
    return false;
  }
}
