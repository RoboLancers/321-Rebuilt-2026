/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb.climbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class TurnOnMagnet extends Command {

  Climb climb;

  public TurnOnMagnet(Climb climb) {
    this.climb = climb;
  }

  public void execute() {
    climb.turnOnMagnet();
  }

  public boolean isFinished() {
    return false;
  }
}
