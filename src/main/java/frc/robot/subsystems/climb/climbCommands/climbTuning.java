/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb.climbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class climbTuning extends Command {

  Climb climb;

  public climbTuning(Climb climb) {
    this.climb = climb;
  }

  public void execute() {
    climb.tuneClimb();
  }
}
