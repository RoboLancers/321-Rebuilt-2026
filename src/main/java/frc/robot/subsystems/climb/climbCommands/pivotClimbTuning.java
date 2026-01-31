/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb.climbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class pivotClimbTuning extends Command {

  Climb climb;

  public pivotClimbTuning(Climb climb) {
    this.climb = climb;
  }

  public void execute() {
    climb.tunePivotClimb();
  }
}
