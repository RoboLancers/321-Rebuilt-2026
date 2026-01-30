/* (C) RoboLancers 2026 */
package frc.robot.commands.climbCommands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class goToPivotAngle extends Command {

  Climb climb;

  public goToPivotAngle(Climb climb) {
    this.climb = climb;
  }

  public void execute(Angle angle) {
    climb.goToPivotAngle(angle);
  }
}
