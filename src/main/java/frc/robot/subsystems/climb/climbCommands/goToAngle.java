/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb.climbCommands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class goToAngle extends Command {

  Climb climb;

  public goToAngle(Climb climb) {
    this.climb = climb;
  }

  public void execute(Angle angle) {
    climb.goToAngle(angle);
  }
}