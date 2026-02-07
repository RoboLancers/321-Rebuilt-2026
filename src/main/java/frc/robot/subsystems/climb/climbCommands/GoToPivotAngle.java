/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb.climbCommands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class GoToPivotAngle extends Command {

  Climb climb;
  Angle angle;

  public GoToPivotAngle(Climb climb, Angle angle) {
    this.climb = climb;
    this.angle = angle;
  }

  public void execute() {
    climb.goToPivotAngle(angle);
  }

  public boolean isFinished() {
    return climb.atPivotTargetAngle();
  }
}
