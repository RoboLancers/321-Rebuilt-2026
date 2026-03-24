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
    addRequirements(climb);
  }

  @Override
  public void execute() {
    climb.goToPivotAngle(angle);
  }

  @Override
  public boolean isFinished() {
    return climb.atPivotTargetAngle();
  }

  @Override
  public void end(boolean interrupted) {
    climb.lockPivot();
  }
}
