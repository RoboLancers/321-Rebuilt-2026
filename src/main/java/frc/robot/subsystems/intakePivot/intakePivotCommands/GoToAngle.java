/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class GoToAngle extends Command{

  IntakePivot intakePivot;
  Angle angle;

  public GoToAngle(IntakePivot intakePivot, Angle angle) {
    this.intakePivot = intakePivot;
    this.angle = angle;
  }

  public void execute() {
    intakePivot.goToAngle(angle);
  }

  public boolean isFinished() {
    return intakePivot.getAngle() == angle;
  }
}
