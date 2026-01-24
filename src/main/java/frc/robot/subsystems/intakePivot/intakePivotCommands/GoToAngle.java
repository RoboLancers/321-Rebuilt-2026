/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class GoToAngle {

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