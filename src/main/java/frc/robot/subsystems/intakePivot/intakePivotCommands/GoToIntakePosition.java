/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakeConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class GoToIntakePosition extends Command {

  IntakePivot intakePivot;

  public GoToIntakePosition(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
  }

  public void init() {}

  public void execute() {
    intakePivot.goToAngle(IntakeConstants.kIntakePosition);
  }

  public boolean isFinished() {
    return intakePivot.getAngle() == IntakeConstants.kIntakePosition;
  }
}
