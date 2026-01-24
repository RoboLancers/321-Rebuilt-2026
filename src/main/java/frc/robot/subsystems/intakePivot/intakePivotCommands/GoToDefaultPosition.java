/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakeConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class GoToDefaultPosition extends Command {

  IntakePivot intakePivot;

  public GoToDefaultPosition(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
  }

  public void execute() {
    intakePivot.goToAngle(IntakeConstants.kDefaultPosition);
  }

  public boolean isFinished() {
    return intakePivot.getAngle() == IntakeConstants.kDefaultPosition;
  }
}
