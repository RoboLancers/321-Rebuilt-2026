/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class JostleWithIntake extends Command {

  IntakePivot intakePivot;

  public JostleWithIntake(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void execute() {}
}
