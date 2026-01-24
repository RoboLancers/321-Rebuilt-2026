/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class IntakeRollerTune extends Command {
  IntakeRollers intakeRollers;

  public IntakeRollerTune(IntakeRollers intakeRollers) {
    this.intakeRollers = intakeRollers;
  }

  public void execute() {
    intakeRollers.tune();
  }

  public boolean isFinished() {
    return false;
  }
}
