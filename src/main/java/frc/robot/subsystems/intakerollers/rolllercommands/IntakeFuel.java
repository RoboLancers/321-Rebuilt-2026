/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakerollers.IntakeRollerConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class IntakeFuel extends Command {

  IntakeRollers intakeRollers;

  public IntakeFuel(IntakeRollers intakeRollers) {
    this.intakeRollers = intakeRollers;
  }

  public void execute() {
    intakeRollers.setVoltage(IntakeRollerConstants.kIntakeFuelVelocity);
  }

  public boolean isFinished() {
    return false;
  }
}
