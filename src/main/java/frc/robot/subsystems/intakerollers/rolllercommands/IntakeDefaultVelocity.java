/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class IntakeDefaultVelocity extends Command {

  IntakeRollers intakeRollers;

  public IntakeDefaultVelocity(IntakeRollers intakeRollers) {
    this.intakeRollers = intakeRollers;
  }

  public void execute() {
    intakeRollers.setVelocity(RPM.of(0));
  }

  public boolean isFinished() {
    return intakeRollers.getRollerVelocity() == (RPM.of(0));
  }
}
