/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakerollers.IntakeRollerConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class IntakeFuel extends Command {

  IntakeRollers intakeRollers;

  public IntakeFuel(IntakeRollers intakeRollers) {
    this.intakeRollers = intakeRollers;
    addRequirements(intakeRollers);
  }

  @Override
  public void execute() {
    intakeRollers.setVelocity(IntakeRollerConstants.kIntakeFuelVelocity);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intakeRollers.setVoltage(Volts.of(0));
  }
}
