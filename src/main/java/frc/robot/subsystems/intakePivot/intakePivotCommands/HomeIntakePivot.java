/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakeConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class HomeIntakePivot extends Command {

  IntakePivot intakePivot;

  public HomeIntakePivot(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void execute() {
    intakePivot.setVoltage(IntakeConstants.kHomingVoltage);
  }

  @Override
  public boolean isFinished() {
    return intakePivot.atHomedPosition();
  }

  @Override
  public void end(boolean interrupted) {
    intakePivot.setVoltage(Volts.of(0));
  }
}
