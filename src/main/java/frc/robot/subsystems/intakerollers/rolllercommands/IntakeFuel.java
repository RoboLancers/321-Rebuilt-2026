/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollerConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class IntakeFuel extends Command {

  IntakeRollers intakeRollers;
  IntakePivot intakePivot;

  public IntakeFuel(IntakeRollers intakeRollers, IntakePivot intakePivot) {
    this.intakeRollers = intakeRollers;
    this.intakePivot = intakePivot;
    addRequirements(intakeRollers);
  }

  @Override
  public void initialize() {
    intakeRollers.setTargetVelocity(IntakeRollerConstants.kIntakeFuelVelocity);
  }

  @Override
  public void execute() {
    if (intakePivot.getAngle().in(Degrees) < 30) {
      intakeRollers.goToVelocity(IntakeRollerConstants.kIntakeFuelVelocity);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intakeRollers.setVoltage(Volts.of(0));
    intakeRollers.setTargetVelocity(RPM.of(0));
  }
}
