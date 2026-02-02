/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakerollers.IntakeRollerConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class OuttakeFuelCommand extends Command {
  IntakeRollers intakeRollers;

  public OuttakeFuelCommand(IntakeRollers intakeRollers) {
    this.intakeRollers = intakeRollers;
  }

  public void initialize() {}

  public void execute() {
    intakeRollers.setVoltage(IntakeRollerConstants.kOuttakeFuelVelocity);
  }

  public boolean isFinished() {
    return false;
  }
}
