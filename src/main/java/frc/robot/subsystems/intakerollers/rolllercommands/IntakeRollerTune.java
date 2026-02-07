/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.util.TunableConstant;

public class IntakeRollerTune extends Command {
  IntakeRollers intakeRollers;
  TunableConstant kP = new TunableConstant("IntakeRollers/kP", 0);
  TunableConstant kD = new TunableConstant("IntakeRollers/kD", 0);
  TunableConstant kG = new TunableConstant("IntakeRollers/kG", 0);
  TunableConstant kV = new TunableConstant("IntakeRollers/kV", 0);

  public IntakeRollerTune(IntakeRollers intakeRollers) {
    this.intakeRollers = intakeRollers;
  }

  public void execute() {
    intakeRollers.tune(kP.get(), kD.get(), kV.get(), kG.get());
  }

  public boolean isFinished() {
    return false;
  }
}
