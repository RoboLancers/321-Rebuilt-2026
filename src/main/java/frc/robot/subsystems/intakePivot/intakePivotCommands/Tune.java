/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.util.TunableConstant;

public class Tune extends Command {

  IntakePivot intakePivot;
  TunableConstant kP = new TunableConstant("/IntakePivot/kP", 0);
  TunableConstant kD = new TunableConstant("/IntakePivot/kD", 0);
  TunableConstant kG = new TunableConstant("/IntakePivot/kG", 0);
  TunableConstant angle = new TunableConstant("/IntakePivot/angle", 0);

  public Tune(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
  }

  public void execute() {
    intakePivot.tune(kP.get(), kD.get(), kG.get(), angle.get());
  }
}
