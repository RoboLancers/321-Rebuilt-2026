/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.util.TunableConstant;

public class Tune extends Command {

  IntakePivot intakePivot;
  TunableConstant kP = new TunableConstant("/IntakePivot/kP", 0);
  TunableConstant kI = new TunableConstant("/IntakePivot/kI", 0);
  TunableConstant kD = new TunableConstant("/IntakePivot/kD", 0);
  TunableConstant kG = new TunableConstant("/IntakePivot/kG", 0);
  TunableConstant angle = new TunableConstant("/IntakePivot/angle", 2 * (Math.PI));
  TunableConstant acceleration = new TunableConstant("IntakePivot/acceleration", 0);
  TunableConstant velocity = new TunableConstant("IntakePivot/velocity", Math.PI / 2);

  public Tune(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void execute() {
    intakePivot.tune(
        kP.get(), kI.get(), kD.get(), kG.get(), angle.get(), acceleration.get(), velocity.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intakePivot.setVoltage(Volts.of(0));
  }
}
