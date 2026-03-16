/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;
import java.util.function.Supplier;

public class GoToAngle extends Command {

  IntakePivot intakePivot;
  Supplier<Angle> angleSupplier;

  public GoToAngle(IntakePivot intakePivot, Supplier<Angle> angleSupplier) {
    this.intakePivot = intakePivot;
    this.angleSupplier = angleSupplier;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    intakePivot.setTargetAngle(angleSupplier.get());
  }

  @Override
  public void execute() {
    intakePivot.setTargetAngle(angleSupplier.get());
    intakePivot.goToAngle(angleSupplier.get());
  }

  @Override
  public boolean isFinished() {
    return intakePivot.atAngle(angleSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    intakePivot.setVoltage(Volts.of(0));
  }
}
