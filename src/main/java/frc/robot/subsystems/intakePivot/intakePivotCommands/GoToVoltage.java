/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;
import java.util.function.Supplier;

public class GoToVoltage extends Command {

  IntakePivot intakePivot;
  Supplier<Voltage> voltageSupplier;

  public GoToVoltage(IntakePivot intakePivot, Supplier<Voltage> voltageSupplier) {
    this.intakePivot = intakePivot;
    this.voltageSupplier = voltageSupplier;
    addRequirements(intakePivot);
  }

  @Override
  public void execute() {
    intakePivot.setVoltage(voltageSupplier.get());
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
