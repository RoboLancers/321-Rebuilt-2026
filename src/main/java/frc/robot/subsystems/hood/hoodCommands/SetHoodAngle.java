/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood.hoodCommands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import java.util.function.Supplier;

public class SetHoodAngle extends Command {

  Hood hood;
  Supplier<Angle> angleSupplier;

  public SetHoodAngle(Hood hood, Supplier<Angle> angleSupplier) {
    this.hood = hood;
    this.angleSupplier = angleSupplier;
    addRequirements(hood);
  }

  @Override
  public void execute() {
    hood.goToAngle(angleSupplier.get());
  }

  @Override
  public boolean isFinished() {
    return hood.atAngle(angleSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    hood.runVolts(Volts.of(0));
  }
}
