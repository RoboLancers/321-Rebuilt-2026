/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakerollers.IntakeRollerConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import java.util.function.Supplier;

public class IntakeFuelAlt extends Command {

  IntakeRollers intakeRollers;
  Supplier<Angle> pivotAngleSupplier;

  public IntakeFuelAlt(IntakeRollers intakeRollers, Supplier<Angle> pivotAngleSupplier) {
    this.intakeRollers = intakeRollers;
    this.pivotAngleSupplier = pivotAngleSupplier;
    addRequirements(intakeRollers);
  }

  @Override
  public void execute() {
    if (pivotAngleSupplier.get().in(Degrees) < 30) {
      intakeRollers.setTargetVelocity(IntakeRollerConstants.kIntakeFuelVelocity);
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
