/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import java.util.function.Supplier;

public class SetIntakeVelocity extends Command {

  IntakeRollers intakeRollers;
  IntakePivot intakePivot;
  Supplier<AngularVelocity> velocitySupplier;

  public SetIntakeVelocity(
      IntakeRollers intakeRollers,
      IntakePivot intakePivot,
      Supplier<AngularVelocity> velocitySupplier) {
    this.intakeRollers = intakeRollers;
    this.intakePivot = intakePivot;
    this.velocitySupplier = velocitySupplier;
    addRequirements(intakeRollers);
  }

  @Override
  public void execute() {
    if (intakePivot.getAngle().in(Degrees) <= 30) {
      intakeRollers.setTargetVelocity(velocitySupplier.get());
      intakeRollers.goToVelocity(velocitySupplier.get());
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
