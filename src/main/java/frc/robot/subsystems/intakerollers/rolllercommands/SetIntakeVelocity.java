/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class SetIntakeVelocity extends Command {

  IntakeRollers intakeRollers;
  Supplier<AngularVelocity> velocitySupplier;

  public SetIntakeVelocity(IntakeRollers intakeRollers, Supplier<AngularVelocity> velocitySupplier) {
    this.intakeRollers = intakeRollers;
    this.velocitySupplier = velocitySupplier;
    addRequirements(intakeRollers);
  }

  @Override
  public void initialize(){
    intakeRollers.setTargetVelocity(velocitySupplier.get());
  }

  @Override
  public void execute() {
    intakeRollers.goToVelocity(velocitySupplier.get());
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
