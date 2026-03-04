/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Shooter;
import java.util.function.Supplier;

public class SetShooterVelocity extends Command {

  Shooter shooter;
  Supplier<AngularVelocity> rpmSupplier;

  public SetShooterVelocity(Shooter shooter, Supplier<AngularVelocity> rpmSupplier) {
    this.shooter = shooter;
    this.rpmSupplier = rpmSupplier;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setVelocity(RPM.of(0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(Volts.of(0));
  }
}
