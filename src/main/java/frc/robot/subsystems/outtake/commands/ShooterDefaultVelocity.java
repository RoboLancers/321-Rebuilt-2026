/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Shooter;

public class ShooterDefaultVelocity extends Command {

  Shooter shooter;

  public ShooterDefaultVelocity(Shooter shooter) {
    this.shooter = shooter;
  }

  public void execute() {
    shooter.setVelocity(RPM.of(0));
  }

  public boolean isFinished() {
    return shooter.getBottomVelocity() == (RPM.of(0)) && shooter.getTopVelocity() == (RPM.of(0));
  }
}
