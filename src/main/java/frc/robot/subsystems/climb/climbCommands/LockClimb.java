/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb.climbCommands;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class LockClimb extends Command {

  Climb climb;

  public LockClimb(Climb climb) {
    this.climb = climb;
    addRequirements(climb);
  }

  @Override
  public void execute() {
    climb.lockClimb();
  }

  @Override
  public boolean isFinished() {
    return climb.atClimbVoltage(Volts.of(0));
  }

  @Override
  public void end(boolean interrupted){}
}
