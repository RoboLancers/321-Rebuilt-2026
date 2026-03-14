/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb.climbCommands;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class LockPivotClimb extends Command {

  Climb climb;

  public LockPivotClimb(Climb climb) {
    this.climb = climb;
    addRequirements(climb);
  }

  @Override
  public void execute() {
    climb.lockPivot();
  }

  @Override
  public boolean isFinished() {
    return climb.atPivotClimbVoltage(Volts.of(0));
  }

  @Override
  public void end(boolean interrupted){}
}
