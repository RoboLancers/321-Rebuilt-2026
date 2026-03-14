/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb.climbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import frc.robot.util.TunableConstant;

public class ClimbTuning extends Command {
  Climb climb;
  TunableConstant kP = new TunableConstant("/Climb/kP", 0);
  TunableConstant kD = new TunableConstant("/Climb/kD", 0);
  TunableConstant kG = new TunableConstant("/Climb/kG", 0);
  TunableConstant kTargetAngle = new TunableConstant("/Climb/kTargetAngle", 0);

  public ClimbTuning(Climb climb) {
    this.climb = climb;
    addRequirements(climb);
  }

  @Override
  public void execute() {
    climb.tuneClimb(kP.get(), kD.get(), kG.get(), kTargetAngle.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
