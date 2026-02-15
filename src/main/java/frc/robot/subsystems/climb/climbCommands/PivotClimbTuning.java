/* (C) RoboLancers 2026 */
package frc.robot.subsystems.climb.climbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import frc.robot.util.TunableConstant;

public class PivotClimbTuning extends Command {
  Climb climb;
  TunableConstant kPivotP = new TunableConstant("/Climb/kPivotP", 0);
  TunableConstant kPivotD = new TunableConstant("/Climb/kPivotD", 0);
  TunableConstant kPivotTargetAngle = new TunableConstant("/Climb/kPivotTargetAngle", 0);

  public PivotClimbTuning(Climb climb) {

    this.climb = climb;
  }

  public void execute() {
    climb.tunePivotClimb(kPivotP.get(), kPivotD.get(), kPivotTargetAngle.get());
  }

  public boolean isFinished() {
    return false;
  }
}
