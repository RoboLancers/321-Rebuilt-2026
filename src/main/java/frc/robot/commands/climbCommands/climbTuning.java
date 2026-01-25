/* (C) RoboLancers 2026 */
package frc.robot.commands.climbCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.climb.Climb;
import frc.robot.util.TunableConstant;

public class climbTuning {

  Climb climb;
  ArmFeedforward armFeedforward;
  PIDController climbController;

  TunableConstant kP = new TunableConstant("/Climb/kP", 0);
  TunableConstant kD = new TunableConstant("/Climb/kD", 0);
  TunableConstant kG = new TunableConstant("/Climb/kG", 0);
  TunableConstant targetAngle = new TunableConstant("/Climb/targetAngle", 0);

  public climbTuning(
      Climb climb,
      ArmFeedforward armFeedforward,
      PIDController climbController,
      TunableConstant kP,
      TunableConstant kD,
      TunableConstant kG,
      TunableConstant targetAngle) {

    this.climb = climb;
    this.armFeedforward = armFeedforward;
    this.climbController = climbController;
    this.kP = kP;
    this.kD = kD;
    this.kG = kG;
    this.targetAngle = targetAngle;
  }

  public void execute() {
    climbController.setPID(kP.get(), 0, kD.get());
    armFeedforward.setKg(kG.get());
    climb.goToAngle(Degrees.of(targetAngle.get()));
  }
}
