/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakeConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.util.TunableConstant;

public class Tune extends Command {

  IntakePivot intakePivot;
  TunableConstant kP = new TunableConstant("/IntakePivot/kP", IntakeConstants.kP);
  TunableConstant kI = new TunableConstant("/IntakePivot/kI", IntakeConstants.kI);
  TunableConstant kD = new TunableConstant("/IntakePivot/kD", IntakeConstants.kD);
  TunableConstant kG = new TunableConstant("/IntakePivot/kG", IntakeConstants.kG);
  TunableConstant angle = new TunableConstant("/IntakePivot/angle", 0);
  TunableConstant acceleration =
      new TunableConstant(
          "IntakePivot/acceleration",
          IntakeConstants.kMaxAcceleration.in(RadiansPerSecondPerSecond));
  TunableConstant velocity =
      new TunableConstant(
          "IntakePivot/velocity", IntakeConstants.kMaxVelocity.in(RadiansPerSecond));

  public Tune(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void execute() {
    intakePivot.tune(
        kP.get(), kI.get(), kD.get(), kG.get(), angle.get(), velocity.get(), acceleration.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intakePivot.setVoltage(Volts.of(0));
  }
}
