/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakerollers.rolllercommands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.util.TunableConstant;

public class IntakeRollerTune extends Command {
  IntakeRollers intakeRollers;
  TunableConstant kP = new TunableConstant("IntakeRollers/kP", 0);
  TunableConstant kD = new TunableConstant("IntakeRollers/kD", 0);
  TunableConstant kV = new TunableConstant("IntakeRollers/kV", 0);
  TunableConstant rollerTargetVelocityRPM =
      new TunableConstant("IntakeRollers/rollerTargetVelocityRPM", 0);

  public IntakeRollerTune(IntakeRollers intakeRollers) {
    this.intakeRollers = intakeRollers;
    addRequirements(intakeRollers);
  }

  @Override
  public void execute() {
    intakeRollers.tune(kP.get(), kD.get(), kV.get(), RPM.of(rollerTargetVelocityRPM.get()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted){
    intakeRollers.setVoltage(Volts.of(0));
  }
}
