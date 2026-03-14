/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.util.TunableConstant;

public class TuneOuttake extends Command {
  Shooter outtake;
  TunableConstant kP = new TunableConstant("/Outtake/kP", 0);
  TunableConstant kD = new TunableConstant("/Outtake/kD", 0);
  TunableConstant kV = new TunableConstant("/Outtake/kV", 0);
  TunableConstant targetRPM = new TunableConstant("/Outtake/targetRPM", 0);

  public TuneOuttake(Shooter outtake) {
    this.outtake = outtake;
    addRequirements(outtake);
  }

  @Override
  public void execute() {
    outtake.tune(kP.get(), kD.get(), kV.get(), targetRPM.get());
  }

  @Override
  public boolean isFinished(){
    return false;
  }

  @Override
  public void end(boolean interrupted){
    outtake.setVoltage(Volts.of(0));
  }
}
