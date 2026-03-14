/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood.hoodCommands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.TunableConstant;

public class TuneHood extends Command {
  Hood hood;
  TunableConstant kP = new TunableConstant("Hood/kP/", 0);
  TunableConstant kD = new TunableConstant("Hood/kD/", 0);
  TunableConstant kG = new TunableConstant("Hood/kG/", 0);
  TunableConstant targetAngle = new TunableConstant("Hood/targetAngle/", 0);

  public TuneHood(Hood hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void execute() {
    hood.tune(kP.get(), kD.get(), kG.get(), targetAngle.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted){
    hood.runVolts(Volts.of(0));
  }
}
