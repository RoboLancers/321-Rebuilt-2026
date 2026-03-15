/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood.hoodCommands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;

public class HomeHood extends Command {

  Hood hood;

  public HomeHood(Hood hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    hood.setTargetAngle(HoodConstants.kStartingAngle);
  }

  @Override
  public void execute() {
    hood.runVolts(HoodConstants.kHomingVoltage);
  }

  @Override
  public boolean isFinished() {
    return hood.atHomedPosition();
  }

  @Override
  public void end(boolean interrupted) {
    hood.zeroEncoder();
    hood.runVolts(Volts.of(0));
  }
}
