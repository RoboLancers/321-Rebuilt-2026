/* (C) RoboLancers 2026 */
package frc.robot.commands.hoodCommands;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;

public class goToAngle extends Command {
  private Hood hood;

  public goToAngle(Hood hood, TalonFX pivotMotor) {
    this.hood = hood;
  }

  // in progress, needs vision to get a targetAngle to the hub
  public void execute() {
    hood.goToAngle(null);
  }

  public boolean isFinished() {
    return true;
  }

  public void end() {}
}
