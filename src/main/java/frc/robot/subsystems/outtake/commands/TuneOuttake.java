/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Shooter;

public class TuneOuttake {

  public static Command tune(Shooter outtake) {
    return outtake.tune();
  }
}
