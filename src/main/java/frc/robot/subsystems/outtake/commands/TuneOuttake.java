/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;

public class TuneOuttake {

  public static Command tune(Outtake outtake) {
    return outtake.tune();
  }
}
