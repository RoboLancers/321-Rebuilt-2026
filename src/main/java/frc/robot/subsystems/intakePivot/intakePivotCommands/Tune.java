/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot.intakePivotCommands;

import frc.robot.subsystems.intakePivot.IntakePivot;

public class Tune {

  IntakePivot intakePivot;

  public Tune(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
  }

  public void execute() {
    intakePivot.tune();
  }
}
