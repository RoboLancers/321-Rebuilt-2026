/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakeConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class JostleWithIntake extends Command{

IntakePivot intakePivot;
Angle setpoint = IntakeConstants.kDefaultPosition;
double jostleVoltage = 0;

  public JostleWithIntake(IntakePivot intakePivot){
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

@Override
public void execute(){
    if (setpoint == IntakeConstants.kDefaultPosition){
      jostleVoltage = IntakeConstants.kJostleVoltage;
    } else {jostleVoltage = -IntakeConstants.kJostleVoltage;}

    if(!intakePivot.atAngle(setpoint)){
      intakePivot.setVoltage(Volts.of(jostleVoltage));
    } else {
      if (setpoint == IntakeConstants.kDefaultPosition){
        setpoint = IntakeConstants.kIntakePosition;
      } else {
        setpoint = IntakeConstants.kDefaultPosition;
      }
    }
}

@Override
public boolean isFinished(){
  return false;
}

@Override
public void end(boolean interrupted){
  intakePivot.setVoltage(Volts.of(0));
}
}
