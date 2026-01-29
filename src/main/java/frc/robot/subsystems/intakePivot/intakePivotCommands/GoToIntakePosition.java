package frc.robot.subsystems.intakePivot.intakePivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakeConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class GoToIntakePosition extends Command{

IntakePivot intakePivotMainMotor;

    public GoToIntakePosition(IntakePivot intakePivotMainMotor){
        this.intakePivotMainMotor = intakePivotMainMotor;

    }

    public void init(){

    }

    public void execute(){
        intakePivotMainMotor.goToAngle(IntakeConstants.kIntakePosition);
    }

    public boolean isFinished(){
        return intakePivotMainMotor.getAngle() == IntakeConstants.kIntakePosition;
    }
}