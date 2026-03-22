package frc.robot.subsystems.intakePivot.intakePivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class ZeroPosition extends Command{
 
    IntakePivot intakePivot;

    public ZeroPosition(IntakePivot intakePivot){
        this.intakePivot = intakePivot;
        addRequirements(intakePivot);
    }

    @Override
    public void initialize(){
        intakePivot.zero();
    }

}
