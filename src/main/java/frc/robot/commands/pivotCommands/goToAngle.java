package frc.robot.commands.pivotCommands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class goToAngle extends Command{
    private Pivot pivot;

    public goToAngle(Pivot pivot, TalonFX pivotMotor){
        this.pivot = pivot;
    }
    
    //in progress, needs vision to get a targetAngle to the hub
    public void execute(){
        pivot.goToAngle(null);
    }

    public boolean isFinished(){
        return true;
    }

    public void end(){

    }

}
