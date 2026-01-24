package frc.robot.subsystems.hood.hoodCommands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;

public class GoToAngle extends Command{
    private Hood hood;

    public GoToAngle(Hood hood, TalonFX pivotMotor){
        this.hood = hood;
    }
    
    //in progress, needs vision to get a targetAngle to the hub
    public void execute(){
        hood.goToAngle(null);
    }

    public boolean isFinished(){
        return true;
    }

    public void end(){

    }

}
