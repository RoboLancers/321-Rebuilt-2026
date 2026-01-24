package frc.robot.commands.climbCommands;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class goToAngle extends Command{

    Climb climb;

    public goToAngle(Climb climb, TalonFX climbMotor){
        this.climb = climb;
    }
    
    public void execute(Angle angle){
        climb.goToAngle(angle);
    }
}
