package frc.robot.subsystems.climb.climbCommands;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class LockPivotClimb extends Command{

    Climb climb;
    TalonFX climbMotor;

    public LockPivotClimb(Climb climb, TalonFX climbMotor){
        this.climb = climb;
        this.climbMotor = climbMotor;
    }
    
    public void execute(){
        climbMotor.setVoltage(0);
    }

    public boolean isFinished(){
        return climb.atPivotClimbVoltage(Volts.of(0));
    }
}
