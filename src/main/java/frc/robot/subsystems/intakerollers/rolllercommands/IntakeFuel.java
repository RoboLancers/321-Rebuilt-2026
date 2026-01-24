package frc.robot.subsystems.intakerollers.rolllercommands;

import frc.robot.subsystems.intakerollers.IntakeRollerConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class IntakeFuel {
    
    IntakeRollers intakeRollers;

    public IntakeFuel(IntakeRollers intakeRollers){
        this.intakeRollers = intakeRollers;
    }

    public void execute(){
        intakeRollers.setVoltage(IntakeRollerConstants.kIntakeFuelVelocity);
    }

    public boolean isFinished(){
        return false;
    }
}
