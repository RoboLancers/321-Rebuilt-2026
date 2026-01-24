package frc.robot.subsystems.intakerollers.rolllercommands;

import frc.robot.subsystems.intakerollers.IntakeRollers;

public class IntakeRollerTune {
    IntakeRollers intakeRollers;

    public IntakeRollerTune(IntakeRollers intakeRollers){
        this.intakeRollers = intakeRollers;
    }

    public void execute(){
        intakeRollers.tune();
    }
}
