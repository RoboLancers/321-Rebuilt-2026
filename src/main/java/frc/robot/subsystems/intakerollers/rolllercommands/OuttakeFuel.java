package frc.robot.subsystems.intakerollers.rolllercommands;

import frc.robot.subsystems.intakerollers.IntakeRollerConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class OuttakeFuel {
    IntakeRollers intakeRollers;

    public OuttakeFuel(IntakeRollers intakeRollers){
        this.intakeRollers = intakeRollers;
    }

    public void initialize(){
        }

    public void execute(){
        intakeRollers.setVoltage(IntakeRollerConstants.kOuttakeFuelVelocity);
    }

    public boolean isFinished(){
        return false;
    }
}
