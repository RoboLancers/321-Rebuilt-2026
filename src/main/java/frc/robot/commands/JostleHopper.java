package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainLogger;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class JostleHopper extends Command{
    
IntakePivot intakePivot;
Drivetrain drivetrain;

    public JostleHopper(IntakePivot intakePivot, Drivetrain drivetrain){
        this.intakePivot = intakePivot;
        this.drivetrain = drivetrain;
        addRequirements(intakePivot, drivetrain);
    }

@Override
public void execute(){
    intakePivot
}
}
