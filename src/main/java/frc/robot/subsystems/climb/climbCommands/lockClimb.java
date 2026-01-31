package frc.robot.subsystems.climb.climbCommands;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class lockClimb extends Command {

  Climb climb;
  TalonFX climbMotor;

  public lockClimb(Climb climb, TalonFX climbMotor) {

    this.climbMotor = climbMotor;
  }

  public void execute() {

    climbMotor.setVoltage(0);
  }
}