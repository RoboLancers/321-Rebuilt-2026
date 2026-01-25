/* (C) RoboLancers 2026 */
package frc.robot.commands.climbCommands;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class setVoltageWithFeedForward extends Command {

  Climb climb;
  TalonFX climbMotor;
  ArmFeedforward armFeedforward;

  public setVoltageWithFeedForward(TalonFX climbMotor, ArmFeedforward armFeedforward) {
    this.climbMotor = climbMotor;
    this.armFeedforward = armFeedforward;
  }

  public void execute(double Volts /*, Drive drive*/) {

    double feedforward =
        armFeedforward.calculate(0 /*replace with the pitch from the gyro in radian*/, 0);
    climbMotor.setVoltage(Volts + feedforward);
  }
}
