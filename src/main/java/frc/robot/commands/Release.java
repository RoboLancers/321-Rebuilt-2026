/* (C) RoboLancers 2026 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelConstants;

public class Release extends Command {

  Tunnel tunnel;
  Shooter shooter;
  Hood hood;

  public Release(Tunnel tunnel, Shooter shooter, Hood hood) {
    this.tunnel = tunnel;
    this.shooter = shooter;
    this.hood = hood;

    addRequirements(tunnel, shooter, hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    tunnel.runAtVelocity(TunnelConstants.kPassFuelRPM);
    shooter.setVelocity(OuttakeConstants.kReleaseRPM);
    hood.goToAngle(HoodConstants.kStartingAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    hood.runVolts(Volts.of(0));
    shooter.setVoltage(Volts.of(0));
    tunnel.runAtVelocity(RPM.of(0));
  }
}
