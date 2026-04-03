/* (C) RoboLancers 2026 */
package frc.robot.subsystems.outtake.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.util.RebuiltUtil;
import java.util.function.Supplier;

public class ShooterDefaultBehavior extends Command {

  Shooter shooter;
  Supplier<Pose2d> robotPoseSupplier;

  public ShooterDefaultBehavior(Shooter shooter, Supplier<Pose2d> robotPoseSupplier) {
    this.shooter = shooter;
    this.robotPoseSupplier = robotPoseSupplier;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Pose2d robotPose = robotPoseSupplier.get();
    if (RebuiltUtil.inAllianceZone(robotPose)) {
      shooter.setTargetVelocity(OuttakeConstants.kAllianceZoneDefaultRPM);
      shooter.goToVelocity(OuttakeConstants.kAllianceZoneDefaultRPM);
    } else {
      shooter.setVoltage(Volts.of(0));
      shooter.setTargetVelocity(RPM.of(0));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setTargetVelocity(RPM.of(0));
    shooter.setVoltage(Volts.of(0));
  }
}
