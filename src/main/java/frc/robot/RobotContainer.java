/* (C) RoboLancers 2026 */
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.rolllercommands.IntakeFuel;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.outtake.commands.ShootFuel;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  private final IntakeRollers intakeRollers = new IntakeRollers();
  private final IntakeFuel intakeFuel = new IntakeFuel(intakeRollers);
  private final Shooter shooter = new Shooter();
  private final ShootFuel shootFuel = new ShootFuel();

  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("IntakeFuel", intakeFuel);
    NamedCommands.registerCommand("ShootFuel", shootFuel.releaseFuel(shooter));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
