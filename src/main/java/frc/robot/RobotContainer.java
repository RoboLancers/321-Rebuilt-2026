/* (C) RoboLancers 2026 */
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Align;
import frc.robot.commands.Score;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.hoodCommands.HoodCommands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.indexerCommands.Index;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.intakePivotCommands.GoToDefaultPosition;
import frc.robot.subsystems.intakePivot.intakePivotCommands.GoToIntakePosition;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.rolllercommands.IntakeFuel;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.outtake.commands.ShootFuel;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.tunnelCommands.RunAtVelocity;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.RebuiltUtil;

public class RobotContainer {

  public Drivetrain drivetrain = Drivetrain.create();
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
