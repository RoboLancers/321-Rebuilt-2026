/* (C) RoboLancers 2026 */
package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.hoodCommands.HomeHood;
import frc.robot.subsystems.questNav.QuestNavSubsystem;

@Logged
public class Robot extends TimedRobot {
  @NotLogged private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final Drivetrain drivetrain = Drivetrain.create();

  private final QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem(drivetrain);

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (m_robotContainer != null) {
      if (m_robotContainer.drivetrain != null) {
        m_robotContainer.drivetrain.driveTrainPeriodic();
      }
    }

    m_robotContainer.latestPoseField.setRobotPose(
        questNavSubsystem.getLatestQuestPose().toPose2d());
    SmartDashboard.putData("latest 2d pose", m_robotContainer.latestPoseField);

    questNavSubsystem.questPeriodic();
    questNavSubsystem.resetQuestPose3d(m_robotContainer.getLatestCameraPose());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    CommandScheduler.getInstance().schedule((new HomeHood(m_robotContainer.hood)));
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    questNavSubsystem.resetQuestPose2d(drivetrain.getPose());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().schedule(new HomeHood(m_robotContainer.hood));
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
