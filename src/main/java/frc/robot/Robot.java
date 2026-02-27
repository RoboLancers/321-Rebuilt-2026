/* (C) RoboLancers 2026 */
package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private String autoSelected;
  private SendableChooser<String> chooser = new SendableChooser<>();
  private Command m_autonomousCommand;
  private static final String kCenterDepotAuto = "Center Depot Auto";
  private static final String kTopDepotAuto = "Top Depot Auto";
  private static final String kBottomDepotAuto = "Bottom Depot Auto";
  private static final String kBottomAuto = "Bottom Auto";
  private static final String kBottomBumpAuto = "Bottom Bump Auto";
  private static final String kCenterAuto = "Center Auto";
  private static final String kTopAuto = "Top Auto";
  private static final String kTopBumpAuto = "Top Bump Auto";
  private static final String kDefaultAuto = "No Auto";

  @Logged(name = "autonomousCommandName")
  public String getAutonomousCommand() {
    return chooser.getSelected();
  }

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    chooser.addOption("Center Depot Auto", kCenterDepotAuto);
    chooser.addOption("Top Depot Auto", kTopDepotAuto);
    chooser.addOption("Bottom Depot Auto", kBottomDepotAuto);
    chooser.addOption("Bottom Auto", kBottomAuto);
    chooser.addOption("Center Auto", kCenterAuto);
    chooser.addOption("Top Auto", kTopAuto);
    chooser.addOption("Bottom Bump Auto", kBottomBumpAuto);
    chooser.addOption("Top Bump Auto", kTopBumpAuto);
    chooser.addOption("No Auto", kDefaultAuto);

    SmartDashboard.putData("Auto choices", chooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoSelected = chooser.getSelected();
    System.out.println("Auto selected: " + autoSelected);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    switch (autoSelected) {
      case kCenterDepotAuto:
      case kTopDepotAuto:
      case kBottomDepotAuto:
      case kBottomAuto:
      case kCenterAuto:
      case kTopAuto:
      case kBottomBumpAuto:
      case kTopBumpAuto:
        break;
    }
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
