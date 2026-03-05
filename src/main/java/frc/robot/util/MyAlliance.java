/* (C) RoboLancers 2026 */
package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class MyAlliance {

  // Default alliance is Blue alliance for FMS and Pathplanner etc
  public static boolean isBlue() {
    Optional<Alliance> myAlliance = DriverStation.getAlliance();
    if (!myAlliance.isPresent()) {
      return true;
    }

    return myAlliance.get() == DriverStation.Alliance.Blue;
  }

  public static boolean isRed() {
    return isBlue() == false;
  }
}
