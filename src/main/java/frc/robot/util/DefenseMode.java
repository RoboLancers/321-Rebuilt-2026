/* (C) RoboLancers 2026 */
package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

public class DefenseMode {

  public enum DefenseLine {
    RedAlliance,
    RedNeutral,
    BlueAlliance,
    BlueNeutral,
    None
  }

  public enum AllianceBasedLine {
    Alliance,
    Neutral,
    OppositeAlliance,
    OppositeNeutral,
    None
  }

  public static boolean isRedAllianceLine(Pose2d pose) {
    return RebuiltUtil.inRedAllianceZone(pose) && !RebuiltUtil.inRedDefenseZone(pose);
  }

  public static boolean isBlueAllianceLine(Pose2d pose) {
    return RebuiltUtil.inBlueAllianceZone(pose) && !RebuiltUtil.inBlueDefenseZone(pose);
  }

  public static boolean isRedNeutralLine(Pose2d pose) {
    return RebuiltUtil.inRedNeutralZone(pose) && !RebuiltUtil.inRedNeutralDefenseZone(pose);
  }

  public static boolean isBlueNeutralLine(Pose2d pose) {
    return RebuiltUtil.inBlueNeutralZone(pose) && !RebuiltUtil.inBlueNeutralDefenseZone(pose);
  }

  public static DefenseLine getDefenseLine(Pose2d pose) {
    DefenseLine line = DefenseLine.None;
    // is red alliance
    // get red lines
    // flip logic for blue
    // default None enum
    if (isRedAllianceLine(pose)) {
      line = DefenseLine.RedAlliance;
    } else if (isBlueAllianceLine(pose)) {
      line = DefenseLine.BlueAlliance;
    } else if (isRedNeutralLine(pose)) {
      line = DefenseLine.RedNeutral;
    } else if (isBlueNeutralLine(pose)) {
      line = DefenseLine.BlueNeutral;
    }
    return line;
  }

  public static boolean isDefenseLine(Pose2d pose) {
    return getDefenseLine(pose) != DefenseLine.None;
  }

  public static AllianceBasedLine getAllianceBasedLine(DefenseLine line) {

    if (MyAlliance.isBlue()) {
      switch (line) {
        case BlueAlliance:
          return AllianceBasedLine.Alliance;
        case RedAlliance:
          return AllianceBasedLine.OppositeAlliance;
        case BlueNeutral:
          return AllianceBasedLine.Neutral;
        case RedNeutral:
          return AllianceBasedLine.OppositeNeutral;
        case None:
        default:
          return AllianceBasedLine.None;
      }
    } else {
      switch (line) {
        case BlueAlliance:
          return AllianceBasedLine.OppositeAlliance;
        case RedAlliance:
          return AllianceBasedLine.Alliance;
        case BlueNeutral:
          return AllianceBasedLine.OppositeNeutral;
        case RedNeutral:
          return AllianceBasedLine.Neutral;
        case None:
        default:
          return AllianceBasedLine.None;
      }
    }
  }

  public static double defenseLineClamp(double velocity, AllianceBasedLine line) {
    double processedVelocity = velocity;

    // Cant go backwards if on opposite alliance line or our neutral line
    if (line == AllianceBasedLine.OppositeAlliance || line == AllianceBasedLine.Neutral) {
      processedVelocity =
          MathUtil.clamp(
              processedVelocity, 0, DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
    } else if (line == AllianceBasedLine.Alliance || line == AllianceBasedLine.OppositeNeutral) {
      // cant go forwardds if on our alliance line or the opposing sides neutral line
      processedVelocity =
          MathUtil.clamp(
              processedVelocity, -DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond), 0);
    }
    // otherwise you are just good
    return processedVelocity;
  }

  public static double defenseClampVelocity(double velocity, Pose2d pose) {
    return defenseLineClamp(velocity, getAllianceBasedLine(getDefenseLine(pose)));
  }
}
