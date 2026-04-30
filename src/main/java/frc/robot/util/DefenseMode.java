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

public static boolean isRedAllianceLine(Pose2d pose){
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
    if (isRedAllianceLine(pose)) {line = DefenseLine.RedAlliance;}
    else if(isBlueAllianceLine(pose)) {line = DefenseLine.BlueAlliance;}
    else if(isRedNeutralLine(pose)) {line = DefenseLine.RedNeutral;}
    else if(isBlueNeutralLine(pose)) {line = DefenseLine.BlueNeutral;}
    return line;
}

public static boolean isDefenseLine(Pose2d pose) {
    return getDefenseLine(pose) != DefenseLine.None;
}

public static AllianceBasedLine getAllianceBasedLine(DefenseLine line){
    AllianceBasedLine allianceBasedLine = AllianceBasedLine.None;
    if (MyAlliance.isBlue()) {
        switch (line) {
        case BlueAlliance: allianceBasedLine = AllianceBasedLine.Alliance;
        case RedAlliance: allianceBasedLine = AllianceBasedLine.OppositeAlliance;
        case BlueNeutral: allianceBasedLine = AllianceBasedLine.Neutral;
        case RedNeutral: allianceBasedLine = AllianceBasedLine.OppositeNeutral;
        case None: allianceBasedLine = AllianceBasedLine.None;
        default: allianceBasedLine = AllianceBasedLine.None;
        };
    } else {
        switch (line) {
        case BlueAlliance: allianceBasedLine = AllianceBasedLine.OppositeAlliance;
        case RedAlliance: allianceBasedLine = AllianceBasedLine.Alliance;
        case BlueNeutral: allianceBasedLine = AllianceBasedLine.OppositeNeutral;
        case RedNeutral: allianceBasedLine = AllianceBasedLine.Neutral;
        case None: allianceBasedLine = AllianceBasedLine.None;
        default: allianceBasedLine = AllianceBasedLine.None;
    };
}
return allianceBasedLine;
}

public static double defenseLineClamp(double velocity, AllianceBasedLine line) {
    double processedVelocity = velocity;
    if (line == AllianceBasedLine.OppositeAlliance || line == AllianceBasedLine.Neutral) {
        processedVelocity = MathUtil.clamp(processedVelocity, 0, DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
    } else if (line == AllianceBasedLine.Alliance || line == AllianceBasedLine.OppositeNeutral) {
        processedVelocity = MathUtil.clamp(processedVelocity, -DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond), 0);
    }
    return processedVelocity;
}

public static double defenseClamp(double velocity, Pose2d pose) {
    return defenseLineClamp(velocity, getAllianceBasedLine(getDefenseLine(pose)));
}

}
