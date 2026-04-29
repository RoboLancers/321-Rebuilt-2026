package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

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

public static DefenseLine getDefenseMode(Pose2d pose) {
    DefenseLine line = DefenseLine.None;
    if (isRedAllianceLine(pose)) {line = DefenseLine.RedAlliance;}
    else if(isBlueAllianceLine(pose)) {line = DefenseLine.BlueAlliance;}
    else if(isRedNeutralLine(pose)) {line = DefenseLine.RedNeutral;}
    else if(isBlueNeutralLine(pose)) {line = DefenseLine.BlueNeutral;}
    return line;
}

public static AllianceBasedLine getAllianceBasedLine(DefenseLine line){
    AllianceBasedLine allianceBasedLine = AllianceBasedLine.None;
    if (MyAlliance.isBlue()) {
        line = (switch) 
        case 
    }

}

public static double defenseClamp(double velocity, DefenseLine line) {
    double processedVelocity = velocity;
    if (line == DefenseLine.BlueAlliance || line == DefenseLine.RedNeutral) {
        processedVelocity = MathUtil.clamp()
    }
}

}
