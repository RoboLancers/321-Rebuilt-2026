package frc.robot.subsystems.outtake.commands;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;

public class OuttakeFuel {
    
public static Command outtakeWithVoltage(Outtake outtake, Supplier<Voltage> volts){
    return outtake.runVolts(volts.get());
}

public static Command outtakeWithVelocity(Outtake outtake, Supplier<AngularVelocity> rpm){
    return outtake.setControl(rpm.get());
}

public static Command feedNeutralZone(Outtake outtake){
    return outtakeWithVelocity(outtake, ()-> OuttakeConstants.kNeutralFeedRPM);
}

public static Command feedOppositeZone(Outtake outtake){
    return outtakeWithVelocity(outtake, ()-> OuttakeConstants.kOppositeFeedRPM);
}

public static Command scoreRegion1(Outtake outtake){
    return outtakeWithVelocity(outtake, ()-> OuttakeConstants.kRegion1ScoreRPM);
}

public static Command scoreRegion2(Outtake outtake){
    return outtakeWithVelocity(outtake, ()-> OuttakeConstants.kRegion2ScoreRPM);
}

public static Command scoreRegion3(Outtake outtake){
    return outtakeWithVelocity(outtake, ()-> OuttakeConstants.kRegion3ScoreRPM);
}

public static Command scoreSetPosition(Outtake outtake){
    return outtakeWithVelocity(outtake, ()-> OuttakeConstants.kSetPositionScoreRPM);
}

public static Command releaseFuel(Outtake outtake){
    return outtakeWithVelocity(outtake, ()-> OuttakeConstants.kReleaseRPM);
}

}
