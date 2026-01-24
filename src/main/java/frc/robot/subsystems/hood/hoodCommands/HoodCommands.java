package frc.robot.subsystems.hood.hoodCommands;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;

public class HoodCommands {

    public static Command goToAngle(Hood hood, Supplier<Angle> angle){
        return Commands.run(
            ()->hood.goToAngle(angle.get())
        );
    }

    public static Command goToScoringAngle(Hood hood){
        return goToAngle(hood,()->HoodConstants.kSetScoreAngle);
    }

    public static Command goToRegion1Angle(Hood hood){
        return goToAngle(hood, ()->HoodConstants.kRegion1ScoreAngle);
    }

    
    public static Command goToRegion2Angle(Hood hood){
        return goToAngle(hood, ()->HoodConstants.kRegion2ScoreAngle);
    }

    
    public static Command goToRegion3Angle(Hood hood){
        return goToAngle(hood, ()->HoodConstants.kRegion3ScoreAngle);
    }

    
    public static Command goToNeutralFeedAngle(Hood hood){
        return goToAngle(hood, ()->HoodConstants.kNeutralFeedAngle);
    }

    
    public static Command goToOppositeFeedAngle(Hood hood){
        return goToAngle(hood, ()->HoodConstants.kOppositeFeedAngle);
    }

    
    public static Command goToReleaseAngle(Hood hood){
        return goToAngle(hood, ()->HoodConstants.kReleaseAngle);
    }
    
    public static Command goToTravelAngle(Hood hood){
        return goToAngle(hood, ()->HoodConstants.kTravelAngle);
    }

}