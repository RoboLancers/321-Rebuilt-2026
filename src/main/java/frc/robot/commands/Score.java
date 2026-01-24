package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.subsystems.outtake.commands.OuttakeFuel;
import frc.robot.util.MyAlliance;

public class Score {
    
  private static final Pose2d redHubPose = new Pose2d(0,0,Rotation2d.kZero);
  private static final Pose2d blueHubPose = new Pose2d(0,0,Rotation2d.kZero);

  private static final Distance region1 = Meters.of(0);
  private static final Distance region2 = Meters.of(0);
  private static final Distance region3 = Meters.of(0);
 
    public static Command scoreFuelFromPose(Drivetrain drivetrain, Outtake outtake){
    return Align.driveToHubScoringPose(drivetrain).andThen(OuttakeFuel.scoreSetPosition(outtake));
  }

  public static Distance getHubDistance(Drivetrain drivetrain){
    Pose2d hubPose = null;

    if (MyAlliance.isRed()){
      hubPose = redHubPose;
    } 
    else {
      hubPose = blueHubPose;}
      
      return Meters.of(drivetrain.getPose().getTranslation().getDistance(hubPose.getTranslation()));
  }

  public static AngularVelocity getScoreVelocity(Drivetrain drivetrain){
    AngularVelocity velocity = RPM.of(0);
    if(0<getHubDistance(drivetrain).in(Meters) && getHubDistance(drivetrain).in(Meters)<region1.in(Meters)){
      velocity = OuttakeConstants.kRegion1ScoreRPM;
    } 
    else if (region1.in(Meters)<getHubDistance(drivetrain).in(Meters) && getHubDistance(drivetrain).in(Meters)<region2.in(Meters)){
      velocity = OuttakeConstants.kRegion2ScoreRPM;
    }
    else if (region2.in(Meters)<getHubDistance(drivetrain).in(Meters)){
      velocity = OuttakeConstants.kRegion3ScoreRPM;
    }

    return velocity;
  }
  
  public static Command scoreFuelFromAnywhere(){}

  public static Command scoreFuelWhileDriving(){}

  


}
