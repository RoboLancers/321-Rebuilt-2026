package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ClimbConstants {

    public static final int kClimbMotorId = 1; 
    
    public static final double kClimbStatorLimit = 1;

    public static final double kClimbSupplyLimit = 1;

    public static final NeutralModeValue kClimbNeutralMode = NeutralModeValue.Brake;

    public static final boolean kClimbInverted = false;

    public static final double kClimbCruiseVelocity = 0;

    public static final double kClimbAcceleration = 0;

    public static final GravityTypeValue kClimbGravityType = GravityTypeValue.Elevator_Static;

    public static final StaticFeedforwardSignValue kClimbFeedForwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    
}
