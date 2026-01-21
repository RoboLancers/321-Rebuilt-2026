package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class PivotConstants {
    
public static final int kPivotMotorId = 1;

public static final boolean kPivotMotorInverted = false;

public static final Current kPivotMotorCurrentLimit = Amps.of(6);

public static final LinearVelocity kPivotMotorMaxVelocity = MetersPerSecond.of(1);

public static final LinearAcceleration kPivotMotorMaxAcceleration = MetersPerSecondPerSecond.of(1);

public static final Angle kStartingAngle = Degrees.of(60);

public static final Angle kAngleTolerance = Degrees.of(1);

}
