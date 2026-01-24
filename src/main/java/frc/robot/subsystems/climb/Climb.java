package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climbCommands.setVoltageWithFeedForward;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Climb extends SubsystemBase{

    double kP;

    double kD;

    double kG;

    private TalonFX climbMotor = new TalonFX(ClimbConstants.kClimbMotorId);

    private ArmFeedforward climbFeedforward = new ArmFeedforward(0, kG, 0, 0, 0);

    private PIDController climbController = new PIDController(kP, kG, kD);

    public Climb create(){
        return new Climb();
    }

    public Climb(){
        configureMotors();
        setClimbPID(kP, 0.0, kD, 0.0, 0.0, 0.0, kG);
    }

    public void configureMotors(){

        TalonFXConfiguration climbMotorConfiguration =
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(ClimbConstants.kClimbStatorLimit)
                        .withSupplyCurrentLimit(ClimbConstants.kClimbSupplyLimit))
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(ClimbConstants.kClimbNeutralMode)
                        .withInverted(ClimbConstants.kClimbInverted
                            ?InvertedValue.Clockwise_Positive
                            :InvertedValue.CounterClockwise_Positive
                            ))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicAcceleration(0))
                .withSlot0(
                    new Slot0Configs()
                        .withGravityType(ClimbConstants.kClimbGravityType)
                        .withStaticFeedforwardSign(ClimbConstants.kClimbFeedForwardSign));
                        
                climbMotor.getConfigurator().apply(climbMotorConfiguration);
    }    

    public void goToAngle(Angle angle) {
        double volts =
        climbController.calculate(/*replace with yaw from gyro*/getAngle().in(Degrees), angle.in(Degrees)) + 
        climbFeedforward.calculate(/*replace with yaw from gyro*/0,0);
        climbMotor.setVoltage(volts);
    }

    public Angle getAngle() {
        Angle angle = Degrees.of(climbMotor.getPosition().getValueAsDouble());
        return angle;
    }

    public void setClimbPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        climbMotor.getConfigurator().apply(
            new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withKG(kG)
        );
    }

    

}