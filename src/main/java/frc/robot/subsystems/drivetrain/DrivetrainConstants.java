/* (C) Robolancers 2026 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

@Logged
public class DrivetrainConstants {
  public record AutoGains(double kP, double kI, double kD) {}

  public static final AutoGains kTranslationGains = new AutoGains(6.328, 0, 0.01); // real (TODO: TUNE)

  public static final Constraints kTranslationConstraints = new Constraints(4.5, 5);

  public static final AutoGains kHeadingGains = new AutoGains(3.14, 0, 0);

  public static final Constraints kHeadingConstraints =
      RobotBase.isReal() ? new Constraints(6 * Math.PI, 12) : new Constraints(2 * Math.PI, 8);

  public static final AutoGains kTuneTranslationGains = new AutoGains(0, 0, 0); // isn't used
  public static final AutoGains kTuneHeadingGains =
      new AutoGains(6, 0, 0.01); // for heading controller

  public static final PIDConstants kAutoTranslationGains = new PIDConstants(5, 0, 0);
  public static final PIDConstants kAutoHeadingGains = new PIDConstants(5, 0, 0);

  public static final Distance kTrackWidth = Inches.of(29);
  public static final Distance kWheelBase = Inches.of(29);

  public static final double kDriveDeadband = 0.05;
  public static final double kRotationDeadband = 0.05;
  public static final AngularVelocity kMaxAngularVelocity = RadiansPerSecond.of(6);
  public static final LinearVelocity kMaxLinearVelocity =
      MetersPerSecond.of(5.0); // TunerConstants.kSpeedAt12Volts

  public static final Distance kAlignmentSetpointTranslationTolerance = Meters.of(0.01);
  public static final Angle kAlignmentSetpointRotationTolerance = Degrees.of(1.9); // 2.0

  public static final Distance kAlignmentVelocityRadius =
      Meters.of(
          0.45); // distance at which to start changing weights from velocity control to PID when
  // aligning
  public static final Distance kAlignmentPIDRadius =
      Meters.of(0.1); // distance at which to end changing weights from velocity control to PID when
  // aligning

  public static final double kMaxPathVelocity = 3.0;
  public static final double kMaxPathAcceleration = 4.0;
  public static final double kMaxPathAngularVelocity = 540;
  public static final double kMaxPathAngularAcceleration = 720;
}
