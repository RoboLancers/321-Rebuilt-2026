/* (C) Robolancers 2026 */
package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

public class TunableConstant {

  private final DoubleSupplier getter;

  public TunableConstant(String key, double defaultValue) {
    // if and only if no constant is there already, set it to the default value
    SmartDashboard.putNumber("/Tuning" + key, SmartDashboard.getNumber(key, defaultValue));

    this.getter = () -> SmartDashboard.getNumber("/Tuning" + key, defaultValue);
  }

  public double get() {
    return this.getter.getAsDouble();
  }
}