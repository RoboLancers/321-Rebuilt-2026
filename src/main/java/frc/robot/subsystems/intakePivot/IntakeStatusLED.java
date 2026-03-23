/* (C) RoboLancers 2026 */
package frc.robot.subsystems.intakePivot;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;

public class IntakeStatusLED {

  private boolean intakeOut;
  private CANdle candle;

  private int kLEDStartIndex;
  private int kLEDEndIndex;
  private Color statusColor;

  private Color kDefaultColor = new Color(0, 0, 0);
  private Color kIntakeOutColor = new Color(255, 255, 0);

  public IntakeStatusLED(CANdle candle, int kLEDStartIndex, int kLEDEndIndex) {
    this.candle = candle;
    this.kLEDStartIndex = kLEDStartIndex;
    this.kLEDEndIndex = kLEDEndIndex;

    if (!(candle == null)) {
      candle.setControl(
          new SolidColor(kLEDStartIndex, kLEDEndIndex).withColor(new RGBWColor((kDefaultColor))));
    }
  }

  public void updateStatusColor(boolean intakeOut) {
    statusColor = intakeOut ? kIntakeOutColor : kDefaultColor;
    if (!(candle == null)) {
      candle.setControl(
          new SolidColor(kLEDStartIndex, kLEDEndIndex).withColor(new RGBWColor((kDefaultColor))));
    }
  }

  public Color getStatusColor() {
    return statusColor;
  }

  public String getStatusColorHex() {
    return statusColor.toHexString();
  }
}
