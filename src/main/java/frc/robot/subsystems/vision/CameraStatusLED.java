/* (C) RoboLancers 2026 */
package frc.robot.subsystems.vision;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;

public class CameraStatusLED {

  public enum StatusType {
    Detected,
    NotDetected,
    Error
  }

  private final Color kDetectedColor = new Color(191, 64, 191); // purple
  private final Color kErrorColor = new Color(255, 0, 0); // red
  private final Color kDefaultColor = new Color(255, 255, 255); // white
  private Color statusColor = kDefaultColor;
  private CANdle candle;
  private int LEDStartIndex;
  private int LEDEndIndex;

  public CameraStatusLED(CANdle candle, int LEDStartIndex, int LEDEndIndex) {
    this.candle = candle;
    this.LEDStartIndex = LEDStartIndex;
    this.LEDEndIndex = LEDEndIndex;
    statusColor = kDefaultColor;

    if (candle != null) {
      this.candle.setControl(
          new SolidColor(LEDStartIndex, LEDEndIndex).withColor(new RGBWColor(statusColor)));
    }
  }

  public void updateStatusColor(StatusType type) {
    switch (type) {
      case Error:
        statusColor = kErrorColor;
        break;
      case Detected:
        statusColor = kDetectedColor;
        break;
      case NotDetected:
      default:
        statusColor = kDefaultColor;
        break;
    }
    if (candle != null) {
      this.candle.setControl(
          new SolidColor(LEDStartIndex, LEDEndIndex).withColor(new RGBWColor(statusColor)));
    }
  }

  public Color getStatusColor() {
    return this.statusColor;
  }

  public String getStatusColorHex() {
    return this.statusColor.toHexString();
  }
}
