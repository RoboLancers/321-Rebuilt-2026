/* (C) RoboLancers 2026 */
package frc.robot.subsystems.vision;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

public class CameraStatusLED {

  public enum StatusType {
    Detected,
    NotDetected,
    Error
  }

  public SolidColor solidColor;
  private final RGBWColor kDetectedColor = new RGBWColor(191, 64, 191, 0); // purple
  private final RGBWColor kErrorColor = new RGBWColor(255, 255, 255, 0); // red
  private final RGBWColor kDefaultColor = new RGBWColor(0, 0, 0, 255); // white
  private RGBWColor statusColor = kDefaultColor;
  private CANdle candle;
  private int LEDStartIndex;
  private int LEDEndIndex;

  public CameraStatusLED(CANdle candle, int LEdStartIndex, int LEDEndIndex) {
    this.candle = candle;
    this.LEDEndIndex = LEdStartIndex;
    this.LEDEndIndex = LEDEndIndex;
    this.solidColor = new SolidColor(LEDStartIndex, LEDEndIndex);
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
    solidColor.withColor(statusColor);
  }

  public RGBWColor getStatusColor() {
    return this.statusColor;
  }

  public String getStatusColorHex() {
    return this.statusColor.toHexString();
  }
}
