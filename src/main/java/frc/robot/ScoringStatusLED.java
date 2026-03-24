/* (C) RoboLancers 2026 */
package frc.robot;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;

public class ScoringStatusLED {

  public CANdle candle;
  public ControlRequest ledRequest;
  public int kLEDStartIndex;
  public int kLEDEndIndex;

  public Color kAtVelocityColor = new Color(0, 255, 0); //green
  public Color kAlignedColor = new Color(0, 255, 0); //green
    public Color kErrorColor = new Color(255,0,0);

  public Color statusColor = kErrorColor;

  public SolidColor kErrorRequest = new SolidColor(kLEDStartIndex, kLEDEndIndex).withColor(new RGBWColor(kErrorColor));


  public SolidColor kAtVelocity =
      new SolidColor(kLEDStartIndex, kLEDEndIndex).withColor(new RGBWColor(kAtVelocityColor));
  public StrobeAnimation kAligned =
      new StrobeAnimation(kLEDStartIndex, kLEDEndIndex).withColor(new RGBWColor(kAlignedColor));

  public enum StatusType {
    Aligned,
    AtVelocity,
    Error
  }

  public ScoringStatusLED(CANdle candle, int kLEDStartIndex, int kLEDEndIndex) {
    this.candle = candle;
    this.kLEDStartIndex = kLEDStartIndex;
    this.kLEDEndIndex = kLEDEndIndex;
    if (!(candle == null)) {
      candle.setControl(ledRequest);
    }
  }

  public void setLEDRequest(StatusType type) {
    switch (type) {
      case Aligned:
        ledRequest = kAligned;
        statusColor = kAlignedColor;
        break;
      case AtVelocity:
        ledRequest = kAtVelocity;
        statusColor = kAtVelocityColor;
        break;
    default:
        ledRequest = kErrorRequest;
        statusColor = kErrorColor;

        break;
    }
    if (!(candle == null)) {
      candle.setControl(ledRequest);
    }
  }

  public Color getStatusColor() {
    return statusColor;
  }

  public String getStatusColorHex() {
    return statusColor.toHexString();
  }
}
