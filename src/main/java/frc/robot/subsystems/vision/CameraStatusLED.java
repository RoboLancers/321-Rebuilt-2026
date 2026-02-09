/* (C) RoboLancers 2026 */
package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.util.Color;

public class CameraStatusLED {

  public enum StatusType {
    Detected,
    NotDetected,
    Error
  }

  private final Color kDetectedColor = new Color(191, 64, 191); // purple
  private final Color kErrorColor = new Color(255, 255, 255); // red
  private final Color kDefaultColor = new Color(255, 0, 0); // white
  private Color statusColor = kDefaultColor;

  public CameraStatusLED(int ledPort) {}

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
  }

  public Color getStatusColor() {
    return this.statusColor;
  }

  public String getStatusColorHex() {
    return this.statusColor.toHexString();
  }
}
