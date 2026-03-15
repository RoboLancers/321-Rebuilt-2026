/* (C) RoboLancers 2026 */
package frc.robot.util;

public class Parity {

  public static boolean isEven(double a) {
    boolean isEven;
    if (Math.round(a / 2) == a / 2) {
      isEven = true;
    } else {
      isEven = false;
    }
    return isEven;
  }

  public static boolean isOdd(double a) {
    return !isEven(a);
  }
}
