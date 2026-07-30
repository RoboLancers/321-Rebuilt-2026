/* (C) RoboLancers 2026 */
package frc.robot.subsystems.questNav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class QuestNavConstants {

  public static final Transform3d kRobotToQuest =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  // Translation: ~5cm accuracy is realistic for Quest VIO.
  // Rotation: tight to make Quest the primary heading source over the Pigeon2.
  // Increase rotation value (e.g. 0.05) to let the Pigeon2 compete more.
  public static final Matrix<N3, N1> kQuestStdDev = VecBuilder.fill(0.05, 0.05, 0.01);

  // Switch to "true" during competitions
  public static final boolean kQuestVersionCheck = false;

  public static final double kQuestCriticalPercent = 10;
}
