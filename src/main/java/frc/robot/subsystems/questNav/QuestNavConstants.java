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
      new Transform3d(new Translation3d(), new Rotation3d());
  public static final Matrix<N3, N1> kQuestStdDev = VecBuilder.fill(0.1, 0.1, 0.1);
  public static final boolean kQuestVersionCheck = false;
  public static final double kQuestCriticalPercent = 10;
}
