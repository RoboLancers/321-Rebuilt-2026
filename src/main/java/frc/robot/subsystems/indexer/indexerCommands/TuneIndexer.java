/* (C) RoboLancers 2026 */
package frc.robot.subsystems.indexer.indexerCommands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.util.TunableConstant;

public class TuneIndexer extends Command {
  Indexer indexer;
  TunableConstant kP = new TunableConstant("Indexer/kP/", 0);
  TunableConstant kD = new TunableConstant("Indexer/kD/", 0);
  TunableConstant kV = new TunableConstant("Indexer/kV/", 0);
  TunableConstant kS = new TunableConstant("Indexer/kS/", 0);
  TunableConstant targetSpeed = new TunableConstant("Indexer/targetSpeed", 0);

  public TuneIndexer(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void execute() {
    indexer.tune(kP.get(), kD.get(), kV.get(), kS.get(), RPM.of(targetSpeed.get()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setVoltage(Volts.of(0));
  }
}
