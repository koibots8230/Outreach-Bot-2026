package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

public class ShootingCommands {
  public static Command startIntake(Spindexer spindexer, TowerIndexer towerIndexer) {
    return Commands.parallel(
        spindexer.setVelocityCommand(SpindexerConstants.SHOOTING_VELOCITY),
        towerIndexer.setVelocityCommand(TowerIndexerConstants.VELOCITY));
  }
}
