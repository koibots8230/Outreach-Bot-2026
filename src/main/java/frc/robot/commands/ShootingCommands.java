package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

public class ShootingCommands {
  public static Command shoot(Spindexer spindexer, TowerIndexer towerIndexer, Shooter shooter) {
    return Commands.sequence(
      shooter.setVelocityCommand(ShooterConstants.SHOOT_SPEED),
      Commands.waitUntil(shooter::atSpeed),
      Commands.parallel(
        spindexer.setVelocityCommand(SpindexerConstants.SHOOTING_VELOCITY),
        towerIndexer.setVelocityCommand(TowerIndexerConstants.VELOCITY)));
  }

  public static Command stopShoot(Spindexer spindexer, TowerIndexer towerIndexer, Shooter shooter) {
    return Commands.parallel(
        shooter.setVelocityCommand(RPM.of(0)),
        spindexer.setVelocityCommand(RPM.of(0)),
        towerIndexer.setVelocityCommand(RPM.of(0)));
  }
}
