package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShootCommands {
  // this file would have indexer stuff too but this is just shooter rn
  public static Command reverseCommand(Shooter shooter) {
    return Commands.sequence(shooter.setVelocityCommand(ShooterConstants.REVERSE_SPEED));
  }

  public static Command shootHigh(Shooter shooter) {
    return Commands.sequence(
        shooter.setVelocityCommand(ShooterConstants.HIGH_GOAL_SPEED),
        Commands.waitSeconds(1.2)); // ,
    // indexer.setSpeedCommand(IndexerConstants.SHOOT_SPEED).repeatedly());
  }

  public static Command shootLow(Shooter shooter) {
    return Commands.sequence(
        shooter.setVelocityCommand(ShooterConstants.LOW_GOAL_SPEED), Commands.waitSeconds(1)); // ,
    // indexer.setSpeedCommand(IndexerConstants.SHOOT_SPEED).repeatedly());
  }

  public static Command stop(Shooter shooter) {
    return shooter.setVelocityCommand(RPM.of(0));
  }
}
