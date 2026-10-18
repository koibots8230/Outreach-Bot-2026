package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.Spindexer;

public class IntakeCommands {
  public static Command startIntake(Spindexer spindexer) {
    return spindexer.setVelocityCommand(SpindexerConstants.INTAKE_VELOCITY);
  }

  public static Command reverseIntake(Spindexer spindexer) {
    return spindexer.setVelocityCommand(SpindexerConstants.REVERSE_VELOCITY);
  }

  public static Command stopIntake(Spindexer spindexer) {
    return spindexer.setVelocityCommand(RPM.of(0));
  }
}
